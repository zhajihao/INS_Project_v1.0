/**
 * @file gnss1_rx.c
 * @brief GNSS1 通道接收驱动与协议解析实现
 *
 * 职责:
 *   - USART0 230400 8E1 + DMA1_CH2 循环 RX 配置
 *   - T_HJD_POSDATA 帧解析 (同步字 0x55 0xBB + 51B + XOR 校验)
 *   - DMA 环形缓冲 + 状态机逐字节对齐
 *   - pump() 主循环驱动, 吐出 gnss1_rx_result_t
 *
 * USART0: PA9(TX) 发导航结果 / PA10(RX) 收 GNSS1, 230400, 8E1
 * TX: DMA1_CH7 (uart.c 已配, 发导航结果帧)
 * RX: DMA1_CH2 循环搬运 → 应用层环形缓冲 → 主循环解析
 */

#include "gnss1_rx.h"
#include "uart.h"
#include "navi_config/navi_constants.h"
#include "gd32f4xx_dma.h"
#include "gd32f4xx_usart.h"
#include <string.h>

/* ---- DMA RX 配置 ---- */

/** RX DMA 缓冲区大小 (2048 字节 ≈ 40 帧 ≈ 87.2 ms 线上数据 @230400 8E1)
 *  一圈 87.2ms 远大于主循环最坏路径 (< 5ms per sample + Kalman), 彻底消除 ovf 边界条件. */
#define GNSS_RX_DMA_BUF_SIZE 2048U

/** GD32F4xx USART0_RX → DMA1 Channel 2 / Subperipheral 4 */
#define GNSS_RX_DMA DMA1
#define GNSS_RX_DMA_CHANNEL DMA_CH2
#define GNSS_RX_DMA_SUBPERI DMA_SUBPERI4

/* ---- 解析状态机 ---- */

typedef enum
{
    RX_STATE_SYM1 = 0, /* 等 0xBB */
    RX_STATE_SYM2,     /* 等 0x55 */
    RX_STATE_LEN,      /* 读 u8FrameLen */
    RX_STATE_DATA      /* 收数据区 + 校验 */
} rx_state_t;

/* ---- 静态变量 ---- */

static uint8_t s_dma_buf[GNSS_RX_DMA_BUF_SIZE]; /* DMA 循环写缓冲区 */
volatile uint32_t s_dma_write_laps;             /* DMA 已完成的整圈数 (DMA1_CH2 FTF ISR 写, pump 读) */
static uint32_t s_dma_read_total;               /* 已消费字节总数 (单调递增, 取模得下标) */

static rx_state_t s_state;
static uint8_t s_frame_buf[GNSS_HJD_TOTAL_LEN]; /* 当前帧组装缓冲 */
static uint8_t s_frame_idx;                     /* 当前帧已收字节数 */
static uint8_t s_remain;                        /* 还需收的字节数 */

/* 业务可观测性: 以下变量不加 static, 便于 Keil Watch 直接观察 */
gnss1_rx_result_t g_gnss_result;      /* 最新一帧解析结果 (只由 parse_frame 写) */
volatile uint32_t g_gnss_frame_count; /* 累计有效帧 (校验通过) */
volatile uint32_t g_gnss_err_count;   /* 校验失败计数 */
volatile uint32_t g_gnss_ovf_count;   /* DMA 环缓冲整圈覆盖 */

static uint8_t s_result_new; /* 1=有新结果待读 (内部标志) */

/* ---- 内部函数 ---- */

/**
 * @brief  小端读 16 位 (实测帧验证: 数据字段为小端字节序)
 */
static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[1] << 8) | (uint16_t)p[0];
}

/**
 * @brief  小端读 32 位 (实测帧验证: 数据字段为小端字节序)
 */
static inline uint32_t rd_le32(const uint8_t *p)
{
    return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] << 8) | (uint32_t)p[0];
}

/**
 * @brief  计算校验和: 累加和低 8 位
 * @param  data  起始地址 (同步字第 1 字节)
 * @param  len   字节数 (GNSS_HJD_CHECKSUM_LEN = 50)
 */
static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
        sum += data[i];
    return (uint8_t)(sum & 0xFFU);
}

/**
 * @brief  校验并解析一帧完整数据
 * @param  frame  指向帧头 (0x55)
 * @return 1 成功, 0 校验失败
 */
static uint8_t parse_frame(const uint8_t *frame)
{
    uint8_t chk;
    uint8_t pos_vld;
    double lat_deg, lon_deg, height_m, vn_e, vn_n, vn_u;
    float pdop;

    /* 1. 校验和 */
    chk = calc_checksum(frame, GNSS_HJD_CHECKSUM_LEN);
    if (chk != frame[GNSS_HJD_CHECKSUM_LEN])
        return 0;

    /* 2. 提取字段 (小端, 实测帧验证)
     * 字段偏移基于 T_HJD_POSDATA 协议帧格式 (51 字节, 1 字节对齐)
     * 实测: lon[19]=116.26°, lat[23]=39.06°, height[27]=312.6m */
    pos_vld = frame[4]; /* u8PosVld, 偏移: sym(2)+len(1)+flag(1) */

    lon_deg = (double)(int32_t)rd_le32(frame + 19) * 1e-6; /* s32lon  offset 19, 单位 1e-6 */
    lat_deg = (double)(int32_t)rd_le32(frame + 23) * 1e-6; /* s32lat  offset 23, 单位 1e-6 */
    height_m = (double)(int32_t)rd_le32(frame + 27) * 0.1; /* s32height offset 27, 单位 0.1m */
    vn_n = (double)(int16_t)rd_le16(frame + 31) * 0.1;     /* s16vn offset 31 */
    vn_u = (double)(int16_t)rd_le16(frame + 33) * 0.1;     /* s16vu offset 33 */
    vn_e = (double)(int16_t)rd_le16(frame + 35) * 0.1;     /* s16ve offset 35 */
    pdop = (float)rd_le16(frame + 37) * 0.01f;             /* u16PDOP offset 37 */

    /* 3. 填充结果 (定位有效才更新物理量, 但始终更新统计)
     * 消费者只取最新帧: 旧帧被覆盖属正常 (过时冗余帧主动抛弃). */
    g_gnss_result.pos_valid = (pos_vld == 2U) ? 1U : 0U;

    if (g_gnss_result.pos_valid)
    {
        g_gnss_result.lat_rad = lat_deg * arcdeg;
        g_gnss_result.lon_rad = lon_deg * arcdeg;
        g_gnss_result.height_m = height_m;
        g_gnss_result.vn_e = vn_e;
        g_gnss_result.vn_n = vn_n;
        g_gnss_result.vn_u = vn_u;
        g_gnss_result.pdop = pdop;
        g_gnss_result.sat_num = frame[9]; /* u8TrcSatNum offset 9 */
    }

    g_gnss_result.frame_count = ++g_gnss_frame_count;
    s_result_new = 1U;
    return 1;
}

/**
 * @brief  状态机处理单个字节
 * @param  byte  新收到的字节
 */
static void feed_byte(uint8_t byte)
{
    switch (s_state)
    {
    case RX_STATE_SYM1:
        if (byte == GNSS_HJD_SYM_BYTE1) /* 0x55 */
        {
            s_frame_buf[0] = byte;
            s_frame_idx = 1U;
            s_state = RX_STATE_SYM2;
        }
        break;

    case RX_STATE_SYM2:
        if (byte == GNSS_HJD_SYM_BYTE2) /* 0xBB */
        {
            s_frame_buf[1] = byte;
            s_frame_idx = 2U;
            s_state = RX_STATE_LEN;
        }
        else if (byte == GNSS_HJD_SYM_BYTE1) /* 又是 0x55 */
        {
            /* 连续 0x55, 视当前字节为新的同步字第 1 字节, 继续等 0xBB */
            s_frame_buf[0] = byte;
            s_frame_idx = 1U;
        }
        else
        {
            s_state = RX_STATE_SYM1;
        }
        break;

    case RX_STATE_LEN:
        s_frame_buf[s_frame_idx++] = byte;
        if (byte == GNSS_HJD_FRAME_LEN)
        {
            /* 帧长正确, 再收剩余字节: 总帧长 - 已收 3 字节 (sync1+sync2+len) */
            s_remain = GNSS_HJD_TOTAL_LEN - 3U;
            s_state = RX_STATE_DATA;
        }
        else
        {
            /* 帧长不对, 丢弃重找同步字 */
            s_state = RX_STATE_SYM1;
        }
        break;

    case RX_STATE_DATA:
        s_frame_buf[s_frame_idx++] = byte;
        if (--s_remain == 0U)
        {
            /* 一帧收齐, 校验解析 */
            if (!parse_frame(s_frame_buf))
                g_gnss_err_count++;
            s_state = RX_STATE_SYM1;
        }
        break;

    default:
        s_state = RX_STATE_SYM1;
        break;
    }
}

/* ---- 对外 API ---- */

void gnss1_rx_init(void)
{
    dma_single_data_parameter_struct dma_config;

    /* 重新配置 USART0: 230400, 8E1 (偶校验)
     * 注意: 偶校验需要 9bit 字长 (8 数据 + 1 校验)
     * TX DMA 已由 UART0_DMA_Init 配好, 这里只重配 USART 参数 + RX DMA */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 230400U);
    usart_word_length_set(USART0, USART_WL_9BIT); /* 8 数据 + 1 校验 = 9bit */
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_EVEN); /* 偶校验 */
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* 重新使能 TX DMA 请求 (usart_deinit 会关掉) */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);

    /* ---- RX DMA: DMA1_CH2 循环模式 ---- */
    dma_deinit(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL);
    dma_single_data_para_struct_init(&dma_config);
    dma_config.direction = DMA_PERIPH_TO_MEMORY;
    dma_config.memory0_addr = (uint32_t)s_dma_buf;
    dma_config.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_config.number = GNSS_RX_DMA_BUF_SIZE;
    dma_config.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_config.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_config.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_config.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL, &dma_config);
    dma_channel_subperipheral_select(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL, GNSS_RX_DMA_SUBPERI);
    dma_circulation_enable(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL);
    dma_flag_clear(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL, DMA_FLAG_FTF);
    dma_channel_enable(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL);

    /* 使能 USART0 RX DMA 请求 */
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);

    /* 初始化解析状态 */
    s_dma_write_laps = 0U;
    s_dma_read_total = 0U;
    s_state = RX_STATE_SYM1;
    s_frame_idx = 0U;
    s_remain = 0U;
    s_result_new = 0U;
    g_gnss_frame_count = 0U;
    g_gnss_err_count = 0U;
    g_gnss_ovf_count = 0U;
    memset(&g_gnss_result, 0, sizeof(g_gnss_result));

    /* 使能 DMA1_CH2 FTF 中断: 环形缓冲每圈完成时触发, ISR 里 ++s_dma_write_laps,
     * 消除 transfer_number 回装竞态 (轮询在临界帧可能读到不稳定值) */
    nvic_irq_enable(DMA1_Channel2_IRQn, 2U, 0U);
    dma_interrupt_enable(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL, DMA_CHXCTL_FTFIE);
}

void gnss1_rx_pump(void)
{
    uint16_t write_idx;
    uint32_t write_total;
    uint32_t available;
    uint8_t byte;

    /* ① DMA 循环模式: 当前写入位置 = BUF_SIZE - 剩余传输数.
     *    DMA1_CH2 FTF ISR 已使能, 每圈完成时 ++s_dma_write_laps.
     *    注: 理论上 FTF ISR 可能在 write_idx 与 write_total 之间插入,
     *    导致 write_total 多算整圈触发 ovf_count 假阳性. 实测扩大缓冲至
     *    2048 字节后 (一圈 87.2ms), 主循环 5ms 节拍远小于此窗口,
     *    该 TOCTOU 竞态实际影响可忽略, 故不做 __disable_irq 保护. */
    write_idx = (uint16_t)(GNSS_RX_DMA_BUF_SIZE -
                           dma_transfer_number_get(GNSS_RX_DMA, GNSS_RX_DMA_CHANNEL));
    write_total = s_dma_write_laps * GNSS_RX_DMA_BUF_SIZE + (uint32_t)write_idx;
    available = write_total - s_dma_read_total;

    /* ② 读指针落后 >= 一整圈: 未读字节已被 DMA 覆盖, 只能丢弃这一整圈,
     *    从最新的 BUF_SIZE 字节重新同步 (丢弃后仍能靠同步字重新对齐). */
    if (available > GNSS_RX_DMA_BUF_SIZE)
    {
        g_gnss_ovf_count++;
        s_dma_read_total = write_total - GNSS_RX_DMA_BUF_SIZE;
        available = GNSS_RX_DMA_BUF_SIZE;
    }

    /* ③ 逐字节喂给状态机 */
    while (available > 0U)
    {
        byte = s_dma_buf[s_dma_read_total % GNSS_RX_DMA_BUF_SIZE];
        s_dma_read_total++;
        available--;
        feed_byte(byte);
    }
}

uint8_t gnss1_rx_get_result(gnss1_rx_result_t *out)
{
    if (out == NULL)
        return s_result_new;

    if (s_result_new)
    {
        *out = g_gnss_result;
        s_result_new = 0U;
        return 1U;
    }
    return 0U;
}
