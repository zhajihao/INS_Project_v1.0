#include "XV7021.h"
#include "systick.h"
#include "spi.h"
/*
 * GD32F405 + 3 x Epson XV7021BB
 * SPI0:
 *   PA5 = SCK  (AF5)
 *   PA6 = MISO (AF5)
 *   PA7 = MOSI (AF5)
 * CS:
 *   gyro1 = PA4
 *   gyro2 = PA15
 *   gyro3 = PB9
 *
 * The three sensors are operated as independent 4-wire SPI slaves.
 * Epson's factory Multi-Slave Function is NOT used.
 */

#define XV_CS1_PORT GPIOA
#define XV_CS1_PIN GPIO_PIN_4
#define XV_CS2_PORT GPIOA
#define XV_CS2_PIN GPIO_PIN_15
#define XV_CS3_PORT GPIOB
#define XV_CS3_PIN GPIO_PIN_9

#define XV_SPI_TIMEOUT 1000000UL

xv7021bb_all_data_t xv7021_data;

/* ---------- SPI 辅助函数 ---------- */
static xv7021bb_status_t command_no_data(xv7021bb_id_t id, uint8_t cmd);

/**
 * @brief  将指定 XV7021BB 的 CS 引脚拉高（释放片选）
 * @param  id  陀螺仪编号 (XV7021BB_1 / _2 / _3)
 */
static void cs_high(xv7021bb_id_t id)
{
    if (id == XV7021BB_1)
        gpio_bit_set(XV_CS1_PORT, XV_CS1_PIN);
    else if (id == XV7021BB_2)
        gpio_bit_set(XV_CS2_PORT, XV_CS2_PIN);
    else
        gpio_bit_set(XV_CS3_PORT, XV_CS3_PIN);
}

/**
 * @brief  将指定 XV7021BB 的 CS 引脚拉低（选中片选）
 * @param  id  陀螺仪编号 (XV7021BB_1 / _2 / _3)
 */
static void cs_low(xv7021bb_id_t id)
{
    if (id == XV7021BB_1)
        gpio_bit_reset(XV_CS1_PORT, XV_CS1_PIN);
    else if (id == XV7021BB_2)
        gpio_bit_reset(XV_CS2_PORT, XV_CS2_PIN);
    else
        gpio_bit_reset(XV_CS3_PORT, XV_CS3_PIN);
}

/**
 * @brief  校验陀螺仪编号是否有效 (0~2)
 * @param  id  待校验的陀螺仪编号
 * @return 1 有效；0 无效
 */
static int id_valid(xv7021bb_id_t id)
{
    return (id == XV7021BB_1 || id == XV7021BB_2 || id == XV7021BB_3);
}

/**
 * @brief  SPI0 DMA 单字节收发
 *
 * 发送 1 字节 tx，同时接收 1 字节返回值，用于所有寄存器级 SPI 通信。
 *
 * @param  tx   待发送的字节
 * @param  ret  状态码输出指针 (XV7021BB_OK / XV7021BB_ERR_TIMEOUT)
 * @return 接收到的字节
 */
static uint8_t spi0_rw(uint8_t tx, xv7021bb_status_t *ret)
{
    uint8_t rx = 0U;
    *ret = (spi0_dma_transfer(&tx, &rx, 1U) == 0) ? XV7021BB_OK : XV7021BB_ERR_TIMEOUT;
    return rx;
}

/**
 * @brief  初始化 Cortex-M4 DWT 周期计数器
 *
 * 启用 CoreDebug 和 DWT CYCCNT，为 short_delay_us() 提供高精度时基。
 */
static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  DWT 实现的微秒级忙等延时
 * @param  us  延时的微秒数（约 1~1000 us 范围有效）
 */
static void short_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}

/**
 * @brief  XV7021BB 板级初始化
 *
 * 完成 GPIO/SPI 时钟使能、SPI0 引脚 AF 配置、三路 CS 输出初始化、
 * SPI0 DMA 初始化，并对 XV7021BB_1 执行 4 线模式配置 + 退出休眠 + 软复位，
 * 为后续单芯片初始化做准备。
 */
void XV7021BB_BoardInit(void)
{
    /* Clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI0);

    /* SPI0 pins: PA5/PA6/PA7 AF5 */
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* Software chip-selects */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, XV_CS1_PIN | XV_CS2_PIN);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, XV_CS1_PIN | XV_CS2_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, XV_CS3_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, XV_CS3_PIN);

    cs_high(XV7021BB_1);
    cs_high(XV7021BB_2);
    cs_high(XV7021BB_3);

    XV7021BB_SPI0_Init();
    delay_ms(50);
    /* 配置SPI 4线模式*/
    XV7021BB_RegWrite(XV7021BB_1, 0x1F, 0x00);
    delay_ms(50);
    /* 退出休眠 */
    command_no_data(XV7021BB_1, 0x06);
    delay_ms(50);
    /* 芯片软复位 */
    command_no_data(XV7021BB_1, 0x09);
}

/**
 * @brief  初始化 SPI0 外设
 *
 * 配置为 8bit MSB 主模式、Mode 0 (CPOL=Low, CPHA=1Edge)、
 * 预分频 /16 (APB2@84MHz → SPI 5.25MHz，低于 XV7021BB 的 10MHz 上限)。
 * 同时启用 SPI0 DMA 和 Cortex-M4 DWT 延时计数器。
 */
void XV7021BB_SPI0_Init(void)
{
    spi_parameter_struct spi_init_struct;

    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;

    /* XV7021BB 4-wire SPI timing: Mode 0 is used here. */
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;

    /* Three independent GPIO CS lines => software NSS. */
    spi_init_struct.nss = SPI_NSS_SOFT;

    /*
     * GD32F405 @ 168 MHz:
     *   APB2 = 84 MHz
     *   SPI0 = APB2
     *   /16 = 5.25 MHz
     *
     * This is below XV7021BB's 10 MHz maximum when VDDI > 2.4 V.
     */
    spi_init_struct.prescale = SPI_PSC_16;
    spi_init_struct.endian = SPI_ENDIAN_MSB;

    spi_init(SPI0, &spi_init_struct);
    spi_enable(SPI0);
    spi0_dma_init();
    dwt_delay_init();
}

/**
 * @brief  向指定 XV7021BB 的寄存器写入 1 字节
 * @param  id   陀螺仪编号
 * @param  reg  寄存器地址 (0x00 ~ 0x1F)
 * @param  data 待写入的 8 位数据
 * @return XV7021BB_OK 成功；XV7021BB_ERR_PARAM 参数错误；XV7021BB_ERR_TIMEOUT SPI 超时
 */
xv7021bb_status_t XV7021BB_RegWrite(xv7021bb_id_t id, uint8_t reg, uint8_t data)
{
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || (reg > 0x1FU))
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw((uint8_t)(reg & 0x1FU | (0x01 << 5)), &ret); /* R/W=0 */
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

    (void)spi0_rw(data, &ret);
    cs_high(id);

    return ret;
}

/**
 * @brief  从指定 XV7021BB 的寄存器读取 1 字节
 * @param  id   陀螺仪编号
 * @param  reg  寄存器地址 (0x00 ~ 0x1F)
 * @param  data 读取数据输出指针
 * @return XV7021BB_OK 成功；XV7021BB_ERR_PARAM 参数错误；XV7021BB_ERR_TIMEOUT SPI 超时
 */
xv7021bb_status_t XV7021BB_RegRead(xv7021bb_id_t id, uint8_t reg, uint8_t *data)
{
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || data == 0 || reg > 0x1FU)
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw((uint8_t)(0x80U | (reg & 0x1FU) | (0x01 << 5)), &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

    *data = spi0_rw(0xFFU, &ret);
    cs_high(id);

    return ret;
}

/**
 * @brief  向 XV7021BB 发送一个无数据的命令（如软复位、退出休眠等）
 *
 * 发送完成后额外延时 20us，确保 XV7021BB 内部状态机完成命令处理。
 *
 * @param  id   陀螺仪编号
 * @param  cmd  命令字节 (0x00 ~ 0x1F)
 * @return XV7021BB_OK 成功；XV7021BB_ERR_PARAM 参数错误；XV7021BB_ERR_TIMEOUT SPI 超时
 */
static xv7021bb_status_t command_no_data(xv7021bb_id_t id, uint8_t cmd)
{
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || cmd > 0x1FU)
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw((uint8_t)(cmd & 0x1FU | (0x01 << 5)), &ret);
    cs_high(id);

    if (ret == XV7021BB_OK)
        short_delay_us(20U);
    return ret;
}

/**
 * @brief  从 XV7021BB 读取 16 位原始角速度值
 * @param  id   陀螺仪编号
 * @param  raw  16 位有符号原始值输出指针
 * @return XV7021BB_OK 成功；XV7021BB_ERR_PARAM 参数错误；XV7021BB_ERR_TIMEOUT SPI 超时
 */
xv7021bb_status_t XV7021BB_ReadRate16(xv7021bb_id_t id, int16_t *raw)
{
    uint8_t h, l;
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || raw == 0)
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw(0x80U | XV7021BB_REG_RATE | (0x01 << 5), &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

    h = spi0_rw(0xFFU, &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }
    l = spi0_rw(0xFFU, &ret);
    cs_high(id);

    if (ret != XV7021BB_OK)
        return ret;

    *raw = (int16_t)(((uint16_t)h << 8) | l);
    return XV7021BB_OK;
}

/**
 * @brief  从 XV7021BB 读取 24 位原始角速度值
 *
 * 读取 3 字节数据并组合为 24 位有符号值，做符号扩展后存入 int32_t。
 *
 * @param  id   陀螺仪编号
 * @param  raw  24 位有符号原始值输出指针
 * @return XV7021BB_OK 成功；XV7021BB_ERR_PARAM 参数错误；XV7021BB_ERR_TIMEOUT SPI 超时
 */
xv7021bb_status_t XV7021BB_ReadRate24(xv7021bb_id_t id, int32_t *raw)
{
    uint8_t b0, b1, b2;
    int32_t v;
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || raw == 0)
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw(0x80U | XV7021BB_REG_RATE | (0x01 << 5), &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

    b0 = spi0_rw(0xFFU, &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }
    b1 = spi0_rw(0xFFU, &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }
    b2 = spi0_rw(0xFFU, &ret);
    cs_high(id);

    if (ret != XV7021BB_OK)
        return ret;

    v = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (v & 0x00800000L)
        v |= (int32_t)0xFF000000L;

    *raw = v;
    return XV7021BB_OK;
}

/**
 * @brief  读取角速度并换算为 °/s
 *
 * 根据 XV7021BB_RATE_24BIT 宏选择 24 位或 16 位读数路径：
 *   - 24 位模式: dps = raw / 17920.0
 *   - 16 位模式: dps = raw / 70.0
 *
 * @param  id   陀螺仪编号
 * @param  rate 角速度数据输出指针 (含 raw 和 dps 字段)
 * @return XV7021BB_OK 成功；其他值表示失败
 */
xv7021bb_status_t XV7021BB_ReadRate(xv7021bb_id_t id, xv7021bb_rate_t *rate)
{
    xv7021bb_status_t ret;
    if (!id_valid(id) || rate == 0)
        return XV7021BB_ERR_PARAM;

#if XV7021BB_RATE_24BIT
    ret = XV7021BB_ReadRate24(id, &rate->raw);
    rate->dps = (float)rate->raw / 17920.0f;
#else
    {
        int16_t r16;
        ret = XV7021BB_ReadRate16(id, &r16);
        rate->raw = r16;
        rate->dps = (float)r16 / 70.0f;
    }
#endif
    return ret;
}

/**
 * @brief  读取 XV7021BB 芯片内部温度
 *
 * 根据 XV7021BB_TEMP_FORMAT_CODE 选择 12 位或 8/10 位格式：
 *   - 12 位: raw = 12 位有符号, degC = 25 + (raw - 400) / 16.0
 *   - 8/10 位: raw = 8 位有符号, degC 暂不换算
 *
 * @param  id    陀螺仪编号
 * @param  temp  温度数据输出指针
 * @return XV7021BB_OK 成功；其他值表示失败
 */
xv7021bb_status_t XV7021BB_ReadTemp(xv7021bb_id_t id, xv7021bb_temp_t *temp)
{
    uint8_t b0, b1 = 0U;
    int16_t raw;
    xv7021bb_status_t ret = XV7021BB_OK;

    if (!id_valid(id) || temp == 0)
        return XV7021BB_ERR_PARAM;

    cs_low(id);
    (void)spi0_rw(0x80U | XV7021BB_REG_TEMP | (0x01 << 5), &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

    b0 = spi0_rw(0xFFU, &ret);
    if (ret != XV7021BB_OK)
    {
        cs_high(id);
        return ret;
    }

#if XV7021BB_TEMP_FORMAT_CODE == 2U
    b1 = spi0_rw(0xFFU, &ret);
    cs_high(id);
    if (ret != XV7021BB_OK)
        return ret;
    raw = (int16_t)(((uint16_t)b0 << 4) | (b1 >> 4));
    if (raw & 0x0800)
        raw |= (int16_t)0xF000;
    temp->raw = raw;
    /* 12-bit temperature output: typical conversion used by Epson's reference implementation. */
    //    temp->degC = 25.0f + ((float)raw * 0.0625f);
    temp->degC = 25.0f + (raw - 400) / 16.0f;
#else
    cs_high(id);
    if (ret != XV7021BB_OK)
        return ret;
    raw = (int16_t)b0;
    temp->raw = raw;
    temp->degC = (float)raw; /* Use 8/10-bit conversion only after selecting that format. */
#endif

    temp->valid = 1U;
    return XV7021BB_OK;
}

/**
 * @brief  读取 XV7021BB 状态寄存器并解析电源状态 / 处理 OK 标志
 * @param  id  陀螺仪编号
 * @param  st  状态信息输出指针 (status, proc_ok, power_state)
 * @return XV7021BB_OK 成功；其他值表示失败
 */
xv7021bb_status_t XV7021BB_ReadStatus(xv7021bb_id_t id, xv7021bb_status_info_t *st)
{
    uint8_t v;
    xv7021bb_status_t ret;

    if (!id_valid(id) || st == 0)
        return XV7021BB_ERR_PARAM;

    ret = XV7021BB_RegRead(id, XV7021BB_REG_STATUS, &v);
    if (ret != XV7021BB_OK)
        return ret;

    st->status = v;
    st->proc_ok = (uint8_t)((v >> 3) & 0x01U);
    st->power_state = (uint8_t)(v & 0x07U);
    return XV7021BB_OK;
}

/**
 * @brief  初始化单个 XV7021BB 陀螺仪
 *
 * 完整流程:
 *   1. 延时 2ms 等待上电稳定
 *   2. 软复位 + 等待芯片就绪 (最多 200ms 轮询电源状态)
 *   3. 配置 DSPCTL2: notch 关闭, 2 阶 LPF, 截止频率 10Hz
 *   4. 配置 DSPCTL3: 采样分频 (fs_div)
 *   5. 配置 OUTCTL1: 选择 24/16 位角速度输出
 *   6. 配置 TSFORMAT: 12 位温度格式
 *   7. 回读关键寄存器做校验
 *
 * @param  id  陀螺仪编号
 * @return XV7021BB_OK 成功；其他值表示失败 (超时 / 写入 / 校验)
 */
xv7021bb_status_t XV7021BB_Init(xv7021bb_id_t id)
{
    xv7021bb_status_t ret;
    uint8_t v;

    if (!id_valid(id))
        return XV7021BB_ERR_PARAM;

    /* Datasheet: serial communication may start after tIF >= 1 ms. */
    short_delay_us(2000U);

    /* Software reset; user-command registers return to defaults. */
    ret = command_no_data(id, XV7021BB_REG_SWRESET);
    if (ret != XV7021BB_OK)
        return ret;

    short_delay_us(20U);

    /* Wait for sensor to leave POR/startup. Datasheet specifies up to 200 ms. */
    {
        uint32_t i;
        xv7021bb_status_info_t st;
        uint8_t ready = 0U;

        for (i = 0U; i < 200U; ++i)
        {
            if (XV7021BB_ReadStatus(id, &st) == XV7021BB_OK)
            {
                if ((st.power_state == 1U) || (st.power_state == 4U))
                {
                    ready = 1U;
                    break;
                }
            }
            short_delay_us(1000U);
        }
        if (!ready)
            return XV7021BB_ERR_STATUS;
    }

    /*
     * Configure DSP2:
     *   notch disabled
     *   LPF order = 2nd
     *   LPF Fc = 10 Hz
     * All reserved bits are kept at their documented default values.
     */
    v = (uint8_t)(((XV7021BB_NOTCH_ENABLE & 1U) << 6) |
                  ((XV7021BB_LPF_ORDER_CODE & 3U) << 4) |
                  (XV7021BB_LPF_FC_CODE & 0x0FU));
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_DSPCTL2, v);
    if (ret != XV7021BB_OK)
        return ret;

    /* DSP3: EnCalibCmd=0, fs divider selectable. */
    v = (uint8_t)(XV7021BB_FS_DIV_CODE & 0x07U);
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_DSPCTL3, v);
    if (ret != XV7021BB_OK)
        return ret;

    /* 24-bit angular-rate data; reserved bit0 remains at default 1. */
#if XV7021BB_RATE_24BIT
    v = 0x05U;
#else
    v = 0x01U;
#endif
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_OUTCTL1, v);
    if (ret != XV7021BB_OK)
        return ret;

    /* Keep default 12-bit temperature format and documented MISO settings. */
    v = 0x4BU; /* 12-bit + default SelMISO bits + reserved defaults */
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_TSFORMAT, v);
    if (ret != XV7021BB_OK)
        return ret;

    /* Verify key settings. */
    ret = XV7021BB_RegRead(id, XV7021BB_REG_DSPCTL2, &v);
    if (ret != XV7021BB_OK || v != 0x00U)
        return XV7021BB_ERR_VERIFY;

    ret = XV7021BB_RegRead(id, XV7021BB_REG_OUTCTL1, &v);
#if XV7021BB_RATE_24BIT
    if (ret != XV7021BB_OK || v != 0x05U)
        return XV7021BB_ERR_VERIFY;
#else
    if (ret != XV7021BB_OK || v != 0x01U)
        return XV7021BB_ERR_VERIFY;
#endif

    return XV7021BB_OK;
}

/**
 * @brief  依次初始化全部三路 XV7021BB 陀螺仪
 *
 * 逐个调用 XV7021BB_Init()，任一失败立即返回错误码。
 *
 * @return XV7021BB_OK 全部成功；其他值表示第一个失败的陀螺仪错误码
 */
xv7021bb_status_t XV7021BB_InitAll(void)
{
    xv7021bb_status_t ret;
    ret = XV7021BB_Init(XV7021BB_1);
    if (ret != XV7021BB_OK)
        return ret;
    ret = XV7021BB_Init(XV7021BB_2);
    if (ret != XV7021BB_OK)
        return ret;
    ret = XV7021BB_Init(XV7021BB_3);
    if (ret != XV7021BB_OK)
        return ret;
    return XV7021BB_OK;
}

/**
 * @brief  对单个 XV7021BB 执行零偏自动校准 (AutoCalib)
 *
 * 前提: 陀螺仪必须完全静止。
 * 流程: 使能 EnCalibCmd → 发送 AUTOC 命令 → 关闭 EnCalibCmd
 *       校准值写入芯片内部寄存器，掉电不丢失。
 *
 * @param  id  陀螺仪编号
 * @return XV7021BB_OK 成功；其他值表示失败
 */
xv7021bb_status_t XV7021BB_ZeroRateCalibrate(xv7021bb_id_t id)
{
    xv7021bb_status_t ret;

    if (!id_valid(id))
        return XV7021BB_ERR_PARAM;

    /* Enable calibration command in DSPCTL3, preserving sampling divider. */
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_DSPCTL3,
                            (uint8_t)(0x40U | (XV7021BB_FS_DIV_CODE & 0x07U)));
    if (ret != XV7021BB_OK)
        return ret;

    /* Sensor must be completely stationary during this command. */
    ret = command_no_data(id, XV7021BB_REG_AUTOC);
    if (ret != XV7021BB_OK)
        return ret;

    /* Disable calibration mode; calibration value remains active until disabled. */
    ret = XV7021BB_RegWrite(id, XV7021BB_REG_DSPCTL3,
                            (uint8_t)(XV7021BB_FS_DIV_CODE & 0x07U));
    return ret;
}

/**
 * @brief  依次对全部三路 XV7021BB 执行零偏自动校准
 * @return XV7021BB_OK 全部成功；其他值表示第一个失败的陀螺仪错误码
 */
xv7021bb_status_t XV7021BB_ZeroRateCalibrateAll(void)
{
    xv7021bb_status_t ret;
    ret = XV7021BB_ZeroRateCalibrate(XV7021BB_1);
    if (ret != XV7021BB_OK)
        return ret;
    ret = XV7021BB_ZeroRateCalibrate(XV7021BB_2);
    if (ret != XV7021BB_OK)
        return ret;
    ret = XV7021BB_ZeroRateCalibrate(XV7021BB_3);
    if (ret != XV7021BB_OK)
        return ret;
    return XV7021BB_OK;
}

/**
 * @brief  采样任务：读取三路陀螺仪角速度数据
 *
 * 遍历 XV7021BB_1 / _2 / _3，分别读取角速度并存入全局 xv7021_data，
 * 构建 valid_mask 标记哪些通道本次成功读取（bit0=gyro1, bit1=gyro2, bit2=gyro3）。
 * 每次调用递增 sample_counter。
 *
 * @return 有效通道掩码 (0x07 表示三路全部成功)
 */
uint8_t XV7021BB_SampleTask(void)
{
    uint8_t i;
    uint8_t valid_mask = 0U;
    xv7021bb_rate_t rate;

    for (i = 0U; i < 3U; ++i)
    {
        if (XV7021BB_ReadRate((xv7021bb_id_t)i, &rate) == XV7021BB_OK)
        {
            xv7021_data.gyro[i] = rate;
            valid_mask |= (uint8_t)(1U << i);
        }
    }
    ++xv7021_data.sample_counter;
    return valid_mask;
}

/**
 * @brief  获取全局 XV7021BB 采样数据只读指针
 * @return xv7021_data 的 const 指针（包含三路角速度 / 温度 / 采样计数）
 */
const xv7021bb_all_data_t *XV7021BB_GetData(void)
{
    return (const xv7021bb_all_data_t *)&xv7021_data;
}
