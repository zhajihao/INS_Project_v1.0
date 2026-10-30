/**
 * @file gnss1_rx.h
 * @brief GNSS1 通道接收驱动与协议解析 (USART0 DMA RX)
 *
 * 职责:
 *   - USART0 230400 8E1 + DMA1_CH2 循环模式接收
 *   - T_HJD_POSDATA 帧解析 (同步字 0x55 0xBB + 51B + XOR 校验)
 *   - 环形缓冲 + 状态机逐字节对齐
 *
 * 协议细节: 同步字/帧长/校验见下方 GNSS_HJD_* 常量, 字段偏移见 gnss1_rx.c 的 parse_frame.
 * 物理: USART0 (PA9 TX 导航结果 / PA10 RX GNSS1), 230400, 8E1
 *   TX: DMA1_CH7 (保留发导航结果帧)
 *   RX: DMA1_CH2 循环模式 + 应用层环形缓冲 (DMA 搬运, 主循环解析)
 */
#ifndef GNSS1_RX_H
#define GNSS1_RX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- 协议常量 (HJD 协议细节, 不随模块名变) ---- */

/** 同步字第 1 字节 (线上先收到) */
#define GNSS_HJD_SYM_BYTE1 0x55U
/** 同步字第 2 字节 */
#define GNSS_HJD_SYM_BYTE2 0xBBU

/** 帧长字段固定值 (= 整帧总字节数) */
#define GNSS_HJD_FRAME_LEN 51U

/** 整帧总长 (含同步字) */
#define GNSS_HJD_TOTAL_LEN 51U

/** 校验和累加范围字节数 (索引 0 起至校验字节前, 不含 u8Xor) */
#define GNSS_HJD_CHECKSUM_LEN (GNSS_HJD_TOTAL_LEN - 1U)

/** 环缓冲溢出时刻快照条数 (环形记录, 最新覆盖最旧) */
#define GNSS_RX_OVF_STAMP_NUM 8U

    /** @brief 溢出时刻快照 (单位 ms, 由 g_sys_tick_ms 提供), 环形写入 */
    extern volatile uint32_t g_gnss_ovf_stamp_ms[GNSS_RX_OVF_STAMP_NUM];
    /** @brief 下一次溢出时刻写入的下标 (0..GNSS_RX_OVF_STAMP_NUM-1) */
    extern volatile uint8_t g_gnss_ovf_stamp_idx;

    /* ---- 解析结果 (GNSS1 通道输出, 供 navi_pipeline 使用) ---- */

    /**
     * @brief 解析后的 GNSS1 导航数据 (导航系物理量)
     */
    typedef struct
    {
        double lat_rad;       /* 纬度 (rad) */
        double lon_rad;       /* 经度 (rad) */
        double height_m;      /* 高度 (m) */
        double vn_e;          /* 东向速度 (m/s) */
        double vn_n;          /* 北向速度 (m/s) */
        double vn_u;          /* 天向速度 (m/s) */
        float pdop;           /* PDOP */
        uint8_t sat_num;      /* 跟踪星数 */
        uint8_t pos_valid;    /* u8PosVld == 2 时为 1 */
        uint32_t frame_count; /* 累计有效帧计数 */
    } gnss1_rx_result_t;

    /* ---- 全局状态变量 (业务可观测性) ---- */

    /** 最新一帧解析结果 (只由 parse_frame 写; 消费者请走 gnss1_rx_get_result) */
    extern gnss1_rx_result_t g_gnss_result;
    /** 累计校验通过的帧数 (持续递增 = 接收正常) */
    extern volatile uint32_t g_gnss_frame_count;
    /** 累计校验失败帧数 (应保持 0) */
    extern volatile uint32_t g_gnss_err_count;
    /** DMA 环缓冲整圈覆盖次数 (非 0 = 数据已丢失, 应保持 0) */
    extern volatile uint32_t g_gnss_ovf_count;

    /* ---- API ---- */

    void gnss1_rx_init(void);

    void gnss1_rx_pump(void);

    uint8_t gnss1_rx_get_result(gnss1_rx_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GNSS1_RX_H */
