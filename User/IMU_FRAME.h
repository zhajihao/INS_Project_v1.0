#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <stdint.h>

/**
 * @brief  UART0 输出的 IMU 数据包结构（56 字节，1 字节对齐）
 *
 * 帧格式:
 *   header1/header2 : 固定 0x55 0xAA
 *   data_length     : 自本字节起至帧尾 end2 的字节数, 代码取 sizeof(帧)-2 = 54
 *                     (其中 cnt 起到 accel_temp_z 为 50 字节)
 *   cnt             : 递增序号
 *   gyro_x/y/z      : 三轴角速度 (°/s)
 *   accel_x/y/z     : 三轴加速度 (m/s²)
 *   gyro_temp_x/y/z : 三颗陀螺仪各自的芯片温度 (℃)
 *   accel_temp_x/y/z: 加速度计的温度（三轴相同，来自同一芯片）(℃)
 *   checksum        : 从 header1 到 accel_temp_z 的累加和低 8 位 (共 sizeof(帧)-3 = 53 字节)
 *   end1/end2       : 固定 0x0D 0x0A
 */
#pragma pack(push, 1)
typedef struct
{
    uint8_t header1;     /**< 帧头第 1 字节，固定 0x55 */
    uint8_t header2;     /**< 帧头第 2 字节，固定 0xAA */
    uint8_t data_length; /**< 数据长度 (自本字节至 end2, 固定 54) */
    uint16_t cnt;        /**< 帧序号（递增） */
    float gyro_x;        /**< X 轴角速度 (°/s) */
    float gyro_y;        /**< Y 轴角速度 (°/s) */
    float gyro_z;        /**< Z 轴角速度 (°/s) */
    float accel_x;       /**< X 轴加速度 (m/s²) */
    float accel_y;       /**< Y 轴加速度 (m/s²) */
    float accel_z;       /**< Z 轴加速度 (m/s²) */
    float gyro_temp_x;   /**< X 轴陀螺仪温度 (℃) */
    float gyro_temp_y;   /**< Y 轴陀螺仪温度 (℃) */
    float gyro_temp_z;   /**< Z 轴陀螺仪温度 (℃) */
    float accel_temp_x;  /**< 加速度计温度 X (℃) */
    float accel_temp_y;  /**< 加速度计温度 Y (℃) */
    float accel_temp_z;  /**< 加速度计温度 Z (℃) */
    uint8_t checksum;    /**< 校验和（累加和低 8 位） */
    uint8_t end1;        /**< 帧尾第 1 字节，固定 0x0D */
    uint8_t end2;        /**< 帧尾第 2 字节，固定 0x0A */
} IMU_Frame_t;
#pragma pack(pop)

/** @brief  编译期断言 IMU_Frame_t 严格 56 字节 */
typedef char IMU_Frame_size_must_be_56[(sizeof(IMU_Frame_t) == 56U) ? 1 : -1];

void imu_send_frame(void);

/* ============================================================
 * 组合导航输出帧 (navi_send_frame)
 * ============================================================ */

#define NAV_FRAME_TYPE 0x81U /**< 导航帧类型标识 (与 IMU 帧区分) */

/**
 * @brief  UART0 输出的组合导航结果数据包 (44 字节, 1 字节对齐)
 *
 * 帧格式:
 *   header1/header2 : 固定 0x55 0xAA
 *   frame_type      : 固定 0x81 (与 IMU 原始帧 0x55 0xAA 的帧头区分)
 *   cnt              : 导航帧递增序号
 *   att[3]           : 姿态角 [pitch, roll, yaw] (rad, float)
 *   vn[3]            : 速度 [E, N, U] (m/s, float)
 *   pos[3]           : 位置 [lat(rad), lon(rad), h(m)] (float)
 *   checksum         : 从 frame_type 到 pos[2] 的累加和低 8 位
 *                      (共 offsetof(checksum)-offsetof(frame_type) = 39 字节, 不含帧头)
 *   end1/end2        : 固定 0x0D 0x0A
 *
 * @note   与 IMU 帧不同: 本帧无 data_length 字段, 且校验范围不含帧头。
 *         末尾另有 3 字节 (checksum/end1/end2) 不参与校验。
 */
#pragma pack(push, 1)
typedef struct
{
    uint8_t header1;    /**< 固定 0x55 */
    uint8_t header2;    /**< 固定 0xAA */
    uint8_t frame_type; /**< 固定 0x81 */
    uint16_t cnt;       /**< 导航帧序号 (递增) */
    float att[3];       /**< [pitch, roll, yaw] (rad) */
    float vn[3];        /**< [E, N, U] (m/s) */
    float pos[3];       /**< [lat(rad), lon(rad), h(m)] */
    uint8_t checksum;   /**< 累加和低 8 位 */
    uint8_t end1;       /**< 固定 0x0D */
    uint8_t end2;       /**< 固定 0x0A */
} Nav_Frame_t;
#pragma pack(pop)

void nav_send_frame(const double att_rad[3], const double vn_mps[3], const double pos[3]);

#endif
