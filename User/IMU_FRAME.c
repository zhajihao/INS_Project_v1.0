#include "IMU_FRAME.h"
#include <stddef.h>
#include "uart.h"
#include "XV7021.h"
#include "ADXL357.h"

#define STANDARD_GRAVITY_M_S2 9.80665f

static IMU_Frame_t imu_frame;
static uint16_t imu_frame_counter;

/**
 * @brief  计算 IMU 帧的 8 位累加和校验
 *
 * 将所有字节累加后取低 8 位，用作通信帧校验字段。
 *
 * @param  bytes   数据缓冲区指针
 * @param  length  参与校验的字节数
 * @return 累加和的低 8 位
 */
uint8_t imu_checksum(uint8_t *bytes, size_t length)
{
  uint16_t sum = 0U;
  size_t index;

  for (index = 0; index < length; ++index)
  {
    sum += bytes[index];
  }
  return (uint8_t)sum;
}

/**
 * @brief  打包并发送一帧 IMU 数据
 *
 * 帧格式: 0x55 0xAA | data_length | cnt | gyro_x dps | gyro_y dps | gyro_z dps
 *        | accel_x m/s² | accel_y m/s² | accel_z m/s²
 *        | gyro_temp_x | gyro_temp_y | gyro_temp_z
 *        | accel_temp_x | accel_temp_y | accel_temp_z | checksum | 0x0D 0x0A
 *
 * 坐标轴映射说明:
 *   - 陀螺仪: xv7021[1]→gyro_x, -xv7021[2]→gyro_y, -xv7021[0]→gyro_z
 *   - 加速度: -adxl357_y→accel_x, -adxl357_x→accel_y, -adxl357_z→accel_z
 *   - 温度与陀螺仪索引顺序保持一致
 *   - 加速度 g 值乘以 9.80665 换算为 m/s²
 *
 * 最后 3 字节 (checksum + 结束符) 不参与 checksum 计算。
 */
void imu_send_frame(void)
{
  imu_frame.header1 = 0x55U;
  imu_frame.header2 = 0xAAU;
  imu_frame.data_length = (uint8_t)(sizeof(imu_frame) - 2U);
  imu_frame.cnt = imu_frame_counter++;

  /* Sensor-to-navigation frame axes follow the existing board mapping. */
  imu_frame.gyro_x = xv7021_data.gyro[1].dps;
  imu_frame.gyro_y = -xv7021_data.gyro[2].dps;
  imu_frame.gyro_z = -xv7021_data.gyro[0].dps;

  imu_frame.accel_x = -adxl357_data.y_g * STANDARD_GRAVITY_M_S2;
  imu_frame.accel_y = -adxl357_data.x_g * STANDARD_GRAVITY_M_S2;
  imu_frame.accel_z = -adxl357_data.z_g * STANDARD_GRAVITY_M_S2;

  imu_frame.gyro_temp_x = xv7021_data.temp[1].degC;
  imu_frame.gyro_temp_y = xv7021_data.temp[2].degC;
  imu_frame.gyro_temp_z = xv7021_data.temp[0].degC;
  imu_frame.accel_temp_x = adxl357_data.temperature_c;
  imu_frame.accel_temp_y = adxl357_data.temperature_c;
  imu_frame.accel_temp_z = adxl357_data.temperature_c;

  imu_frame.checksum = imu_checksum((uint8_t *)&imu_frame, sizeof(imu_frame) - 3);
  imu_frame.end1 = 0x0DU;
  imu_frame.end2 = 0x0AU;

  (void)UART0_DMA_Send((const uint8_t *)&imu_frame, sizeof(imu_frame));
}

/* ============================================================
 * 组合导航结果帧打包 / 发送
 * ============================================================ */

static Nav_Frame_t nav_frame;
static uint16_t nav_frame_counter;

/**
 * @brief  打包并发送一帧组合导航结果
 *
 * 打包规则与 imu_send_frame 一致:
 *   帧头 0x55 0xAA | type 0x81 | uint16 cnt | att[3]/vn[3]/pos[3] (float)
 *   | checksum (type ~ pos[2] 累加和) | 帧尾 0x0D 0x0A
 */
void nav_send_frame(const double att_rad[3], const double vn_mps[3], const double pos[3])
{
    nav_frame.header1    = 0x55U;
    nav_frame.header2    = 0xAAU;
    nav_frame.frame_type = NAV_FRAME_TYPE;
    nav_frame.cnt        = nav_frame_counter++;

    for (int i = 0; i < 3; ++i)
    {
        nav_frame.att[i] = (float)att_rad[i];
        nav_frame.vn[i]  = (float)vn_mps[i];
        nav_frame.pos[i] = (float)pos[i];
    }

    /* checksum: 从 frame_type 到 pos[2] */
    nav_frame.checksum = imu_checksum(
        (uint8_t *)&nav_frame.frame_type,
        sizeof(nav_frame) - offsetof(Nav_Frame_t, checksum) - 2 /* 减 end1/end2 */);
    nav_frame.end1 = 0x0DU;
    nav_frame.end2 = 0x0AU;

    (void)UART0_DMA_Send((const uint8_t *)&nav_frame, sizeof(nav_frame));
}
