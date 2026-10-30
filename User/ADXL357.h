#ifndef ADXL357_H
#define ADXL357_H

#include <stdint.h>

/* ============= 寄存器地址定义 ============= */
#define ADXL357_REG_DEVID_AD 0x00U
#define ADXL357_REG_DEVID_MST 0x01U
#define ADXL357_REG_PARTID 0x02U
#define ADXL357_REG_STATUS 0x04U
#define ADXL357_STATUS_DATA_RDY (1U << 0) /* 加速度数据就绪标志 */
#define ADXL357_REG_TEMP2 0x06U
#define ADXL357_REG_XDATA3 0x08U
#define ADXL357_REG_FILTER 0x28U
#define ADXL357_FILTER_HPF_OFF (0U << 4)  /* 高通滤波关闭 */
#define ADXL357_FILTER_ODR_1000 (2U << 0) /* 输出数据率 1000 Hz */
#define ADXL357_REG_FIFO_SAMPLES 0x29U
#define ADXL357_REG_RANGE 0x2CU
#define ADXL357_RANGE_10G (1U << 0)    /* ±10g 量程 */
#define ADXL357_INT_POL_HIGH (1U << 6) /* 中断高电平有效 */
#define ADXL357_REG_POWER_CTL 0x2DU
#define ADXL357_POWER_MEASURE 0U /* 测量模式 */

/**
 * @brief  ADXL357 加速度计全局数据结构
 */
typedef struct
{
    float x_g;           /**< X 轴加速度 (g) */
    float y_g;           /**< Y 轴加速度 (g) */
    float z_g;           /**< Z 轴加速度 (g) */
    float temperature_c; /**< 芯片温度 (℃) */
} adxl357_data_t;

/** @brief  ADXL357 全局数据实例（存放最新一次采样结果） */
extern adxl357_data_t adxl357_data;

uint8_t adxl357_init(void);

uint8_t adxl357_read_accel(int32_t *raw_x, int32_t *raw_y, int32_t *raw_z);

void adxl357_convert_to_g(int32_t raw_x, int32_t raw_y, int32_t raw_z,
                          float *x_g, float *y_g, float *z_g);

uint16_t adxl357_read_temp(void);

float adxl357_temp_to_celsius(uint16_t temp_lsb);

#endif
