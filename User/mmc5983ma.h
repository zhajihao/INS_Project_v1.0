#ifndef __MMC5983MA_H
#define __MMC5983MA_H

#include "gd32f4xx.h"
#include <stdint.h>
#include <math.h>

#define NULL 0
/* ======================== I2C 配置 ======================== */
#define MMC5983MA_I2C I2C0
#define MMC5983MA_I2C_CLK RCU_I2C0
#define MMC5983MA_I2C_GPIO_CLK RCU_GPIOB
#define MMC5983MA_I2C_GPIO_PORT GPIOB
#define MMC5983MA_I2C_SCL_PIN GPIO_PIN_6
#define MMC5983MA_I2C_SDA_PIN GPIO_PIN_7
#define MMC5983MA_I2C_GPIO_AF GPIO_AF_4
#define MMC5983MA_I2C_SPEED 400000U /* 400kHz 快速模式 */

/* ======================== 器件地址 ======================== */
#define MMC5983MA_I2C_ADDR 0x30U /* 7 位地址 0110000 */

/* ======================== 寄存器地址 ======================== */
#define MMC5983MA_REG_XOUT0 0x00U
#define MMC5983MA_REG_XOUT1 0x01U
#define MMC5983MA_REG_YOUT0 0x02U
#define MMC5983MA_REG_YOUT1 0x03U
#define MMC5983MA_REG_ZOUT0 0x04U
#define MMC5983MA_REG_ZOUT1 0x05U
#define MMC5983MA_REG_XYZOUT2 0x06U
#define MMC5983MA_REG_TOUT 0x07U
#define MMC5983MA_REG_STATUS 0x08U
#define MMC5983MA_REG_CTRL0 0x09U
#define MMC5983MA_REG_CTRL1 0x0AU
#define MMC5983MA_REG_CTRL2 0x0BU
#define MMC5983MA_REG_CTRL3 0x0CU
#define MMC5983MA_REG_PRODUCT_ID 0x2FU

/* ======================== Control 0 位定义 ======================== */
#define MMC5983MA_CTRL0_TM_M (1U << 0)     /* 触发磁场测量 */
#define MMC5983MA_CTRL0_TM_T (1U << 1)     /* 触发温度测量 */
#define MMC5983MA_CTRL0_INT_MEAS (1U << 2) /* 测量完成中断使能 */
#define MMC5983MA_CTRL0_SET (1U << 3)      /* 执行 SET */
#define MMC5983MA_CTRL0_RESET (1U << 4)    /* 执行 RESET */
#define MMC5983MA_CTRL0_AUTO_SR (1U << 5)  /* 自动 SET/RESET 使能 */
#define MMC5983MA_CTRL0_OTP_RD (1U << 6)   /* OTP 读取 */

/* ======================== Control 1 位定义 ======================== */
#define MMC5983MA_CTRL1_SW_RST (1U << 7) /* 软复位 */
#define MMC5983MA_CTRL1_BW_00 (0U << 0)  /* 带宽 00 */
#define MMC5983MA_CTRL1_BW_01 (1U << 0)  /* 带宽 01 */
#define MMC5983MA_CTRL1_BW_10 (2U << 0)  /* 带宽 10 */
#define MMC5983MA_CTRL1_BW_11 (3U << 0)  /* 带宽 11 */

/* ======================== Control 2 位定义 ======================== */
#define MMC5983MA_CTRL2_CMM_EN (1U << 7)       /* 连续测量使能 */
#define MMC5983MA_CTRL2_EN_PRD_SET (1U << 6)   /* 自动 SET 周期使能 */
#define MMC5983MA_CTRL2_PRD_SET_1 (0U << 3)    /* 每 1 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_25 (1U << 3)   /* 每 25 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_75 (2U << 3)   /* 每 75 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_100 (3U << 3)  /* 每 100 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_250 (4U << 3)  /* 每 250 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_500 (5U << 3)  /* 每 500 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_1000 (6U << 3) /* 每 1000 次测量 SET */
#define MMC5983MA_CTRL2_PRD_SET_2000 (7U << 3) /* 每 2000 次测量 SET */
#define MMC5983MA_CTRL2_CM_FREQ_1HZ (1U << 0)
#define MMC5983MA_CTRL2_CM_FREQ_10HZ (2U << 0)
#define MMC5983MA_CTRL2_CM_FREQ_20HZ (3U << 0)
#define MMC5983MA_CTRL2_CM_FREQ_50HZ (4U << 0)
#define MMC5983MA_CTRL2_CM_FREQ_100HZ (5U << 0)

/* ======================== Status 位定义 ======================== */
#define MMC5983MA_STATUS_MEAS_M_DONE (1U << 0)
#define MMC5983MA_STATUS_MEAS_T_DONE (1U << 1)
#define MMC5983MA_STATUS_OTP_RD_DONE (1U << 4)

/* ======================== 常量 ======================== */
#define MMC5983MA_18BIT_ZERO 131072.0f      /* 18 位模式下零磁场对应值 2^17 */
#define MMC5983MA_COUNTS_PER_GAUSS 16384.0f /* 灵敏度 16384 Counts/G (18bit) */

/* ======================== 数据结构 ======================== */
/**
 * @brief  MMC5983MA 磁力计输出数据
 *
 * 字段 x_raw/y_raw/z_raw 为 18 位无符号原始值（零磁场 = 131072）；
 * x_gauss/y_gauss/z_gauss 为换算后的有符号磁场值（单位 Gauss）。
 */
typedef struct
{
    int32_t x_raw;     /**< 原始 X 值 (18位无符号) */
    int32_t y_raw;     /**< 原始 Y 值 (18位无符号) */
    int32_t z_raw;     /**< 原始 Z 值 (18位无符号) */
    float x_gauss;     /**< 换算后 X 磁场值 (Gauss) */
    float y_gauss;     /**< 换算后 Y 磁场值 (Gauss) */
    float z_gauss;     /**< 换算后 Z 磁场值 (Gauss) */
    float temperature; /**< 温度 (℃) */
} MMC5983MA_Data_t;

/* ======================== 高层 API ======================== */

uint8_t MMC5983MA_Init(void);

uint8_t MMC5983MA_SetOperation(void);

uint8_t MMC5983MA_ResetOperation(void);

uint8_t MMC5983MA_TriggerMeasurement(void);

uint8_t MMC5983MA_WaitMeasurementDone(uint32_t timeout);

uint8_t MMC5983MA_ReadRawData(MMC5983MA_Data_t *data);

uint8_t MMC5983MA_ReadDataWithSetReset(MMC5983MA_Data_t *data);

uint8_t MMC5983MA_ReadTemperature(float *temp);

uint8_t MMC5983MA_SelfTest(void);

/* ======================== 底层 I2C 接口 ======================== */

uint8_t MMC5983MA_I2C_WriteReg(uint8_t reg, uint8_t data);

uint8_t MMC5983MA_I2C_ReadReg(uint8_t reg, uint8_t *data);

uint8_t MMC5983MA_I2C_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* __MMC5983MA_H */
