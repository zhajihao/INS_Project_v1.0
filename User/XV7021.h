#ifndef XV7021_H
#define XV7021_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============= 寄存器地址定义 ============= */
#define XV7021BB_REG_DSPCTL1  0x01U   /* DSP 控制 1 */
#define XV7021BB_REG_DSPCTL2  0x02U   /* DSP 控制 2 (notch/LPF) */
#define XV7021BB_REG_DSPCTL3  0x03U   /* DSP 控制 3 (校准/采样分频) */
#define XV7021BB_REG_STATUS   0x04U   /* 状态寄存器 */
#define XV7021BB_REG_SLEEP_IN 0x05U   /* 进入休眠命令 */
#define XV7021BB_REG_SLEEP_OUT 0x06U  /* 退出休眠命令 */
#define XV7021BB_REG_STANDBY  0x07U   /* 进入待机命令 */
#define XV7021BB_REG_TEMP     0x08U   /* 温度寄存器 */
#define XV7021BB_REG_SWRESET  0x09U   /* 软复位命令 */
#define XV7021BB_REG_RATE     0x0AU   /* 角速度数据寄存器 */
#define XV7021BB_REG_OUTCTL1  0x0BU   /* 输出控制 1 (位宽/格式) */
#define XV7021BB_REG_AUTOC    0x0CU   /* 自动校准命令 */
#define XV7021BB_REG_TSFORMAT 0x1CU   /* 温度格式寄存器 */
#define XV7021BB_REG_IFCTL    0x1FU   /* 接口控制寄存器 */

/* ============= 编译期配置项 ============= */
#define XV7021BB_RATE_24BIT       1U   /* 1=24位角速度; 0=16位 */
#define XV7021BB_LPF_FC_CODE      0U   /* 低通截止频率: 0=10Hz */
#define XV7021BB_LPF_ORDER_CODE   0U   /* 低通阶数: 0=2阶 */
#define XV7021BB_NOTCH_ENABLE     0U   /* 陷波滤波: 0=关闭 */
#define XV7021BB_FS_DIV_CODE      3U   /* 采样分频: 3=÷8 */
#define XV7021BB_TEMP_FORMAT_CODE 2U   /* 温度格式: 2=12位 */

/* ============= 类型定义 ============= */

/** @brief  陀螺仪编号 (板上共 3 颗，各自独立 SPI 从机) */
typedef enum
{
    XV7021BB_1 = 0,
    XV7021BB_2 = 1,
    XV7021BB_3 = 2
} xv7021bb_id_t;

/** @brief  API 返回状态码 */
typedef enum
{
    XV7021BB_OK = 0,          /**< 成功 */
    XV7021BB_ERR_PARAM = -1,  /**< 参数非法 */
    XV7021BB_ERR_TIMEOUT = -2, /**< SPI DMA 超时 */
    XV7021BB_ERR_STATUS = -3,  /**< 芯片状态异常 (上电就绪超时) */
    XV7021BB_ERR_VERIFY = -4   /**< 回读校验失败 */
} xv7021bb_status_t;

/** @brief  角速度数据 (原始值 + °/s 换算值) */
typedef struct
{
    int32_t raw;   /**< 原始寄存器值 (24 位或 16 位) */
    float dps;     /**< 换算后角速度 (°/s) */
} xv7021bb_rate_t;

/** @brief  温度数据 (原始值 + 摄氏度换算值) */
typedef struct
{
    int16_t raw;    /**< 原始温度寄存器值 */
    float degC;     /**< 换算后摄氏温度 */
    uint8_t valid;  /**< 读取成功标志 */
} xv7021bb_temp_t;

/** @brief  芯片状态解析结果 */
typedef struct
{
    uint8_t status;     /**< 原始状态寄存器值 */
    uint8_t proc_ok;    /**< bit3: 处理状态 OK */
    uint8_t power_state;/**< bit[2:0]: 电源状态 */
} xv7021bb_status_info_t;

/** @brief  全部陀螺仪的累积采样数据 */
typedef struct
{
    xv7021bb_rate_t gyro[3];       /**< 三颗陀螺仪最新角速度 */
    xv7021bb_temp_t temp[3];       /**< 三颗陀螺仪最新温度 */
    uint8_t status[3];             /**< 三颗陀螺仪各自的状态字 */
    uint32_t sample_counter;       /**< 累计采样次数 */
} xv7021bb_all_data_t;

/** @brief  XV7021BB 全局数据实例 */
extern xv7021bb_all_data_t xv7021_data;

/* ============= API ============= */

void XV7021BB_BoardInit(void);

void XV7021BB_SPI0_Init(void);

xv7021bb_status_t XV7021BB_Init(xv7021bb_id_t id);

xv7021bb_status_t XV7021BB_InitAll(void);

xv7021bb_status_t XV7021BB_RegWrite(xv7021bb_id_t id, uint8_t reg, uint8_t data);

xv7021bb_status_t XV7021BB_RegRead(xv7021bb_id_t id, uint8_t reg, uint8_t *data);

xv7021bb_status_t XV7021BB_ReadRate16(xv7021bb_id_t id, int16_t *raw);

xv7021bb_status_t XV7021BB_ReadRate24(xv7021bb_id_t id, int32_t *raw);

xv7021bb_status_t XV7021BB_ReadRate(xv7021bb_id_t id, xv7021bb_rate_t *rate);

xv7021bb_status_t XV7021BB_ReadTemp(xv7021bb_id_t id, xv7021bb_temp_t *temp);

xv7021bb_status_t XV7021BB_ReadStatus(xv7021bb_id_t id, xv7021bb_status_info_t *status);

xv7021bb_status_t XV7021BB_ZeroRateCalibrate(xv7021bb_id_t id);

xv7021bb_status_t XV7021BB_ZeroRateCalibrateAll(void);

uint8_t XV7021BB_SampleTask(void);

const xv7021bb_all_data_t *XV7021BB_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
