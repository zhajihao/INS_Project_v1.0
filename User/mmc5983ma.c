#include "mmc5983ma.h"

/* ============================================================
 *                      I2C 底层驱动
 * ============================================================ */

/**
 * @brief  初始化 I2C0 GPIO 和 I2C 外设
 */
static void MMC5983MA_I2C_Init(void)
{
    /* 使能 GPIO 和 I2C 时钟 */
    rcu_periph_clock_enable(MMC5983MA_I2C_GPIO_CLK);
    rcu_periph_clock_enable(MMC5983MA_I2C_CLK);

    /* 配置 GPIO 为复用开漏模式 */
    gpio_af_set(MMC5983MA_I2C_GPIO_PORT, MMC5983MA_I2C_GPIO_AF,
                MMC5983MA_I2C_SCL_PIN | MMC5983MA_I2C_SDA_PIN);
    gpio_mode_set(MMC5983MA_I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                  MMC5983MA_I2C_SCL_PIN | MMC5983MA_I2C_SDA_PIN);
    gpio_output_options_set(MMC5983MA_I2C_GPIO_PORT, GPIO_OTYPE_OD,
                            GPIO_OSPEED_50MHZ,
                            MMC5983MA_I2C_SCL_PIN | MMC5983MA_I2C_SDA_PIN);

    /* 复位 I2C */
    i2c_deinit(MMC5983MA_I2C);

    /* 配置 I2C 时钟 */
    i2c_clock_config(MMC5983MA_I2C, MMC5983MA_I2C_SPEED, I2C_DTCY_2);

    /* 配置 I2C 地址模式（7 位） */
    i2c_mode_addr_config(MMC5983MA_I2C, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, 0x00);

    /* 使能 I2C */
    i2c_enable(MMC5983MA_I2C);

    /* 使能 ACK */
    i2c_ack_config(MMC5983MA_I2C, I2C_ACK_ENABLE);
}

/**
 * @brief  I2C 等待标志位
 */
static uint8_t MMC5983MA_I2C_WaitFlag(i2c_flag_enum flag, uint32_t timeout)
{
    while (timeout--)
    {
        if (i2c_flag_get(MMC5983MA_I2C, flag))
        {
            return 0;
        }
    }
    return 1; /* 超时 */
}

/**
 * @brief  I2C 写单个寄存器
 */
uint8_t MMC5983MA_I2C_WriteReg(uint8_t reg, uint8_t data)
{
    uint32_t timeout = 100000;

    /* 等待总线空闲 */
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 1;
    }

    /* 发送 START */
    i2c_start_on_bus(MMC5983MA_I2C);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_SBSEND, timeout))
        return 2;

    /* 发送器件地址 + 写 */
    i2c_master_addressing(MMC5983MA_I2C, MMC5983MA_I2C_ADDR << 1, I2C_TRANSMITTER);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_ADDSEND, timeout))
        return 3;
    i2c_flag_clear(MMC5983MA_I2C, I2C_FLAG_ADDSEND);

    /* 发送寄存器地址 */
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_TBE, timeout))
        return 4;
    i2c_data_transmit(MMC5983MA_I2C, reg);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_BTC, timeout))
        return 5;

    /* 发送数据 */
    i2c_data_transmit(MMC5983MA_I2C, data);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_BTC, timeout))
        return 6;

    /* 发送 STOP */
    i2c_stop_on_bus(MMC5983MA_I2C);
    timeout = 100000;
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 7;
    }

    return 0;
}

/**
 * @brief  I2C 读单个寄存器
 */
uint8_t MMC5983MA_I2C_ReadReg(uint8_t reg, uint8_t *data)
{
    uint32_t timeout = 100000;

    /* 等待总线空闲 */
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 1;
    }

    /* 发送 START */
    i2c_start_on_bus(MMC5983MA_I2C);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_SBSEND, timeout))
        return 2;

    /* 发送器件地址 + 写 */
    i2c_master_addressing(MMC5983MA_I2C, MMC5983MA_I2C_ADDR << 1, I2C_TRANSMITTER);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_ADDSEND, timeout))
        return 3;
    i2c_flag_clear(MMC5983MA_I2C, I2C_FLAG_ADDSEND);

    /* 发送寄存器地址 */
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_TBE, timeout))
        return 4;
    i2c_data_transmit(MMC5983MA_I2C, reg);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_BTC, timeout))
        return 5;

    /* 重新发送 START */
    i2c_start_on_bus(MMC5983MA_I2C);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_SBSEND, timeout))
        return 6;

    /* 发送器件地址 + 读 */
    i2c_master_addressing(MMC5983MA_I2C, MMC5983MA_I2C_ADDR << 1, I2C_RECEIVER);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_ADDSEND, timeout))
        return 7;
    i2c_flag_clear(MMC5983MA_I2C, I2C_FLAG_ADDSEND);

    /* 单字节接收，先发 NACK */
    i2c_ack_config(MMC5983MA_I2C, I2C_ACK_DISABLE);

    /* 等待接收完成 */
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_RBNE, timeout))
        return 8;
    *data = i2c_data_receive(MMC5983MA_I2C);

    /* 发送 STOP */
    i2c_stop_on_bus(MMC5983MA_I2C);
    timeout = 100000;
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 9;
    }

    /* 恢复 ACK */
    i2c_ack_config(MMC5983MA_I2C, I2C_ACK_ENABLE);

    return 0;
}

/**
 * @brief  I2C 连续读多个寄存器
 */
uint8_t MMC5983MA_I2C_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint32_t timeout = 100000;
    uint16_t i;

    if (len == 0 || buf == NULL)
        return 1;

    /* 等待总线空闲 */
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 2;
    }

    /* 发送 START */
    i2c_start_on_bus(MMC5983MA_I2C);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_SBSEND, timeout))
        return 3;

    /* 发送器件地址 + 写 */
    i2c_master_addressing(MMC5983MA_I2C, MMC5983MA_I2C_ADDR << 1, I2C_TRANSMITTER);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_ADDSEND, timeout))
        return 4;
    i2c_flag_clear(MMC5983MA_I2C, I2C_FLAG_ADDSEND);

    /* 发送寄存器地址 */
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_TBE, timeout))
        return 5;
    i2c_data_transmit(MMC5983MA_I2C, reg);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_BTC, timeout))
        return 6;

    /* 重新发送 START */
    i2c_start_on_bus(MMC5983MA_I2C);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_SBSEND, timeout))
        return 7;

    /* 发送器件地址 + 读 */
    i2c_master_addressing(MMC5983MA_I2C, MMC5983MA_I2C_ADDR << 1, I2C_RECEIVER);
    if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_ADDSEND, timeout))
        return 8;
    i2c_flag_clear(MMC5983MA_I2C, I2C_FLAG_ADDSEND);

    /* 使能 ACK */
    i2c_ack_config(MMC5983MA_I2C, I2C_ACK_ENABLE);

    /* 读取 len 个字节 */
    for (i = 0; i < len; i++)
    {
        /* 最后一个字节前发送 NACK */
        if (i == (len - 1))
        {
            i2c_ack_config(MMC5983MA_I2C, I2C_ACK_DISABLE);
        }

        if (MMC5983MA_I2C_WaitFlag(I2C_FLAG_RBNE, timeout))
            return 9;
        buf[i] = i2c_data_receive(MMC5983MA_I2C);
    }

    /* 发送 STOP */
    i2c_stop_on_bus(MMC5983MA_I2C);
    timeout = 100000;
    while (i2c_flag_get(MMC5983MA_I2C, I2C_FLAG_I2CBSY))
    {
        if (timeout-- == 0)
            return 10;
    }

    /* 恢复 ACK */
    i2c_ack_config(MMC5983MA_I2C, I2C_ACK_ENABLE);

    return 0;
}

/* ============================================================
 *                   MMC5983MA 驱动实现
 * ============================================================ */

/**
 * @brief  初始化 MMC5983MA
 * @return 0 成功，其他 失败
 */
uint8_t MMC5983MA_Init(void)
{
    uint8_t id = 0;

    /* 1. 初始化 I2C */
    MMC5983MA_I2C_Init();

    /* 2. 软复位 */
    if (MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST) != 0)
    {
        return 1;
    }
    /* 等待复位完成（手册要求至少 10ms） */
    for (volatile uint32_t i = 0; i < 500000; i++)
        ;

    /* 3. 读取 Product ID 验证通信 */
    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_PRODUCT_ID, &id) != 0)
    {
        return 2;
    }
    if (id != 0x30)
    {
        return 3; /* ID 不匹配 */
    }

    /* 4. 配置 Control 1: 带宽 = 01 (100Hz) */
    if (MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_BW_01) != 0)
    {
        return 4;
    }

    /* 5. 执行一次 SET 操作，确保传感器处于最佳状态 */
    if (MMC5983MA_SetOperation() != 0)
    {
        return 5;
    }

    return 0;
}

/**
 * @brief  执行 SET 操作
 */
uint8_t MMC5983MA_SetOperation(void)
{
    uint8_t ctrl0 = 0;

    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_CTRL0, &ctrl0) != 0)
    {
        return 1;
    }
    ctrl0 |= MMC5983MA_CTRL0_SET;
    if (MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL0, ctrl0) != 0)
    {
        return 2;
    }

    /* 等待 SET 完成，手册要求约 500us */
    for (volatile uint32_t i = 0; i < 30000; i++)
        ;

    return 0;
}

/**
 * @brief  执行 RESET 操作
 */
uint8_t MMC5983MA_ResetOperation(void)
{
    uint8_t ctrl0 = 0;

    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_CTRL0, &ctrl0) != 0)
    {
        return 1;
    }
    ctrl0 |= MMC5983MA_CTRL0_RESET;
    if (MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL0, ctrl0) != 0)
    {
        return 2;
    }

    /* 等待 RESET 完成 */
    for (volatile uint32_t i = 0; i < 30000; i++)
        ;

    return 0;
}

/**
 * @brief  触发一次磁场测量
 */
uint8_t MMC5983MA_TriggerMeasurement(void)
{
    uint8_t ctrl0 = 0;

    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_CTRL0, &ctrl0) != 0)
    {
        return 1;
    }
    ctrl0 |= MMC5983MA_CTRL0_TM_M;
    return MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL0, ctrl0);
}

/**
 * @brief  等待测量完成
 * @param  timeout 超时计数
 */
uint8_t MMC5983MA_WaitMeasurementDone(uint32_t timeout)
{
    uint8_t status = 0;

    while (timeout--)
    {
        if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != 0)
        {
            return 1;
        }
        if (status & MMC5983MA_STATUS_MEAS_M_DONE)
        {
            return 0; /* 测量完成 */
        }
    }
    return 2; /* 超时 */
}

/**
 * @brief  读取原始数据（不包含 SET/RESET 处理）
 * @param  data 输出数据结构
 * @note   18 位无符号格式，零磁场对应 131072
 */
uint8_t MMC5983MA_ReadRawData(MMC5983MA_Data_t *data)
{
    uint8_t buf[7] = {0};

    if (data == NULL)
        return 1;

    /* 连续读取 7 个字节：Xout0, Xout1, Yout0, Yout1, Zout0, Zout1, XYZout2 */
    if (MMC5983MA_I2C_ReadMulti(MMC5983MA_REG_XOUT0, buf, 7) != 0)
    {
        return 2;
    }

    /* 组合 18 位数据（无符号） */
    data->x_raw = ((int32_t)buf[0] << 10) | ((int32_t)buf[1] << 2) | ((int32_t)(buf[6] >> 6) & 0x03);
    data->y_raw = ((int32_t)buf[2] << 10) | ((int32_t)buf[3] << 2) | ((int32_t)(buf[6] >> 4) & 0x03);
    data->z_raw = ((int32_t)buf[4] << 10) | ((int32_t)buf[5] << 2) | ((int32_t)(buf[6] >> 2) & 0x03);

    /* 换算为高斯值（有符号）：先减去零点偏移，再除以灵敏度 */
    data->x_gauss = ((float)data->x_raw - MMC5983MA_18BIT_ZERO) / MMC5983MA_COUNTS_PER_GAUSS;
    data->y_gauss = ((float)data->y_raw - MMC5983MA_18BIT_ZERO) / MMC5983MA_COUNTS_PER_GAUSS;
    data->z_gauss = ((float)data->z_raw - MMC5983MA_18BIT_ZERO) / MMC5983MA_COUNTS_PER_GAUSS;

    return 0;
}

/**
 * @brief  使用 SET/RESET 方式读取数据（消除偏移，最精确）
 * @param  data 输出数据结构
 * @note   流程：SET → 测量 → RESET → 测量 → (SET值 - RESET值)/2
 */
uint8_t MMC5983MA_ReadDataWithSetReset(MMC5983MA_Data_t *data)
{
    MMC5983MA_Data_t set_data, reset_data;

    if (data == NULL)
        return 1;

    /* ---- 第 1 步：SET + 测量 ---- */
    if (MMC5983MA_SetOperation() != 0)
        return 2;
    if (MMC5983MA_TriggerMeasurement() != 0)
        return 3;
    if (MMC5983MA_WaitMeasurementDone(100000) != 0)
        return 4;
    if (MMC5983MA_ReadRawData(&set_data) != 0)
        return 5;

    /* ---- 第 2 步：RESET + 测量 ---- */
    if (MMC5983MA_ResetOperation() != 0)
        return 6;
    if (MMC5983MA_TriggerMeasurement() != 0)
        return 7;
    if (MMC5983MA_WaitMeasurementDone(100000) != 0)
        return 8;
    if (MMC5983MA_ReadRawData(&reset_data) != 0)
        return 9;

    /* ---- 第 3 步：计算 (SET - RESET) / 2 ---- */
    data->x_raw = (set_data.x_raw - reset_data.x_raw) / 2;
    data->y_raw = (set_data.y_raw - reset_data.y_raw) / 2;
    data->z_raw = (set_data.z_raw - reset_data.z_raw) / 2;

    data->x_gauss = ((float)data->x_raw) / MMC5983MA_COUNTS_PER_GAUSS;
    data->y_gauss = ((float)data->y_raw) / MMC5983MA_COUNTS_PER_GAUSS;
    data->z_gauss = ((float)data->z_raw) / MMC5983MA_COUNTS_PER_GAUSS;

    return 0;
}

/**
 * @brief  读取温度
 * @param  temp 输出温度值（℃）
 */
uint8_t MMC5983MA_ReadTemperature(float *temp)
{
    uint8_t ctrl0 = 0;
    uint8_t tout = 0;
    uint32_t timeout = 100000;

    if (temp == NULL)
        return 1;

    /* 触发温度测量 */
    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_CTRL0, &ctrl0) != 0)
        return 2;
    ctrl0 |= MMC5983MA_CTRL0_TM_T;
    if (MMC5983MA_I2C_WriteReg(MMC5983MA_REG_CTRL0, ctrl0) != 0)
        return 3;

    /* 等待温度测量完成 */
    while (timeout--)
    {
        uint8_t status = 0;
        if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_STATUS, &status) != 0)
            return 4;
        if (status & MMC5983MA_STATUS_MEAS_T_DONE)
            break;
    }
    if (timeout == 0)
        return 5;

    /* 读取温度寄存器 */
    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_TOUT, &tout) != 0)
        return 6;

    /* 温度换算：0.8℃/LSB，0 表示 -75℃ */
    *temp = (float)tout * 0.8f - 75.0f;

    return 0;
}

/**
 * @brief  自检（读取 Product ID）
 */
uint8_t MMC5983MA_SelfTest(void)
{
    uint8_t id = 0;
    if (MMC5983MA_I2C_ReadReg(MMC5983MA_REG_PRODUCT_ID, &id) != 0)
        return 1;
    return (id == 0x30) ? 0 : 2;
}
/**
 * @brief  温度滑动平均滤波
 *
 * MMC5983MA 温度输出仅 8 位分辨率（0.8℃/LSB），原始值跳变明显，
 * 故采用滑动平均滤波（窗口 MA_WINDOW_SIZE = 8）平滑输出。
 */
#define MA_WINDOW_SIZE 8

float temp_buffer[MA_WINDOW_SIZE];
int ma_index = 0;
float ma_sum = 0;
int ma_count = 0;

/**
 * @brief  初始化温度滤波缓冲区
 */
void temp_filter_init()
{
    for (int i = 0; i < MA_WINDOW_SIZE; i++)
    {
        temp_buffer[i] = 0;
    }
    ma_index = 0;
    ma_sum = 0;
    ma_count = 0;
}

/**
 * @brief  温度滤波更新
 *
 * 每次读取新温度后调用此函数，将新值写入循环缓冲区、替换最旧值、
 * 返回当前窗口滑动平均。
 *
 * @param  new_temp  新读取的温度值 (℃)
 * @return           当前窗口平均温度 (℃)
 */
float temp_filter_update(float new_temp)
{
    /* 减去将被覆盖的旧值 */
    ma_sum -= temp_buffer[ma_index];
    /* 存入新值 */
    temp_buffer[ma_index] = new_temp;
    ma_sum += new_temp;

    /* 更新循环索引 */
    ma_index = (ma_index + 1) % MA_WINDOW_SIZE;

    /* 填充计数，缓冲区未满时按实际数量求平均 */
    if (ma_count < MA_WINDOW_SIZE)
    {
        ma_count++;
    }

    /* 返回平均值 */
    return ma_sum / ma_count;
}
