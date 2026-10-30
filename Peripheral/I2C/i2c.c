#include "i2c.h"

/**
 * @brief  初始化 I2C0
 *
 * 配置内容:
 *   - PB6 / PB7 复用 AF4 (I2C0)
 *   - 开漏输出 + 上拉
 *   - 100 kHz 标准模式
 *   - 自身 7 位地址 0x30
 *   - 使能 ACK
 */
void i2c0_config(void)
{
    /* 使能 GPIO 时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* 使能 I2C0 时钟 */
    rcu_periph_clock_enable(RCU_I2C0);

    /* I2C0 GPIO 配置: PB6=SCL, PB7=SDA */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_6);
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_7);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    /* I2C 时钟: 100 kHz, Duty=50% */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C 自身地址: 7 位, 0x30 */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    /* 使能 I2C0 + ACK */
    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

/**
 * @brief  从 I2C0 默认从机读取 2 字节数据
 *
 * 完整流程: START → 发送从机地址 (写) → 重新 START →
 *          发送从机地址 (读) → 读 2 字节 (最后一字节前先发 NACK) → STOP
 *
 * @param  i2c_receiver  接收缓冲区指针（必须至少 2 字节）
 * @note   此函数为阻塞轮询实现，**不要在中断中调用**。
 */
void i2c0_read_data(uint8_t *i2c_receiver)
{
    int i;

    /* 最后一个字节之前发 NACK，避免总线上继续被应答 */
    i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
    /* 等待总线空闲 */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* 发 START */
    i2c_start_on_bus(I2C0);
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    /* 发从机地址 + 写方向 */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
    /* 收到从机 ACK 前禁用本地 ACK (单字节接收场景) */
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* 等上一字节转移完成 */
    while (!i2c_flag_get(I2C0, I2C_FLAG_BTC));

    /* 读第 1 字节 */
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    i2c_receiver[i++] = i2c_data_receive(I2C0);
    /* 读第 2 字节 */
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    i2c_receiver[i++] = i2c_data_receive(I2C0);

    /* 发 STOP */
    i2c_stop_on_bus(I2C0);
    while (I2C_CTL0(I2C0) & 0x0200); /* 等待 STOP 生成完成 */

    /* 恢复 ACK 配置 */
    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}
