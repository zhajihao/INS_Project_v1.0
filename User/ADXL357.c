#include "ADXL357.h"
#include "gd32f4xx.h"
#include "spi.h"
#include "systick.h"

#define ADXL357_SPI_MAX_BURST 9U
#define ADXL357_COUNTS_PER_G 51200.0f

adxl357_data_t adxl357_data;

/**
 * @brief  将 ADXL357 的片选信号 (PB12) 拉低
 */
static void adxl357_cs_low(void)
{
    gpio_bit_reset(GPIOB, GPIO_PIN_12);
}

/**
 * @brief  将 ADXL357 的片选信号 (PB12) 拉高
 */
static void adxl357_cs_high(void)
{
    gpio_bit_set(GPIOB, GPIO_PIN_12);
}

/**
 * @brief  初始化 SPI1 外设及 GPIO，用于与 ADXL357 通信
 *
 * 配置 SPI1 为 8bit MSB 主模式，时钟极性低 + 1边沿采样，
 * 预分频 16 (APB2@84MHz → SPI 5.25MHz)，并启用 DMA 传输。
 * GPIO 引脚：PB13=SCK, PB14=MISO, PB15=MOSI (AF5)，PB12=CS (软件控制)。
 */
static void adxl357_spi_init(void)
{
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    adxl357_cs_high();

    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.prescale = SPI_PSC_16;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1);
    spi1_dma_init();
}

/**
 * @brief  通过 SPI1 DMA 从 ADXL357 读取单个寄存器
 * @param  reg_addr  寄存器地址（7 位，函数内部会左移并设置 R/W 位）
 * @return 读取到的 8 位寄存器值；DMA 传输失败时返回 0
 */
static uint8_t adxl357_spi_read_byte(uint8_t reg_addr)
{
    uint8_t tx[2] = {(uint8_t)((reg_addr << 1) | 1U), 0xFFU};
    uint8_t rx[2] = {0U, 0U};

    adxl357_cs_low();
    if (spi1_dma_transfer(tx, rx, 2U) != 0)
    {
        adxl357_cs_high();
        return 0U;
    }
    adxl357_cs_high();
    return rx[1];
}

/**
 * @brief  通过 SPI1 DMA 向 ADXL357 写入单个寄存器
 * @param  reg_addr  寄存器地址（7 位）
 * @param  data      待写入的 8 位数据
 * @return 1 写入成功；0 DMA 传输失败
 */
static uint8_t adxl357_spi_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg_addr << 1), data};
    uint8_t rx[2];
    int result;

    adxl357_cs_low();
    result = spi1_dma_transfer(tx, rx, 2U);
    adxl357_cs_high();
    return (result == 0) ? 1U : 0U;
}

/**
 * @brief  通过 SPI1 DMA 连续读取 ADXL357 多个寄存器
 * @param  reg_addr  起始寄存器地址
 * @param  data      接收缓冲区指针（至少 length 字节）
 * @param  length    要读取的字节数（最大 ADXL357_SPI_MAX_BURST = 9）
 * @return 1 读取成功；0 参数非法或 DMA 传输失败
 */
static uint8_t adxl357_spi_read_multi(uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    uint8_t tx[ADXL357_SPI_MAX_BURST + 1U];
    uint8_t rx[ADXL357_SPI_MAX_BURST + 1U];
    uint8_t i;

    if (data == 0 || length == 0U || length > ADXL357_SPI_MAX_BURST)
        return 0U;

    tx[0] = (uint8_t)((reg_addr << 1) | 1U);
    for (i = 1U; i <= length; ++i)
        tx[i] = 0xFFU;

    adxl357_cs_low();
    if (spi1_dma_transfer(tx, rx, (uint16_t)length + 1U) != 0)
    {
        adxl357_cs_high();
        return 0U;
    }
    adxl357_cs_high();

    for (i = 0U; i < length; ++i)
        data[i] = rx[i + 1U];
    return 1U;
}

/**
 * @brief  读取并校验 ADXL357 的设备 ID
 *
 * 依次读取 DEVID_AD (期望 0xAD)、DEVID_MST (期望 0x1D)、
 * PARTID (期望 0xED 或 0xE9)，用于确认 SPI 通信正常且芯片型号匹配。
 *
 * @return 1 ID 校验通过；0 ID 不匹配或通信失败
 */
static uint8_t adxl357_read_device_id(void)
{
    uint8_t device_id = adxl357_spi_read_byte(ADXL357_REG_DEVID_AD);
    uint8_t manufacturer_id = adxl357_spi_read_byte(ADXL357_REG_DEVID_MST);
    uint8_t part_id = adxl357_spi_read_byte(ADXL357_REG_PARTID);

    return (device_id == 0xADU && manufacturer_id == 0x1DU &&
            (part_id == 0xEDU || part_id == 0xE9U));
}

/**
 * @brief  初始化 ADXL357 加速度计
 *
 * 流程：SPI 初始化 → 校验设备 ID → 配置量程 ±10g、HPF 关闭、ODR 1000Hz、
 * FIFO 10 采样 → 进入测量模式。
 *
 * @return 0 初始化成功；1 ID 不匹配或寄存器写入失败
 */
uint8_t adxl357_init(void)
{
    adxl357_spi_init();
    if (!adxl357_read_device_id())
        return 1U;

    if (!adxl357_spi_write_byte(ADXL357_REG_RANGE,
                                ADXL357_RANGE_10G | ADXL357_INT_POL_HIGH) ||
        !adxl357_spi_write_byte(ADXL357_REG_FILTER,
                                ADXL357_FILTER_HPF_OFF | ADXL357_FILTER_ODR_1000) ||
        !adxl357_spi_write_byte(ADXL357_REG_FIFO_SAMPLES, 10U) ||
        !adxl357_spi_write_byte(ADXL357_REG_POWER_CTL, ADXL357_POWER_MEASURE))
        return 1U;

    return 0U;
}

/**
 * @brief  读取 ADXL357 三轴加速度原始值
 *
 * 轮询 DATA_RDY 标志（最多 3 次，每次间隔 1ms），读取 9 字节 XDATA-ZDATA，
 * 组合为 20 位有符号原始值并做符号扩展。
 *
 * @param  x  X 轴原始值输出指针
 * @param  y  Y 轴原始值输出指针
 * @param  z  Z 轴原始值输出指针
 * @return 1 读取成功；0 指针为空、超时或 SPI 通信失败
 */
uint8_t adxl357_read_accel(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t data[9];
    uint8_t attempt;
    int32_t raw_x, raw_y, raw_z;

    if (x == 0 || y == 0 || z == 0)
        return 0U;

    for (attempt = 0U; attempt < 3U; ++attempt)
    {
        if ((adxl357_spi_read_byte(ADXL357_REG_STATUS) & ADXL357_STATUS_DATA_RDY) != 0U)
            break;
        if (attempt < 2U)
            delay_1ms(1U);
    }
    if (attempt == 3U || !adxl357_spi_read_multi(ADXL357_REG_XDATA3, data, 9U))
        return 0U;

    raw_x = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    raw_y = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    raw_z = ((int32_t)data[6] << 12) | ((int32_t)data[7] << 4) | ((int32_t)data[8] >> 4);

    if ((raw_x & 0x80000) != 0)
        raw_x |= (int32_t)0xFFF00000U;
    if ((raw_y & 0x80000) != 0)
        raw_y |= (int32_t)0xFFF00000U;
    if ((raw_z & 0x80000) != 0)
        raw_z |= (int32_t)0xFFF00000U;

    *x = raw_x;
    *y = raw_y;
    *z = raw_z;
    return 1U;
}

/**
 * @brief  将 ADXL357 原始加速度值转换为 g 单位
 *
 * 除以灵敏度 51200 counts/g（±10g 量程）。
 *
 * @param  raw_x  X 轴原始值
 * @param  raw_y  Y 轴原始值
 * @param  raw_z  Z 轴原始值
 * @param  gx     X 轴 g 值输出指针
 * @param  gy     Y 轴 g 值输出指针
 * @param  gz     Z 轴 g 值输出指针
 */
void adxl357_convert_to_g(int32_t raw_x, int32_t raw_y, int32_t raw_z,
                          float *gx, float *gy, float *gz)
{
    if (gx == 0 || gy == 0 || gz == 0)
        return;
    *gx = (float)raw_x / ADXL357_COUNTS_PER_G;
    *gy = (float)raw_y / ADXL357_COUNTS_PER_G;
    *gz = (float)raw_z / ADXL357_COUNTS_PER_G;
}

/**
 * @brief  读取 ADXL357 温度寄存器原始值
 * @return 16 位温度 LSB；0 表示 SPI 通信失败
 */
uint16_t adxl357_read_temp(void)
{
    uint8_t data[2] = {0U, 0U};
    if (!adxl357_spi_read_multi(ADXL357_REG_TEMP2, data, 2U))
        return 0U;
    return ((uint16_t)data[0] << 8) | data[1];
}

/**
 * @brief  将 ADXL357 温度 LSB 转换为摄氏度
 *
 * 使用 ADXL357 数据手册公式: T(°C) = 25 - (TEMP_LSB - 1852) / 9.05
 *
 * @param  temp_lsb  16 位温度寄存器值
 * @return 摄氏温度值（float）
 */
float adxl357_temp_to_celsius(uint16_t temp_lsb)
{
    return 25.0f - ((int32_t)temp_lsb - 1852) / 9.05f;
}
