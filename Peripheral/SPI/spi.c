#include "gd32f4xx.h"
#include "spi.h"
#include <stddef.h>

/* SPI0 和 SPI1 共用的 DMA 缓冲区。
 * 注意: 两个 SPI 端口不能同时发起 DMA 传输，否则会互相覆盖缓冲区数据。
 * 当前工程中 SPI0 供 XV7021BB 使用（固定 1 字节收/发），
 *      SPI1 供 ADXL357 使用（最多 9 字节），二者分时运行无冲突。 */
#define SPI_DMA_BUFFER_SIZE 16U
static uint8_t spi_dma_tx_buffer[SPI_DMA_BUFFER_SIZE];
static uint8_t spi_dma_rx_buffer[SPI_DMA_BUFFER_SIZE];

/* DMA 传输 / SPI 标志等待的超时计数 */
#define SPI_DMA_TIMEOUT 10000000U

/**
 * @brief  配置 SPI 的 TX / RX DMA 通道
 *
 * 两个通道均配置为: 8bit, 内存自增, 外设固定地址, 单次模式。
 * TX: Memory→Periph; RX: Periph→Memory。
 * 调用前需确保目标 DMA 时钟已开启（根据 dma == DMA0 / DMA1 自动选择）。
 *
 * @param  dma      DMA 外设 (DMA0 或 DMA1)
 * @param  tx_ch    TX DMA 通道
 * @param  rx_ch    RX DMA 通道
 * @param  subperi  DMA 子外设映射 (DMA_SUBPERI0 ~ DMA_SUBPERI3)
 * @param  spi      SPI 外设基地址 (SPI0 / SPI1)
 */
static void spi_dma_config(uint32_t dma, dma_channel_enum tx_ch, dma_channel_enum rx_ch,
                           dma_subperipheral_enum subperi, uint32_t spi)
{
  dma_single_data_parameter_struct p;
  rcu_periph_clock_enable(dma == DMA0 ? RCU_DMA0 : RCU_DMA1);

  /* TX: Memory → Periph */
  dma_deinit(dma, tx_ch);
  dma_single_data_para_struct_init(&p);
  p.direction = DMA_MEMORY_TO_PERIPH;
  p.memory0_addr = (uint32_t)spi_dma_tx_buffer;
  p.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  p.number = 1U;
  p.periph_addr = (uint32_t)&SPI_DATA(spi);
  p.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  p.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
  p.priority = DMA_PRIORITY_HIGH;
  dma_single_data_mode_init(dma, tx_ch, &p);
  dma_channel_subperipheral_select(dma, tx_ch, subperi);

  /* RX: Periph → Memory */
  dma_deinit(dma, rx_ch);
  p.direction = DMA_PERIPH_TO_MEMORY;
  p.memory0_addr = (uint32_t)spi_dma_rx_buffer;
  p.number = 1U;
  dma_single_data_mode_init(dma, rx_ch, &p);
  dma_channel_subperipheral_select(dma, rx_ch, subperi);
}

/** @brief  初始化 SPI0 DMA: DMA1 CH5(TX) / CH0(RX), 子外设 DMA_SUBPERI3
 *  @note   RX 原用 CH2, 因 USART0_RX (GNSS1) 需占用 DMA1_CH2 而迁至 CH0 */
void spi0_dma_init(void) { spi_dma_config(DMA1, DMA_CH5, DMA_CH0, DMA_SUBPERI3, SPI0); }

/** @brief  初始化 SPI1 DMA: DMA0 CH4(TX) / CH3(RX), 子外设 DMA_SUBPERI0 */
void spi1_dma_init(void) { spi_dma_config(DMA0, DMA_CH4, DMA_CH3, DMA_SUBPERI0, SPI1); }

/**
 * @brief  SPI DMA 同步收发底层实现
 *
 * 完整流程:
 *   1. 将 tx 数据填入共享缓冲区
 *   2. 配置 DMA 传输长度
 *   3. 使能 SPI 的 DMA TX/RX 请求
 *   4. 使能 DMA 通道，CPU 轮询 DMA FTF 标志等待完成
 *   5. 再轮询 SPI TRANS 标志确保最后一字节完全移出移位寄存器
 *   6. 将接收数据从共享缓冲区拷贝到 rx
 *   7. 清理: 关闭 DMA 通道、SPI DMA 请求、清除所有标志
 *
 * @param  dma      DMA 外设
 * @param  tx_ch    TX DMA 通道
 * @param  rx_ch    RX DMA 通道
 * @param  spi      SPI 外设基地址
 * @param  tx       发送缓冲区
 * @param  rx       接收缓冲区
 * @param  size     收发字节数 (1 ~ SPI_DMA_BUFFER_SIZE)
 * @return 0 成功；-1 参数 / DMA 超时 / SPI 忙超时
 */
static int spi_dma_transfer_bytes(uint32_t dma, dma_channel_enum tx_ch, dma_channel_enum rx_ch,
                                  uint32_t spi, const uint8_t *tx, uint8_t *rx, uint16_t size)
{
  uint32_t timeout = SPI_DMA_TIMEOUT;
  uint16_t i;
  if (tx == NULL || rx == NULL || size == 0U || size > SPI_DMA_BUFFER_SIZE)
    return -1;

  /* 关闭 DMA 通道，清理 FTF 标志 */
  dma_channel_disable(dma, tx_ch);
  dma_channel_disable(dma, rx_ch);
  dma_flag_clear(dma, tx_ch, DMA_FLAG_FTF);
  dma_flag_clear(dma, rx_ch, DMA_FLAG_FTF);

  /* 填充发送缓冲区，清零接收缓冲区 */
  for (i = 0; i < size; ++i)
  {
    spi_dma_tx_buffer[i] = tx[i];
    spi_dma_rx_buffer[i] = 0U;
  }

  /* 重新配置 DMA 内存地址和传输长度 */
  dma_memory_address_config(dma, tx_ch, DMA_MEMORY_0, (uint32_t)spi_dma_tx_buffer);
  dma_memory_address_config(dma, rx_ch, DMA_MEMORY_0, (uint32_t)spi_dma_rx_buffer);
  dma_transfer_number_config(dma, tx_ch, size);
  dma_transfer_number_config(dma, rx_ch, size);

  /* 使能 SPI 的 DMA 请求 & 启动 DMA */
  spi_dma_enable(spi, SPI_DMA_RECEIVE);
  spi_dma_enable(spi, SPI_DMA_TRANSMIT);
  dma_channel_enable(dma, rx_ch);
  dma_channel_enable(dma, tx_ch);

  /* 等待 DMA TX/RX 都完成（FTF 置位） */
  while ((dma_flag_get(dma, tx_ch, DMA_FLAG_FTF) == RESET ||
          dma_flag_get(dma, rx_ch, DMA_FLAG_FTF) == RESET) &&
         timeout-- != 0U)
  {
  }
  if (timeout == 0U)
  {
    /* DMA 超时: 停止 DMA / SPI，清空 FIFO 残留 */
    dma_channel_disable(dma, tx_ch);
    dma_channel_disable(dma, rx_ch);
    spi_dma_disable(spi, SPI_DMA_TRANSMIT);
    spi_dma_disable(spi, SPI_DMA_RECEIVE);
    dma_flag_clear(dma, tx_ch, DMA_FLAG_FTF);
    dma_flag_clear(dma, rx_ch, DMA_FLAG_FTF);
    if (spi_i2s_flag_get(spi, SPI_FLAG_RBNE) != RESET)
      (void)spi_i2s_data_receive(spi);
    return -1;
  }

  /* 等待 SPI 移位寄存器完全移空（TRANS 标志） */
  timeout = SPI_DMA_TIMEOUT;
  while (spi_i2s_flag_get(spi, SPI_FLAG_TRANS) != RESET && timeout-- != 0U)
  {
  }
  if (timeout == 0U)
  {
    dma_channel_disable(dma, tx_ch);
    dma_channel_disable(dma, rx_ch);
    spi_dma_disable(spi, SPI_DMA_TRANSMIT);
    spi_dma_disable(spi, SPI_DMA_RECEIVE);
    dma_flag_clear(dma, tx_ch, DMA_FLAG_FTF);
    dma_flag_clear(dma, rx_ch, DMA_FLAG_FTF);
    return -1;
  }

  /* 拷贝接收数据到调用者缓冲区 */
  for (i = 0; i < size; ++i)
    rx[i] = spi_dma_rx_buffer[i];

  /* 收尾: 关闭 DMA 和 SPI DMA 请求，清除标志 */
  dma_channel_disable(dma, tx_ch);
  dma_channel_disable(dma, rx_ch);
  spi_dma_disable(spi, SPI_DMA_TRANSMIT);
  spi_dma_disable(spi, SPI_DMA_RECEIVE);
  dma_flag_clear(dma, tx_ch, DMA_FLAG_FTF);
  dma_flag_clear(dma, rx_ch, DMA_FLAG_FTF);
  return 0;
}

/** @brief  SPI0 DMA 收发（DMA1 CH5/CH0, SPI0 外设） */
int spi0_dma_transfer(const uint8_t *tx, uint8_t *rx, uint16_t size)
{
  return spi_dma_transfer_bytes(DMA1, DMA_CH5, DMA_CH0, SPI0, tx, rx, size);
}

/** @brief  SPI1 DMA 收发（DMA0 CH4/CH3, SPI1 外设） */
int spi1_dma_transfer(const uint8_t *tx, uint8_t *rx, uint16_t size)
{
  return spi_dma_transfer_bytes(DMA0, DMA_CH4, DMA_CH3, SPI1, tx, rx, size);
}
