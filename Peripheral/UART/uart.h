#ifndef UART_H
#define UART_H

#include "sys.h"

/* ============ UART0 硬件资源定义 ============ */

/** @brief UART0 DMA TX 缓冲区大小（字节，DMA 单次发送上限） */
#define UART0_DMA_TX_BUFFER_SIZE 128U
/** @brief UART0 GPIO 时钟 */
#define UART0_GPIO_CLK RCU_GPIOA
/** @brief UART0 GPIO 端口 */
#define UART0_GPIO_PORT GPIOA
/** @brief UART0 TX 引脚 */
#define UART0_TX_PIN GPIO_PIN_9
/** @brief UART0 RX 引脚 */
#define UART0_RX_PIN GPIO_PIN_10
/** @brief UART0 GPIO 复用功能 (USART0) */
#define UART0_PIN_AF GPIO_AF_7

/**
 * @brief UART0 DMA TX 丢包计数
 *
 * 当 DMA 正在传输 (FTF 未置位) 时调用 UART0_DMA_Send 会被拒绝，
 * 同时此计数递增，用于统计带宽竞争导致的丢包。
 */
extern volatile uint32_t uart0_dma_tx_dropped_packets;

/* ============ API ============ */

void UART0_DMA_Init(uint32_t baudrate);

uint8_t UART0_DMA_Send(const uint8_t *data, uint16_t length);

uint8_t UART0_DMA_IsBusy(void);

#endif
