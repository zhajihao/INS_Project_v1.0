#include "uart.h"
#include "gd32f4xx_dma.h"
#include <stdio.h>
#include <string.h>

/* ============= DMA TX 配置 ============= */
#define UART0_TX_DMA_CHANNEL DMA_CH7
#define UART0_TX_DMA_SUBPERIPHERAL DMA_SUBPERI4

/* 发送 DMA 缓冲区 */
static uint8_t uart0_tx_buffer[UART0_DMA_TX_BUFFER_SIZE];
static uint8_t uart0_tx_busy;
volatile uint32_t uart0_dma_tx_dropped_packets;

/**
 * @brief  初始化 UART0 (GPIO + USART0 230400 8E1 + DMA1 CH7 TX)
 *
 * @note   本函数是 USART0 参数 (波特率/字长/校验/使能) 的唯一配置者.
 *         RX 侧由 gnss1_rx_init() 追加: DMA1_CH2 循环模式 + DENR 使能.
 *         TX DMA 请求 (DENT) 也在本函数内使能, 且 gnss1_rx_init 不再调用
 *         usart_deinit, 因此发送链路不依赖两个初始化函数的调用顺序.
 *
 * @param  baudrate  波特率 (实际使用 230400, 与 GNSS1 接收机一致)
 */
void UART0_DMA_Init(uint32_t baudrate)
{
    dma_single_data_parameter_struct dma_config;

    rcu_periph_clock_enable(UART0_GPIO_CLK);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_DMA1);

    /* GPIO: PA9=TX, PA10=RX, AF7 */
    gpio_af_set(UART0_GPIO_PORT, UART0_PIN_AF, UART0_TX_PIN | UART0_RX_PIN);
    gpio_mode_set(UART0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                  UART0_TX_PIN | UART0_RX_PIN);
    gpio_output_options_set(UART0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            UART0_TX_PIN);

    /* USART0: 230400 8E1 (8 数据位 + 偶校验, 校验位计入字长 → WL_9BIT) */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, baudrate);
    usart_word_length_set(USART0, USART_WL_9BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_EVEN);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* DMA1 CH7: Memory → Periph, 内存自增, 8bit */
    dma_deinit(DMA1, UART0_TX_DMA_CHANNEL);
    dma_single_data_para_struct_init(&dma_config);
    dma_config.direction = DMA_MEMORY_TO_PERIPH;
    dma_config.memory0_addr = (uint32_t)uart0_tx_buffer;
    dma_config.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_config.number = sizeof(uart0_tx_buffer);
    dma_config.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_config.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_config.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_config.priority = DMA_PRIORITY_MEDIUM;
    dma_single_data_mode_init(DMA1, UART0_TX_DMA_CHANNEL, &dma_config);
    dma_channel_subperipheral_select(DMA1, UART0_TX_DMA_CHANNEL, UART0_TX_DMA_SUBPERIPHERAL);
    dma_circulation_disable(DMA1, UART0_TX_DMA_CHANNEL);
    dma_flag_clear(DMA1, UART0_TX_DMA_CHANNEL, DMA_FLAG_FTF);
    uart0_tx_busy = 0U;
    uart0_dma_tx_dropped_packets = 0U;
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
}

uint8_t UART0_DMA_Send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > sizeof(uart0_tx_buffer))
        return 0U;

    if (uart0_tx_busy != 0U &&
        dma_flag_get(DMA1, UART0_TX_DMA_CHANNEL, DMA_FLAG_FTF) == RESET)
    {
        ++uart0_dma_tx_dropped_packets;
        return 0U;
    }

    dma_channel_disable(DMA1, UART0_TX_DMA_CHANNEL);
    dma_flag_clear(DMA1, UART0_TX_DMA_CHANNEL, DMA_FLAG_FTF);
    memcpy(uart0_tx_buffer, data, length);
    dma_memory_address_config(DMA1, UART0_TX_DMA_CHANNEL, DMA_MEMORY_0,
                              (uint32_t)uart0_tx_buffer);
    dma_transfer_number_config(DMA1, UART0_TX_DMA_CHANNEL, length);
    uart0_tx_busy = 1U;
    dma_channel_enable(DMA1, UART0_TX_DMA_CHANNEL);
    return 1U;
}

uint8_t UART0_DMA_IsBusy(void)
{
    return (uint8_t)(uart0_tx_busy != 0U &&
                     dma_flag_get(DMA1, UART0_TX_DMA_CHANNEL, DMA_FLAG_FTF) == RESET);
}

/**
 * @brief  printf 重定向 (polling, 调试用, 不与 DMA TX 同时使用)
 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET)
    {
    }
    usart_data_transmit(USART0, (uint8_t)ch);
    while (usart_flag_get(USART0, USART_FLAG_TC) == RESET)
    {
    }
    return ch;
}
