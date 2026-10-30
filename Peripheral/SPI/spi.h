#ifndef __SPI_H
#define __SPI_H

#include "sys.h"

/* SPI0 使用 DMA1: TX=CH5, RX=CH0; 子外设映射 DMA_SUBPERI3 */
/* SPI1 使用 DMA0: TX=CH4, RX=CH3; 子外设映射 DMA_SUBPERI0 */

void spi0_dma_init(void);

void spi1_dma_init(void);

int spi0_dma_transfer(const uint8_t *tx, uint8_t *rx, uint16_t size);

int spi1_dma_transfer(const uint8_t *tx, uint8_t *rx, uint16_t size);

#endif
