#ifndef __EXTI_H
#define __EXTI_H

#include "sys.h"

/**
 * @brief  ADXL359 DRDY 中断标志（EXTI12 上拉沿触发）
 *
 * 由 EXTI10_15_IRQHandler 在检测到 PC12 上升沿后置 1；
 * 主循环读到后可触发一次完整的 ADXL359 数据读取流程。
 */
extern uint8_t ADXL359_DRDY_FLAG;

void Exti_Init(void);

#endif
