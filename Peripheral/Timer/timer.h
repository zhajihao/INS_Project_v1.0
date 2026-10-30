#ifndef TIMER_H
#define TIMER_H

#include "gd32f4xx.h"

/**
 * @brief  IMU 采样周期到期标志 (TIMER3 中断置 1, 主循环读完后应清零)
 *         volatile 保证 ISR 写 / 主循环读的跨上下文可见性.
 */
extern volatile uint8_t imu_sample_due;

void TIMER3_Init(uint16_t period_ms);

#endif
