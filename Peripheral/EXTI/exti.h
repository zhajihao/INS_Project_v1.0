#ifndef __EXTI_H
#define __EXTI_H

#include "sys.h"

/**
 * @brief  ADXL357/359 DRDY 中断标志（EXTI12 上升沿触发）
 *
 * 由 EXTI10_15_IRQHandler 在检测到 PC12 上升沿后置 1；
 * 主循环读到后可触发一次完整的 ADXL357 数据读取流程。
 *
 * @note 名称沿用历史 (ADXL359), 本板实际贴装 ADXL357。
 *       当前固件未接入该中断: Exti_Init() 无调用者, 采样由 TIMER3 5ms 节拍驱动
 *       (见 Peripheral/Timer/timer.c 的 imu_sample_due), 此模块为预留通路。
 */
extern uint8_t ADXL359_DRDY_FLAG;

void Exti_Init(void);

#endif
