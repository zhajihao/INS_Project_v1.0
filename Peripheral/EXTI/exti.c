/**
 ******************************************************************************
 * @file         exti.c
 * @author       Guoxx
 * @version      V1.0
 * @date         2025-07-25
 * @brief        外部中断驱动
 ******************************************************************************
 */
#include "systick.h"
#include "exti.h"

/** @brief ADXL357/359 DRDY 中断标志（EXTI12 上升沿触发后置 1, 名称沿用历史） */
uint8_t ADXL359_DRDY_FLAG = 0;

/**
 * @brief  外部中断初始化
 *
 * 中断源配置:
 *   - PC12 → EXTI_12  上升沿触发 (ADXL357 DRDY)
 *   - PD2  → EXTI_2   上升沿触发 (预留)
 *
 * 中断优先级: 抢占 3，子优先级 0。
 *
 * @note 本固件当前未调用本函数: 采样由 TIMER3 5ms 节拍驱动, 此处为预留通路。
 */
void Exti_Init(void)
{
    /* 使能 GPIO 时钟 */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* 配置为输入 (下拉，检测上升沿) */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_12);
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_2);

    /* 使能 NVIC 中断 */
    nvic_irq_enable(EXTI10_15_IRQn, 3U, 0U);
    nvic_irq_enable(EXTI2_IRQn, 3U, 0U);

    /* 连接 EXTI line 到对应 GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN12);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOD, EXTI_SOURCE_PIN2);

    /* 配置 EXTI line 触发方式: 上升沿中断 */
    exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_RISING);

    /* 使能 EXTI 中断 & 清标志 */
    exti_interrupt_enable(EXTI_12);
    exti_interrupt_enable(EXTI_2);
    exti_interrupt_flag_clear(EXTI_12);
    exti_interrupt_flag_clear(EXTI_2);
}

/**
 * @brief  EXTI10~15 中断服务函数（PC12 = EXTI12）
 *
 * 检测 PC12 为高电平时置位 ADXL359_DRDY_FLAG (标志名沿用历史, 本板为 ADXL357)。
 * 读完后立即清 EXTI 标志防止同一沿再次进入中断。
 */
void EXTI10_15_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_12))
    {
        if (gpio_input_bit_get(GPIOC, GPIO_PIN_12))
        {
            ADXL359_DRDY_FLAG = 1;
        }
        exti_interrupt_flag_clear(EXTI_12);
    }
}

/**
 * @brief  EXTI2 中断服务函数 (PD2 = EXTI2，当前预留未使用)
 */
void EXTI2_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_2))
    {
        if (gpio_input_bit_get(GPIOD, GPIO_PIN_2))
        {
            /* TODO: PD2 对应的外部中断处理 */
        }
        exti_interrupt_flag_clear(EXTI_2);
    }
}
