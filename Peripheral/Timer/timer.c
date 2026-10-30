#include "timer.h"

/* TIMER3 时钟来自 APB1 = 84 MHz */
#define TIMER3_CLOCK_HZ 84000000U
/* 预分频后计数器时钟 = 1 MHz (1 us tick) */
#define TIMER3_COUNTER_HZ 1000000U
/* 16 位定时器最大周期 ≈ 65 ms (65536 us) */
#define TIMER3_MAX_PERIOD_MS 65U

/**
 * @brief  IMU 采样周期到期标志
 *
 * 由 TIMER3_IRQHandler 在溢出中断中置 1；
 * main 主循环检测到为 1 时立即执行一次完整的 IMU 采样流程
 * (SPI0 陀螺 + SPI1 加计 + navi_pipeline_feed_imu).
 * volatile 保证 ISR 写、主循环读的跨上下文可见性 (ARMCC -O3 安全).
 */
volatile uint8_t imu_sample_due = 0U;

/**
 * @brief  初始化 TIMER3 为周期定时器
 *
 * 预分频: 84 MHz → 1 MHz (timer_config.prescaler = 83)
 * 周期: period_ms × 1000 - 1 (计数到该值触发 overflow 中断)
 * 中断优先级: NVIC 抢占 4，子优先级 0
 *
 * @param  period_ms  中断周期 (1 ~ 65 ms)
 */
void TIMER3_Init(uint16_t period_ms)
{
    timer_parameter_struct timer_config;

    if (period_ms == 0U || period_ms > TIMER3_MAX_PERIOD_MS)
        return;

    rcu_periph_clock_enable(RCU_TIMER3);
    timer_deinit(TIMER3);
    timer_struct_para_init(&timer_config);
    timer_config.prescaler = (TIMER3_CLOCK_HZ / TIMER3_COUNTER_HZ) - 1U;
    timer_config.alignedmode = TIMER_COUNTER_EDGE;
    timer_config.counterdirection = TIMER_COUNTER_UP;
    timer_config.period = (period_ms * (TIMER3_COUNTER_HZ / 1000U)) - 1U;
    timer_config.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER3, &timer_config);
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER3, TIMER_INT_UP);
    nvic_irq_enable(TIMER3_IRQn, 4U, 0U);
    timer_enable(TIMER3);
}

/**
 * @brief  TIMER3 溢出中断服务函数
 *
 * 每 period_ms 触发一次，置位 imu_sample_due 通知主循环
 * 执行 IMU 数据采集。
 */
void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)
    {
        imu_sample_due = 1U;
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
    }
}
