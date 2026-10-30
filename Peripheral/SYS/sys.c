#include "sys.h"

/**
 * @brief 执行 WFI 指令 (Wait For Interrupt)，让 Cortex-M4 进入低功耗待机
 *
 * WFI 指令让 CPU 停止执行直到发生中断事件，期间 CPU 时钟停止，
 * 但外设时钟仍可运行（需配置 PMU / RCC 进一步降低功耗）。
 */
__asm void WFI_SET(void)
{
    WFI;
}

/**
 * @brief 关总中断 (CPSID I)
 *
 * 关的是普通可屏蔽中断，Fault 和 NMI 不受影响。
 * 关中断期间 CPU 不响应任何普通中断请求。
 */
__asm void INTX_DISABLE(void)
{
    CPSID   I
    BX      LR
}

/**
 * @brief 开总中断 (CPSIE I)
 */
__asm void INTX_ENABLE(void)
{
    CPSIE   I
    BX      LR
}

/**
 * @brief 写入主堆栈指针 MSP
 * @param  addr  新的 MSP 值 (栈顶地址)
 *
 * 注意: 此函数通常只在特权模式下调用。
 *       在 RTOS 场景中用于从一个任务切换到另一个任务时恢复其栈顶。
 */
__asm void MSR_MSP(unsigned int addr)
{
    MSR MSP, r0 			/* set Main Stack value */
    BX r14
}
