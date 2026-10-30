#ifndef __SYS_H
#define __SYS_H

#include "gd32f4xx.h"
#include "gd32f4xx_libopt.h"

/* ============= Cortex-M4 位带 (BITBAND) 宏 =============
 *
 * 直接操作 GPIO 寄存器的单个 bit，不需要读-改-写，
 * 原子且高效。详见《ARM Cortex-M3 权威指南》第 87~92 页 (M4 相同)。
 *
 * 计算公式:
 *   bitband = (bitband_region_base) + ((addr & 0xFFFFF) << 5) + (bitnum << 2)
 *
 * GD32F4 位带区域: 0x40000000 映射到 0x42000000
 */

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

/* ============ GPIO ODR / IDR 寄存器地址映射 ============ */
/* ODR: 偏移 +20 */
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) /* 0x40020014 */
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) /* 0x40020414 */
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) /* 0x40020814 */
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) /* 0x40020C14 */
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) /* 0x40021014 */
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) /* 0x40021414 */
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) /* 0x40021814 */
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) /* 0x40021C14 */
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) /* 0x40022014 */

/* IDR: 偏移 +16 */
#define GPIOA_IDR_Addr    (GPIOA_BASE+16) /* 0x40020010 */
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) /* 0x40020410 */
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) /* 0x40020810 */
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) /* 0x40020C10 */
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) /* 0x40021010 */
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) /* 0x40021410 */
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) /* 0x40021810 */
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) /* 0x40021C10 */
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) /* 0x40022010 */

/* ============ 便捷操作宏: 直接读写单个 GPIO pin ============
 *
 * 用法: PAout(5) = 1;   /  if (PAin(5)) { ... }
 * 注意: n 必须小于 16 (16 位寄存器宽度)。
 */
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  /**< PA 口某 pin 输出 */
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  /**< PA 口某 pin 输入 */

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  /**< PB 口某 pin 输出 */
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  /**< PB 口某 pin 输入 */

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  /**< PC 口某 pin 输出 */
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  /**< PC 口某 pin 输入 */

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  /**< PD 口某 pin 输出 */
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  /**< PD 口某 pin 输入 */

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  /**< PE 口某 pin 输出 */
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  /**< PE 口某 pin 输入 */

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  /**< PF 口某 pin 输出 */
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  /**< PF 口某 pin 输入 */

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  /**< PG 口某 pin 输出 */
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  /**< PG 口某 pin 输入 */

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  /**< PH 口某 pin 输出 */
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  /**< PH 口某 pin 输入 */

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  /**< PI 口某 pin 输出 */
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  /**< PI 口某 pin 输入 */

/* ============= Cortex-M 内核级辅助函数 (汇编实现) ============= */

void WFI_SET(void);

void INTX_DISABLE(void);

void INTX_ENABLE(void);

void MSR_MSP(unsigned int addr);

#endif
