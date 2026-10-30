# Spec: 头文件函数声明注释清理

## 目标
按用户指令「头文件里函数不需要注释，只需要在源文件函数定义时注释」，
删除 30 个工程头文件中所有「函数声明」前的注释块（含 `/** @brief ... */` Doxygen 块与 `/* ... */` 简单块）。

## 删除规则（必须严格遵守）
仅删除「函数声明」（返回类型 + 函数名 + `(...)` + `;`）紧邻上方的注释块。
保留以下注释不动：
1. 文件头 `/** @file ... */` 或 `/*! \file ... */` 块（含版权、协议、调用链说明）
2. 宏定义 (`#define`) 行尾或上方注释
3. 枚举值、结构体字段的行内/上方注释
4. 类型定义 (struct/enum/typedef) 上方注释
5. extern 全局变量说明注释
6. `#ifndef / #define / #endif` 块注释、协议格式说明、`#pragma pack` 注释

## 业务行为约束
- 仅删除注释，不动任何代码、宏值、声明顺序、`const`/`__restrict` 关键字
- 编译产物尺寸应与当前基线完全一致（注释不进代码段）

## 涉及文件清单（30 个，按目录分组）

### A. User/ (11 个)
| 文件 | 函数声明注释数 | 类型 |
|---|---|---|
| User/navi_bridge.h | 6 | Doxygen 块 |
| User/version.h | 4 | Doxygen 块 |
| User/mmc5983ma.h | 11 | Doxygen 块 |
| User/gnss_hjd.h | 5 | Doxygen 块 |
| User/IMU_FRAME.h | 2 | Doxygen 块 |
| User/XV7021.h | 14 | Doxygen 块 |
| User/ADXL357.h | 5 | Doxygen 块 |
| User/main.h | 1 | `/* */` 单行 |
| User/systick.h | 3 | `/* */` 单行 |
| User/gd32f4xx_it.h | 10 | `/* */` 单行 |
| User/gd32f4xx_libopt.h | 0 | — |

### B. Navi/navi_math (3 个)
| 文件 | 函数声明注释数 |
|---|---|
| math_matrix.h | 0 |
| navi_vector.h | 0 |
| navi_matrix.h | 0 |

### C. Navi/navi_kalman (5 个)
| 文件 | 函数声明注释数 |
|---|---|
| kalman15d/Kalman15d_SSMat.h | 0 |
| kalman15d/kalman15d.h | 5 (`/* */` 单行) |
| navi_kalman7d.h | 0 |
| navi_kalman19d.h | 0 |
| navi_kalman12d.h | 0 |

### D. Navi/navi_config (4 个)
| 文件 | 函数声明注释数 |
|---|---|
| navi_filter_params.h | 0 |
| GNSS_Infor.h | 1 (`/* GNSS 类型判定 */`) |
| navi_types.h | 0 |
| navi_constants.h | 0 |

### E. Navi/navi_compensation (3 个)
| 文件 | 函数声明注释数 |
|---|---|
| navi_error_comp.h | 3 (`/* */` 单行) |
| navi_time_sync.h | 1 (`/* */` 单行) |
| navi_leverarm.h | 2 (`/* */` 单行) |

### F. Navi/navi_core (5 个)
| 文件 | 函数声明注释数 |
|---|---|
| navi_ins.h | 0 |
| navi_coarse_align.h | 0 |
| navi_ahrs.h | 0 |
| navi_earth.h | 0 |
| navi_attitude.h | 0 |

### G. Navi/ 顶层 (2 个)
| 文件 | 函数声明注释数 |
|---|---|
| navi_api.h | 2 (Doxygen 风格) |
| constants.h | 0 |

### H. Peripheral/ (6 个)
| 文件 | 函数声明注释数 |
|---|---|
| SPI/spi.h | 4 (Doxygen) |
| UART/uart.h | 3 (Doxygen) |
| SYS/sys.h | 4 (Doxygen) |
| I2C/i2c.h | 1 (Doxygen) |
| EXTI/exti.h | 1 (Doxygen) |
| Timer/timer.h | 1 (Doxygen) |

## 风险点
1. `math_matrix.h` 与 `navi_matrix.h` 含 `const float *__restrict` 声明，必须保留关键字原样
2. `Kalman15d_SSMat.h` 中含被注释掉的 `//double rk[6][6];` `//double Hk[6][19];` 字段，属结构体历史注释，保留
3. `navi_filter_params.h` 等仅含宏定义的文件无需改动
4. `gd32f4xx_libopt.h` 无函数声明，无需改动
5. `constants.h` 已废弃重定向，无函数声明

## 验证
1. 逐文件 Edit 后用 Grep `@brief` 抽样复核，确保仅文件头/结构体/宏保留
2. Keil 全量 rebuild：`& "D:\Keil_v5\UV4\UV4.exe" -r "project\template.uvprojx" -j0 -o "build_log.txt"`
3. 检查日志末尾：0 Error(s), 0 Warning(s)
4. Code/RO/RW/ZI 尺寸应与基线 39,272 / 1,088 / 252 / 58,212 完全一致

## 不在本次范围
- Library/CMSIS/.cmsis 等第三方库头文件不动
- 源文件 (.c) 中函数定义处的注释保留不动
- readme.md、版本日志等文档不动
