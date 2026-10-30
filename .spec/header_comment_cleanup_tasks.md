# Tasks: 头文件函数声明注释清理

按 spec.md 执行，分 4 批：

## Batch 1: User/ 高优先级 Doxygen 文件

- [X] User/navi_pipeline.h — 删 6 个 Doxygen 块
- [X] User/version.h — 4 个
- [X] User/mmc5983ma.h — 11 个
- [X] User/gnss1_rx.h — 5 个
- [X] User/IMU_FRAME.h — 2 个
- [X] User/XV7021.h — 14 个 (已被前次操作清理)
- [X] User/ADXL357.h — 5 个 (已被前次操作清理)

## Batch 2: User/ 简单注释文件

- [X] User/main.h — 1 个 `/* */`
- [X] User/systick.h — 3 个 `/* */`
- [X] User/gd32f4xx_it.h — 10 个 `/* */`

## Batch 3: Navi/

- [X] Navi/navi_kalman/kalman15d/kalman15d.h — 5 个 `/* */`
- [X] Navi/navi_config/GNSS_Infor.h — 1 个
- [X] Navi/navi_compensation/navi_error_comp.h — 3 个
- [X] Navi/navi_compensation/navi_time_sync.h — 1 个
- [X] Navi/navi_compensation/navi_leverarm.h — 2 个
- [X] Navi/navi_api.h — 2 个 Doxygen 风格

## Batch 4: Peripheral/

- [X] Peripheral/SPI/spi.h — 4 个
- [X] Peripheral/UART/uart.h — 3 个
- [X] Peripheral/SYS/sys.h — 4 个
- [X] Peripheral/I2C/i2c.h — 1 个
- [X] Peripheral/EXTI/exti.h — 1 个
- [X] Peripheral/Timer/timer.h — 1 个

## 验证

- [X] Grep `@brief` 复核每个改动文件 — 残留全部为文件头/结构体/枚举/extern，按规则保留
- [X] Keil rebuild 0 Error 0 Warning
- [X] Code/RO/RW/ZI 与基线一致：39,272 / 1,088 / 252 / 58,212 ✓ 完全匹配
  （v1.3.0 执行时点值；v1.5.1 当前为 38,256 / 1,088 / 180 / 42,012）
