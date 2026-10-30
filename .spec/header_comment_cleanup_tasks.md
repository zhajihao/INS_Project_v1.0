# Tasks: 头文件函数声明注释清理

按 spec.md 执行，分 4 批：

## Batch 1: User/ 高优先级 Doxygen 文件
- [x] User/navi_bridge.h — 删 6 个 Doxygen 块
- [x] User/version.h — 4 个
- [x] User/mmc5983ma.h — 11 个
- [x] User/gnss_hjd.h — 5 个
- [x] User/IMU_FRAME.h — 2 个
- [x] User/XV7021.h — 14 个 (已被前次操作清理)
- [x] User/ADXL357.h — 5 个 (已被前次操作清理)

## Batch 2: User/ 简单注释文件
- [x] User/main.h — 1 个 `/* */`
- [x] User/systick.h — 3 个 `/* */`
- [x] User/gd32f4xx_it.h — 10 个 `/* */`

## Batch 3: Navi/
- [x] Navi/navi_kalman/kalman15d/kalman15d.h — 5 个 `/* */`
- [x] Navi/navi_config/GNSS_Infor.h — 1 个
- [x] Navi/navi_compensation/navi_error_comp.h — 3 个
- [x] Navi/navi_compensation/navi_time_sync.h — 1 个
- [x] Navi/navi_compensation/navi_leverarm.h — 2 个
- [x] Navi/navi_api.h — 2 个 Doxygen 风格

## Batch 4: Peripheral/
- [x] Peripheral/SPI/spi.h — 4 个
- [x] Peripheral/UART/uart.h — 3 个
- [x] Peripheral/SYS/sys.h — 4 个
- [x] Peripheral/I2C/i2c.h — 1 个
- [x] Peripheral/EXTI/exti.h — 1 个
- [x] Peripheral/Timer/timer.h — 1 个

## 验证
- [x] Grep `@brief` 复核每个改动文件 — 残留全部为文件头/结构体/枚举/extern，按规则保留
- [x] Keil rebuild 0 Error 0 Warning
- [x] Code/RO/RW/ZI 与基线一致：39,272 / 1,088 / 252 / 58,212 ✓ 完全匹配
