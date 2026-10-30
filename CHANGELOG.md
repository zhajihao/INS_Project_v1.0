
# 更新记录 (CHANGELOG)

本文件是本工程**唯一**的版本变更记录来源：

- `User/version.h` 只保存版本号宏（`FW_VERSION_MAJOR/MINOR/PATCH`）与编译时间宏；
- `README.md` 只描述**当前**状态与用法，不再保存历史；
- 逐条提交的细节用 `git log` / `git show <commit>` 查证，与本文档互补。

## 维护约定

1. 每次发版：修改 `User/version.h` 的三个版本宏 → 在本文件顶部新增一节 → 打标签 `git tag -a vX.Y.Z -m "..."` → `git push --tags`。
2. 一节内容按「主题 → 关键改动 → 编译尺寸 → 实测数据」组织，只写**结论与可复现的数字**，不复述代码。
3. 文件更名、接口更名要在当版明确写出新旧名字，避免历史条目里的旧名被误当成现役模块。

## 版本与标签对照

| 版本             | 日期               | 主题                                             |
| ---------------- | ------------------ | ------------------------------------------------ |
| **v1.5.0** | 2026-10-30         | 健壮性与可观测性收尾                             |
| v1.4.0           | 2026-10-28         | 清理全工程打点代码，合并导航算法最终优化成果     |
| v1.3.0           | 2026-10-27         | Kalman 矩阵内层优化 + 优化等级升到 Level 3 (-O3) |
| v1.2.0           | 2026-10-27         | USART0 改为 GNSS1 真机接收口，Kalman float32 化  |
| v1.1.0           | 2026-10-23         | 移植复旦微 A7 组合导航算法包 (Navi/)             |
| v1.0.7           | 2026-09-23         | 初始化失败返回区分故障码                         |
| v1.0.6           | 2026-09-23         | UART0 接收 + 版本查询协议                        |
| v1.0.5           | 2026-09-23         | SPI/串口 DMA 化，5ms 采样节拍确立                |
| 初始搭建         | 2026-09-14 ~ 09-22 | 外部晶振、轴映射、地磁驱动                       |

---

## [v1.5.0] - 2026-10-30 — 健壮性与可观测性收尾

### 健壮性

- **初始化失败不再从 `main()` 返回**：裸机下 `main` 返回属未定义行为（向量表无调用者，会跳飞或复位）。改为 `main_fault_halt()` 记录 `g_main_fault_code` 后关总中断死循环停机；故障码非 `static`，可直接在 Keil Watch 查看。
- **USART0 配置归属收敛**：波特率/字长/校验/DENT 统一由 `UART0_DMA_Init` 配置（230400 8E1），`gnss1_rx_init` 只追加 RX DMA 通道（DMA1_CH2）+ DENR + 环缓冲复位，不再调用 `usart_deinit`，消除重复配置与调用顺序依赖。
- **修正串口校验位配置**：原 `USART_WL_8BIT` + `USART_PM_ODD`（实为 8O1，注释误写 8N1）改为 `USART_WL_9BIT` + `USART_PM_EVEN`，与 GNSS1 的 8E1 真正匹配。

### 正确性

- **上报姿态改为实时反解**：`navi_pipeline_get_state()` 由直读 `g_imu.att` 改为 `m2att(g_imu.cnb, att_rad)`。原实现的问题：15 维 Kalman 链路递推的是 `cnb`，`g_imu.att` 仅在粗对准完成时写一次，导致上报姿态停留在对准时刻。粗对准未完成时仍回退 `g_imu.att`。
- **环缓冲溢出时刻快照**：新增 `g_gnss_ovf_stamp_ms[8]` + `g_gnss_ovf_stamp_idx`（由 `systick.c` 的自由运行毫秒计数 `g_sys_tick_ms` 提供时标），用于事后区分「调试器 halt 造成」与「主循环真实卡顿」。pump 主路径行为不变。

### 可观测性

- **`g_sys_tick_ms`**：SysTick 1ms 中断递增的自由运行毫秒计数（约 49.7 天回绕），为无阻塞时间戳提供基准。

### 清理

- 删除死 API：`navi_pipeline_get_phase` / `navi_pipeline_phase_str` / `navi_phase_t`（`navi_pipeline.h` 现仅保留 `init` / `feed_imu` / `run_kalman` / `get_state`）。
- 删除 `s_gnss2_stub` 中未使用的 `heading_deg` 字段。
- 删除未使用的 19 维 Kalman 静态矩阵 `g_k19` / `g_k19s`（ZI 约 −14.3KB）。

### 更名（本版起生效，历史条目中的旧名不再使用）

| 旧名                      | 新名                        | 说明                                                                                                                                                                                                 |
| ------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `User/gnss_hjd.c/.h`    | `User/gnss1_rx.c/.h`      | 驱动+协议解析；结果类型 `gnss_hjd_result_t` → `gnss1_rx_result_t`，API `gnss_hjd_get_result` → `gnss1_rx_get_result`                                                                       |
| `User/navi_bridge.c/.h` | `User/navi_pipeline.c/.h` | 导航处理链封装；API `navi_bridge_feed_imu` / `navi_bridge_try_run_kalman` / `navi_bridge_get_state` → `navi_pipeline_feed_imu` / `navi_pipeline_run_kalman` / `navi_pipeline_get_state` |

- GNSS RX 环缓冲 1024 → **2048 字节**（覆盖窗口 48.9ms → **87.2ms**，约 40 帧）。
- 删除宽泛的死接口 `gnss1_rx_has_data()`。

### 编译尺寸 (ARMCC V5 -O3 -Otime)

```
Code 38,224 → 38,232 B (+8)
RO 1,088 B (不变)
RW 164 → 180 B (+16, 故障码 + 溢出快照)
ZI 41,980 → 42,012 B (+32)
0 Error 0 Warning
```

---

## [v1.4.0] - 2026-10-28 — 清理打点代码，合并导航算法最终优化

### 删除 DWT 耗时测量基础设施

- 彻底删除 `CoreDebug->DEMCR` / `DWT->CTRL` / `DWT->CYCCNT` 相关代码：
  - `main.c`：`g_loop_cycles_last` / `_max` / `g_loop_slow_count` / `g_loop_over5ms_count`、`LOOP_SLOW_THRESHOLD` / `OVER5MS_THRESHOLD`、`g_sample_count` / `g_spi_acc_max` / `g_spi_gyro_max` / `g_temp_max` / `g_nav_feed_max` / `g_sample_total_max` / `g_send_max`，以及 `main()` 内全部 `t_a~t_f` / `prev_loop_cycles` 局部变量与打点块；
  - `navi_pipeline.c`（原 `navi_bridge.c`）：`g_kf_run_count` / `g_kf_update_max` / `g_kf_mes_max` / `g_kf_fb_max`、`g_feed_pre_max` / `g_feed_tc_max` / `g_feed_ec_max` / `g_feed_chk_max` / `g_feed_ins_max`，以及 `imu_frame_process_all` 内 `t0~t5` / `try_run_kalman` 内 `t0~t3` 统计块。
- **保留的诊断变量**（`gnss1_rx.c/.h`，`volatile` 全局，供 Watch 观察，且被业务逻辑直接引用，不是打点）：`g_gnss_frame_count`、`g_gnss_err_count`、`g_gnss_ovf_count`，以及 `s_dma_write_laps`（pump 计算 `write_total` 的核心，DMA1_CH2 FTF ISR 写）。

### mat_inverse float 化

- 静态数组 `double aug[722]` → `float aug[722]`（ZI −2,888 B）；函数签名 `double* A, double* A_inv` → `float*`，内部临时量全 float、`fabs` → `fabsf`、`1e-10` → `1e-10f`。
- 高斯消元由软件浮点 `__aeabi_dmul/__aeabi_dadd` 提速为 Cortex-M4F 硬件 FPU `vmla.f32`。
- `Kalman15d_MeaUpdate.c` 删除 `inv_in6` / `inv_in7` 桥接循环（每步 72~98 次 float↔double 隐式转换），直接调用 `mat_inverse(6, …)` / `mat_inverse(7, …)`。
- 实测：MeaUpdate 1.606 → **1.066ms**（−34%）；TimeUpdate 1.697ms 持平（本就全 float）。

### DMA1_CH2 FTF 中断计圈

- 原来在 pump 里靠 `transfer_number` 回装瞬间轮询 FTF，存在竞态（临界帧可能读到不稳定值）；改为 `DMA1_Channel2_IRQHandler` 每圈写满精确触发，ISR 仅清 FTF + `++s_dma_write_laps`（48.9ms 才触发一次，零负担）。
- pump 删除 FTF 轮询块，init 末尾使能 `DMA1_Channel2_IRQn` + `DMA_CHXCTL_FTFIE`；`s_dma_write_laps` 由 `static` 改 `volatile`；`gd32f4xx_it.c` 新增该 ISR。

### 其他清理

- 删除 `g_gnss_drop_count`（单槽邮箱覆盖属正常行为，消费者只取最新帧，非零不等于故障）及其 getter 与 extern 声明。
- 编译尺寸：mat_inverse float 化 Code 39,272 → 38,544（−728 B）；FTF 计圈 +56 B；全删 DWT 打点 Code 38,648 → 38,264、RW 244 → 164。

### 烤机验证（17.5min / 30min 两轮，168MHz）

- 全链路零吞节拍、零慢轮次；`g_gnss_ovf_count = 0`、`g_gnss_err_count = 2`（0.011%，原 536/0.56%）。
- 业务行为零改动：主循环骨架、SPI/DMA/UART、GNSS 解析与融合时序均未变。

---

## [v1.3.0] - 2026-10-27 — Kalman 矩阵内层优化 + Level 3 (-O3)

- `navi_math` 6 个 float 内核（`mat_multi_f` / `mat_plus_f` / `mat_minus_f` / `mat_assign_f` / `mat_transpose_f` / `mat_unit_f`）形参加 `const` + `__restrict`，声明与定义同步；告知编译器 a/b/c 互不重叠，消除别名检查与冗余重载（工程内无同址调用，零行为影响）。
- Keil 优化等级 Level 2 → Level 3（`<Optim>4</Optim> <oTime>1</oTime>`，命令行 `-O3 -Otime`），仅改优化选项与指针限定符。
- **-O3 实测**（168MHz，÷168000 得 ms）：主循环峰值 906,421 → **786,661 周期**（5.39 → 4.682ms，−13.2%）；Kalman 时间更新 335,695 → 277,125（2.00 → 1.650ms）；量测更新 329,243 → 269,862（1.96 → 1.606ms）。首次进入 5ms 预算内，余量 53,339 周期（0.318ms）。
- 反汇编确认 `mat_multi_f` 内层由约 11 条指令/MAC 降至约 4.5 条：k 循环 2 路展开、指针走查替代每次重算 `i*n+k`、相邻两次 MAC 共用同一 S 寄存器（不逐次 `VSTR`）。
- 尺寸：Code 34,512 → 39,256 B（+4,744，来自循环展开），RO 1,088 / RW 244 / ZI 58,212 均不变。
- 数值语义：ARMCC V5 的 `-O3` 不改变浮点语义（`--fpmode` 默认 `std`，严格 IEEE 754，不做重结合），与 `-O2` 的差异只在指令调度层。代价是局部变量常被优化进寄存器，Keil Watch 出现 `<cannot evaluate>`，单步体验下降。
- 烧录验证（2026-10-28）：复位连跑后吞节拍计数与慢轮次计数均为 0，实证 5ms 采样节拍零吞帧；主循环峰值 4.654ms 稳定 <5ms。
- 已识别风险（本版未改代码）：`xv7021_sample_due`（`Peripheral/Timer/timer.c`）为普通 `uint8_t`、无 `volatile`，却被 TIMER3 ISR 写、主循环读；`-O3` 理论上可把该读取提到循环外导致采样冻结。实测未触发（跨编译单元 + 主循环调用大量外部函数），**建议后续加 `volatile` 加固**。

---

## [v1.2.0] - 2026-10-27 — GNSS1 真机接入 + Kalman float32 化

### USART0 改为 GNSS1 专用接收口

- 新增 `User/gnss_hjd.c/.h`（v1.5.0 起更名 `gnss1_rx.c/.h`），解析 `T_HJD_POSDATA` 帧：同步字 `0x55 0xBB`、整帧 51 字节、累加和低 8 位校验（索引 0~49）。
- 字段为**小端**：经纬度单位 1e-6 度，高度/速度 0.1 单位。原文档写的「大端 + 1e-7」与实测帧不符，已纠正。
- RX 走 DMA1_CH2 循环模式 + 应用层环形缓冲，主循环逐字节解析；SPI0_RX 由 DMA1_CH2 迁至 DMA1_CH0（SUBPERI3）避让。
- 删除 UART0 接收环形缓冲、RBNE 中断与版本查询协议 `version_protocol_poll()`。
- 环缓冲 256 → 1024 字节（12.2ms → 48.9ms 覆盖窗口），并改为「DMA 圈数 + 累计读指针」建模，可检测回绕覆盖（`g_gnss_ovf_count`），不再喂入已被覆盖的旧字节。

### 致命修复：栈溢出

- `Stack_Size` 0x2000 → **0x7000**：15 维 Kalman 的 `MesUpdate` 栈需求 22,040 B 远超原 8,192 B，溢出约 14KB 压坏 `.bss`（`s_dma_buf` / `g_gnss_result` / `aug` / `imu_temp_para`），表现为粗对准一结束 GNSS 结果即被清零、求逆结果不可信。

### 性能优化（分三步）

1. **优化等级 Level 0 → Level 2 (-O2)**；删除 `GetKalman15d_SSMat.c` 中 4 个从未引用的局部数组（`mat_temp10/11/12`、`v_temp3`）。
2. **乘法下沉 float（`mat_multi_f32`）**：对外仍是 double 签名，内部 float 累加，3×225 float 静态暂存（2,700 B，零 heap）；反汇编确认内层为 `VLDR` + `VMLA.F32`。实测 Kalman 两步 11.13/11.13 → **2.44/2.66ms**，主循环峰值 24.97 → **7.79ms**（−68.8%）。
3. **全 float 化（方案 B）**：`Kalman15d_SSMat` 8 个成员 → `float`（ZI −4,224 B）；`navi_math` 新增 6 个 float 原生内核并**删除** `mat_multi_f32` 及其 2,700 B 暂存区；`TimeUpdate` 9 处、`MeaUpdate` 19 处改 `_f` 调用；`mat_inverse` 仍 double，用 `inv_in/out` 桥接（每步 72 次转换，原约 2,300 次）。实测两步 → **2.00 / 1.96ms**，峰值 6.53 → **5.39ms**；`Kalman15d_MesUpdate` 栈数组 21.5KB → 约 11.2KB。

- 消除 `GetErrorPara_TC` 中 18 处 `pow(x,2/3/4)`（改乘法），实测该项 1.353ms 中 `pow` 约占 1.25ms。
- 尺寸：Code 34,008 → 34,280（float32 化）→ 34,064（消 pow）→ 34,512（全 float）；ZI 62,436 → 65,140 → **58,212**。
- 精度权衡：由 double（≈15 位有效）降为 float（≈7 位有效），回退方式为把调用改回 `mat_multi`（无需改结构体）。

---

## [v1.1.0] - 2026-10-23 — 移植 A7 组合导航算法包

- 新增 `Navi/`（43 个文件：21 `.c` / 22 `.h`）与 `User/navi_bridge.c/.h`（v1.5.0 起更名 `navi_pipeline.c/.h`），逐行复刻 A7 `FM_QL_lwip/main.c` 处理链：`imu_frame_process_all` + `ErrorComp×2` + 粗对准状态机 + `ins_update_RT`。
- 15 维 Kalman 全链路：`GetKalman15d_SSMat` + `TimeUpdate` + `MesUpdate` + `Feedback`。
- GNSS1/GNSS2 先用 stub 模拟（34.2°N / 108.8°E / 400m，heading1=180°、heading2=45°）。
- 变量命名对齐 A7：`g_imu` / `g_sys` / `g_ahrs` / `gnss_info` / `g_k15s`。
- 角度/弧度转换统一走 `navi_constants.h` 常量，零字面量 `PI`。
- 双子样 5ms×2 = 10ms 一次完整 INS + Kalman 周期；粗对准只在上电跑一次（`flag_Finish_calign` 锁存）。
- `mat_inverse.c` 的 `malloc` 改静态数组（零 heap）；`startup` Stack 0x400 → 0x2000、Heap 0x400 → 0x0000。
- 坐标系映射：gyro `navi_x=+gyro[1]`、`navi_y=-gyro[2]`、`navi_z=-gyro[0]`；accel `navi_x=-adxl.y_g`、`navi_y=-adxl.x_g`、`navi_z=-adxl.z_g`。

---

## [v1.0.7] - 2026-09-23

- 传感器初始化失败时返回区分加速度计/陀螺仪的故障码（本版是 `main()` 直接 `return`，v1.5.0 改为停机）。

## [v1.0.6] - 2026-09-23

- 增加 UART0 接收、版本查询命令解析及二进制版本应答（协议 `55 AA 01 01 02 0D 0A`，已于 v1.2.0 随 USART0 改作 GNSS1 接收而移除）。

## [v1.0.5] - 2026-09-23

- ADXL357 的 SPI1 数据读写改为 DMA 传输；UART0 增加 DMA 发送，波特率 230400。
- 加速度计、三路陀螺及温度数据按 5ms 周期采集。
- 串口帧陀螺仪角速度单位 °/s、加速度单位 m/s²。
- 精简未使用的 SPI / UART / 定时器接口，统一相关变量命名。

## 初始搭建 - 2026-09-14 ~ 2026-09-22

- 2026-09-14：改用外部 16 MHz 晶振。
- 2026-09-17：修正陀螺仪和加速度计的轴映射与方向。
- 2026-09-22：添加 MMC5983MA 地磁传感器驱动（当前采集与输出暂不使用）。
