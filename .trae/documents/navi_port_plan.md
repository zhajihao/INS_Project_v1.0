# 组合导航算法迁移 IMU_Project 实施计划

## Repository Research

### 当前 IMU_Project 现状
- **MCU**: GD32F405 (Cortex-M4F @ 168 MHz, 单精度 FPU 仅)
- **Flash 总量**: 480 kB / 当前已用 9.46 kB (1.97%)
- **SRAM 总量**: 128 kB / 当前已用 ~1.44 kB RAM + 1 kB Stack / 0 Heap (自动裁剪)
- **采样**: 5 ms 周期 (TIMER3), 168 MHz AHB, SPI0/1 5.25 MHz
- **硬件输入**: ADXL357 (SPI1 加速度) + 3× XV7021BB (SPI0 陀螺仪) + UART0 (230400, DMA TX + 中断 RX)
- **主循环**: 5 ms 触发 → SPI 采样 → 换算 → 打包 → DMA 发帧
- **无 GNSS**: 当前只有 UART0 用于发 IMU 帧，无额外 GNSS 串口

### navi 算法源 (system_platform_backup_20261023_postPushTrim/navi/)
- **21 个 .c / 22 个 .h**, 约 4500 行代码
- **全部用 double** (64-bit), Cortex-M4F 单精度 FPU 无法加速, 所有 double 靠软件模拟
- **关键文件**:
  - navi_types.h: 153 处 double 声明, 核心结构体 Imu_Infor / Ethi / Sys_Infor / Kalman7d/12d/19d_State
  - navi_api.c/h: 对外统一接口 navi_init() + navi_run_one_frame()
  - navi_kalman19d.c (609 行): 19×19 EKF 主解算, 初始化 P/Q/R/S, 构造 F/H/M 矩阵
  - navi_kalman7d.c (303 行): 7×7 精对准 EKF
  - navi_ins.c / navi_coarse_align.c / navi_earth.c / navi_attitude.c: INS 解算 + 对准 + 地球参数
  - navi_math/navi_matrix.c / mat_inverse.c / navi_vector.c: 矩阵/向量/求逆 (**mat_inverse.c 用了 malloc**)
  - navi_compensation/*: 误差补偿/时间同步/杆臂补偿 (ErrorComp_Cali 在 navi_api.c 中被注释掉了)

### 外部依赖分析
- **math.h**: 19 个 .c 文件 include <math.h>, 使用 sin/cos/sqrt/asin/acos/atan2/pow (全是 double 版)
- **malloc/free**: 仅 mat_inverse.c 的高斯-约旦求逆用 `aug = malloc(n*2*n*sizeof(double))`, 大小 n×2n 临时矩阵
- **printf/fopen**: navi_api.c 原注释中提到"移除 PC 端 fopen/printf", 实际代码无此类调用
- **POSIX/文件 IO**: 无

### 关键风险点

| 风险 | 严重性 | 说明 |
|---|---|---|
| **double 算力** | 🔴 高 | Cortex-M4F 单精度 FPU, 所有 double 运算靠软件模拟。200Hz + 19×19 EKF 全量迭代肯定超 5ms 帧预算 |
| **mat_inverse malloc** | 🟡 中 | GD32 startup 中 HEAP=0 被自动裁剪, mat_inverse 调用 malloc 会 HardFault |
| **GNSS 输入** | 🟡 中 | 当前板无 GNSS 串口, 19d EKF 量测更新依赖位置/速度观测, 必须 stub |
| **RAM 预算** | 🟡 中 | 19×19 double P 矩阵 ≈ 2.9 KB, 加上全部 Kalman 状态 + Imu_Infor/Ethi/Sys_Infor 结构体 + 深调用栈, 总 RAM 估算 35-45 KB (含 8 KB Stack), 占 128 KB 的 ~35% |
| **Flash 预算** | 🟢 低 | 算法代码 + math.h double 软件库估算 70-100 KB, 占 480 KB 的 ~20% |

### 用户决策
| 项 | 用户选择 |
|---|---|
| GNSS | 用模拟值先跑, 后续接真接收机 |
| 精度策略 | 保留 double, 降采样率 |
| 移植范围 | 全量移植 (navi_core + navi_kalman7d/12d/19d + kalman15d + navi_math + navi_compensation + navi_api + navi_config), GNSS stub |

### 降采样率方案
- navi_init 默认 fs=200Hz, nn=2 (双子样), nts=0.01s (导航 100Hz)
- **方案**: 保持 IMU 物理采样 ts=5ms 不变 (硬件定时), 改为 nn=4 (四子样), nts=0.02s → **导航 50Hz, 帧预算 20ms** (宽松 4×)
- 好处: (1) 20ms 内可以跑完 19d EKF 全量迭代 + 求逆 (2) 四子样旋转矢量对陀螺误差拟合更好, INS 精度不降反升
- 代价: 导航输出频率从 100Hz 降到 50Hz, 但对 IMU 板级调试够用

## Files and Modules

### 新增文件 (全部 navi 算法)

从 `FDW_Board/software/fmql_A7_eth/SDK/system_platform_backup_20261023_postPushTrim/navi/` 拷贝到 IMU_Project 根目录新建的 `Navi/` 目录 (保持子目录结构):

| 源路径 | 目标路径 | 文件数 |
|---|---|---|
| navi/navi_api.c, navi_api.h, constants.h | Navi/ | 3 |
| navi/navi_config/* | Navi/navi_config/ | 4 (navi_types.h, navi_constants.h, navi_filter_params.h, GNSS_Infor.h) |
| navi/navi_math/* | Navi/navi_math/ | 6 (navi_vector.c/h, navi_matrix.c/h, mat_inverse.c, math_matrix.h) |
| navi/navi_core/* | Navi/navi_core/ | 10 (navi_ins.c/h, navi_earth.c/h, navi_attitude.c/h, navi_ahrs.c/h, navi_coarse_align.c/h) |
| navi/navi_kalman/* | Navi/navi_kalman/ | 6 (navi_kalman7d.c/h, navi_kalman12d.c/h, navi_kalman19d.c/h) |
| navi/navi_kalman/kalman15d/* | Navi/navi_kalman/kalman15d/ | 7 |
| navi/navi_compensation/* | Navi/navi_compensation/ | 7 |
| **合计** | | **43** |

### 需修改的文件

| 文件 | 修改内容 |
|---|---|
| `Navi/navi_math/mat_inverse.c` | 移除 malloc/free, 改为调用方传入的静态/栈缓冲区 |
| `Navi/navi_api.c` | navi_init 中: 采样参数改为 fs=200, nn=4, nts=0.02, ts=0.005 |
| `User/main.c` | (1) 初始化 navi (2) 主循环中缓存 4 帧 IMU 子样 (3) 每 20ms 调用 navi_run_one_frame (4) 导航状态打包发送 (5) GNSS 模拟注入 |
| `User/main.h` | 新增 navi 相关 include |
| `User/IMU_FRAME.c/h` | 扩展帧格式支持导航输出 (att/vn/pos) 或新增独立导航帧 |
| `startup_gd32f405.s` | 增加 STACK_Size 从 0x400 (1KB) 到 0x2000 (8KB) |
| `project/template.uvprojx` | Keil 工程新增 Navi/ 目录及其全部 .c 文件到编译列表, 新增 Navi/ 子目录到 Include Path |

### 新增转换层文件 (桥接当前 IMU 输出 → navi 输入)

| 文件 | 职责 |
|---|---|
| `User/navi_bridge.c/h` | (1) 将当前 IMU 采样 (dps, m/s²) 转换为 navi 要求的 gyro_ori (rad/帧) 和 acc_ori (m/s·帧) (2) 累积 4 个子样后触发 navi_run_one_frame (3) 提供导航输出 (att/vn/pos) 到 UART 发送的接口 |

## Implementation Steps

1. **创建 Navi 目录结构并拷贝全部 43 个 navi 源文件** (从 FDW_Board 根到 IMU_Project/Navi/)

2. **修复 mat_inverse.c 的 malloc/free 问题**
   - 将 `double* aug = malloc(n*2*n*sizeof(double))` 改为调用方传入参数, 或改为 static 全局数组 (最大支持 19×38 = 722 个 double ≈ 5.8 KB, 适合放 BSS)
   - 移除所有 free() 调用

3. **调整 navi_init() 采样参数** (navi_api.c)
   - `nn = 4` (四子样)
   - `nts = 0.02` (导航 50Hz, 20ms 周期)
   - `ts = 0.005` (IMU 原始采样 200Hz, 保持硬件一致)
   - fs=200 不变

4. **扩展 startup 栈配置**
   - startup_gd32f405.s 中 Stack_Size 从 0x400 改 0x2000 (8KB, 深矩阵运算调用链需要)

5. **创建 navi_bridge 转换层** (User/navi_bridge.c/h)
   - 对外 API: `navi_bridge_init()`, `navi_bridge_feed_imu(accel_mps2[3], gyro_radps[3])`, `navi_bridge_get_state(att, vn, pos)`
   - 内部: 维护 Imu_Infor/Ethi/Sys_Infor/Kalman7d_State/Kalman19d_State 等所有导航状态结构体 (static 全局)
   - 内部: 维护 4 帧 IMU 子样缓存 `imu_subsample_buf[4][3]` (gyro) 和 `imu_subsample_buf_acc[4][3]` (acc)
   - 内部: 当 4 帧子样填满时, 累加得到 `gyro_ori[3] = Σ gyro_buf[i]` (rad/帧), `acc_ori[3] = Σ acc_buf[i] × ts` (m/s·帧), 然后调用 navi_run_one_frame
   - GNSS stub: 初始化时写入模拟 GNSS 观测值 `sys->vn_GNSS[0]=0, pos_GNSS[lat, lon, h]`

6. **改造 main.c**
   - 初始化段: `navi_bridge_init()` (在传感器初始化之后, 主循环之前)
   - 主循环: 每 5ms 采样后, 调用 `navi_bridge_feed_imu(accel_mps2, gyro_radps)` —— 由 bridge 内部管理累积和触发时机
   - 移除原有的直接 `imu_send_frame()` 调用 (改为从 navi_bridge 读导航状态打包发送)
   - 保留 version_protocol_poll() 调用不变

7. **扩展 UART 输出帧格式**
   - 方案 A (推荐): 保留原 IMU 原始帧, 每 20ms 额外发一帧导航结果 (att/vn/pos), 帧头 0x55AA 不同 ID
   - 方案 B: 单一帧同时包含 IMU 原始数据 + 导航结果 (帧较长, 64 字节可能不够)
   - 选方案 A, 扩展 UART0_DMA_TX_BUFFER_SIZE 到 128 字节

8. **修改 Keil template.uvprojx**
   - 新增 Navi/navi_api.c + Navi/navi_core/*.c + Navi/navi_math/*.c + Navi/navi_kalman/*.c + Navi/navi_kalman/kalman15d/*.c + Navi/navi_compensation/*.c 到编译组
   - Include Path 新增 `.\Navi`, `.\Navi\navi_config`, `.\Navi\navi_core`, `.\Navi\navi_math`, `.\Navi\navi_kalman`, `.\Navi\navi_kalman\kalman15d`, `.\Navi\navi_compensation`
   - 注意: template.uvprojx 是 XML 格式, 手工改容易出错, 建议先在 Keil IDE 里 GUI 添加后重新打开

9. **编译验证 (ARMCC)**
   - 必须有 .uvprojx 配合 IDE 或命令行 UV4.exe 才能编译; 无法在当前环境直接验证
   - 预期: 第一次编译可能报 include 路径缺失 / math.h 找不到 / malloc 隐式声明等, 逐个修
   - 预期 Flash 增量 70-100 KB, RAM 增量 30-40 KB

10. **链接后 map 检查**
    - 确认 Total ROM 不超 480 kB (预期 ~80 KB, 远低于上限)
    - 确认 Total RW+ZI 不超 128 kB (预期 ~35 KB, 含 8KB Stack)
    - 确认 Stack_Size 生效, 无 HEAP 残留

## Dependencies and Considerations

- **navi 源中的数学函数**: Keil ARMCC 自带 math.h 的 double 版 (sin/cos/sqrt/pow), 编译时自动链接软件模拟库, 约 20-30 KB Flash
- **navi_compensation/navi_error_comp.c**: navi_api.c 中 `ErrorComp_Cali(imu)` 调用被注释, 意味着误差补偿/温度补偿阶段目前被跳过, para_calerr 只做了最简单的标度因数=1 初始化 (等价于不补偿)
- **gnss2.c / gnss.c (源 User/)**: 原 FDW_Board 里有 GNSS 解析代码, 后续接真接收机时可以复用
- **常量 arcdeg**: navi_types.h 里用 `arcdeg = π/180` 把角度转弧度, 原工程用 45 表示 45°
- **双子样 vs 四子样**: navi_coarse_align.c / navi_ins.c 中的旋转矢量算法需要核对是否支持 nn=4
- **navi_ins.c 中的中值积分**: 如果 INS 递推内部有 "当前帧角速度/2" 的假设 (双子样中常用), 需要调整为四子样版本

## Validation

| 序号 | 验证项 | 方法 |
|---|---|---|
| 1 | navi_bridge 调用 navi_run_one_frame 正确触发 | 在 bridge 内加计数/标志, 观察是否每 4 帧触发一次 |
| 2 | navi_init 成功完成, 无 HardFault/Debug Monitor | 在 navi_init 后加 GPIO toggle (或 printf 打印), 板子跑起来看 LED |
| 3 | mat_inverse 不再调用 malloc, 无 HEAP 残留 | 检查 map 文件中 HEAP section 是否仍为 0 |
| 4 | 导航状态 att/vn/pos 能正常输出 (哪怕是模拟值) | UART0 用调试助手接, 看导航帧数据是否连续变化 |
| 5 | 原 IMU 采样流程不中断, 5ms 周期稳定 | DWT 或 GPIO 翻转 + 示波器看 TIMER3 中断间隔 |
| 6 | Total ROM / RAM 不超 MCU 容量上限 | map 文件末尾 Total ROM / Total RW Size 行 |
| 7 | 19×19 矩阵求逆 + EKF 迭代能在 20ms 内跑完 | DWT 测 navi_run_one_frame 单次调用周期数 |

## Risks

| 风险 | 影响 | 处理 |
|---|---|---|
| **19d EKF 单帧超 20ms** | 导航帧还没算完下一帧又到, 丢帧 + 输出抖动 | 临时关闭 19d, 只跑粗对准 + 7d; 或进一步降 nn 到 2 (100Hz IMU 但精度好); 或只跑 INS 纯解算跳过 Kalman |
| **mat_inverse 改法引入数值 bug** | 求逆结果错误 → P 矩阵不正定 → EKF 发散 | 把 aug 改 static [19*2][19] (BSS), 不改变算法逻辑; 或在调用方直接传入栈上数组 |
| **navi_ins.c 不支持 nn=4** | 四子样旋转矢量积分公式不匹配 → INS 姿态错误 | 先跑 nn=2, ts=0.005, nts=0.01 (100Hz, 预算 10ms); 确认 INS 正常后再改 nn=4 |
| **RAM 预算爆掉 (Stack 深调用)** | HardFault | 逐步增加 Stack_Size: 先 4KB, 跑起来看栈溢出不, 不行再 8KB |
| **math.h 的 double sin/cos/sqrt 精度 / 速度** | 地球参数计算慢 → 拖 INS 后腿 | 保留 double; 如太慢可把 navi_earth.c 中高频调用的 sin/cos 提前算好存查表 |
| **GNSS stub 导致 19d EKF 无法量测更新** | k19->flag_NavIntegInit 永远不触发, 停在精对准阶段 | navi_api.c 中 nav_integ 函数需要 flag_GNSSvalid=1 才做量测更新, stub 时把 sys->flag_GNSSvalid 置 1 |
