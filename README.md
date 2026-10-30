# IMU_Project — GD32F405 组合导航固件

> **v1.5.0** · GD32F405 (Cortex-M4F, 168MHz) 组合导航固件。
> 当前状态：**GNSS1 真机接入 + INS / 15 维 Kalman 融合链路闭环**
> （USART0 230400 8E1 + DMA1_CH2 循环缓冲，解析 T_HJD_POSDATA 帧）；GNSS2 仍为 stub。
>
> 版本变更历史见 [CHANGELOG.md](CHANGELOG.md)；本文件只描述**当前**代码状态。

---

## 目录结构

```
IMU_Project/
├── User/                          应用层 (主循环 + 板级驱动 + 导航流水线)
│   ├── main.c                     主循环 / 初始化 / 致命故障停机 (g_main_fault_code)
│   ├── navi_pipeline.c            导航流水线编排 —— feed_imu + run_kalman + get_state
│   ├── navi_pipeline.h            流水线对外 API
│   ├── version.c / version.h      固件版本号 (v1.5.0) + 上电 printf
│   ├── gnss1_rx.c / gnss1_rx.h    GNSS1 帧接收解析 (USART0 DMA1_CH2 循环 + 环形缓冲)
│   ├── systick.c / systick.h      SysTick 1ms 中断 + 自由运行毫秒计数 g_sys_tick_ms
│   ├── XV7021.c / XV7021.h        三轴陀螺 SPI 驱动
│   ├── ADXL357.c / ADXL357.h      三轴加计 SPI 驱动 (DMA)
│   ├── IMU_FRAME.c                机体系→导航系坐标映射规则
│   ├── gd32f4xx_it.c               中断向量 (含 DMA1_Channel2_IRQHandler 环缓冲计圈)
│   └── ...
├── Peripheral/                    外设封装层
│   ├── UART/uart.c / uart.h       USART0 (230400 8E1): TX DMA1_CH7 发导航结果帧
│   ├── SPI/spi.c / spi.h          SPI0 (XV7021) DMA1 CH5/CH0, SPI1 (ADXL357) DMA0 CH4/CH3
│   └── Timer/timer.c / timer.h    TIMER3 5ms 采样定时器 (ISR 只置 imu_sample_due)
├── Navi/                          组合导航算法包
│   ├── navi_api.c / navi_api.h    navi_init / navi_run_one_frame (黑盒路径, 当前不使用)
│   ├── navi_config/               常量/类型/滤波器参数
│   │   ├── navi_constants.h       PI / arcdeg (deg→rad) / 地球参数
│   │   ├── navi_types.h           Imu_Infor / Sys_Infor / AHRS_Infor / GNSS_Infor / Ethi / Kalman15d_SSMat
│   │   └── navi_filter_params.h   Kalman 噪声参数 (RTK/SPP)
│   ├── navi_core/                 核心算法
│   │   ├── navi_ins.c             ins_update_RT 双子样惯导递推
│   │   ├── navi_attitude.c        a2mat (欧拉角→姿态阵) / m2att (姿态阵→欧拉角) / vect3d_assign
│   │   ├── navi_coarse_align.c    CoarseAlign_Horizon_RT 水平粗对准 (30s)
│   │   └── navi_earth.c           earth() 地球参数计算
│   ├── navi_compensation/         误差补偿
│   │   ├── navi_error_comp.c      ErrorComp×2 级 (温补 + 标定)
│   │   ├── navi_error_comp.h
│   │   ├── GetFlag_GNSS_type.c    GetGNSSType (GNSStype 位判定 + memcpy vn/pos)
│   │   │                          + IsGNSSDataValid (PDOP ∈ (0,2.1) 门控 → sys.flag_GNSSvalid)
│   ├── navi_math/                 矩阵/向量数学库
│   │   ├── navi_matrix.c          mat_multi + 6 个 float 原生内核 (mat_multi_f / mat_plus_f / ...)
│   │   ├── mat_inverse.c          高斯消元求逆 (float, static aug[722], 零 heap)
│   │   └── navi_vector.c
│   └── navi_kalman/               卡尔曼滤波器 (当前只用 15d)
│       ├── kalman15d/
│       │   ├── kalman15d.c        GetKalman15d_SSMat / TimeUpdate / MesUpdate / Feedback
│       │   ├── Kalman15d_SSMat.h  15 维状态结构体 (SSMat 成员为 float)
│       │   └── ...
│       ├── navi_kalman7d.c        7d 精对准 (当前不使用)
│       └── navi_kalman19d.c       19d 组合导航 (当前不使用)
├── Library/                       GD32F4xx CMSIS / StdPeriph
├── startup/                       startup_gd32f405.s (Stack 0x7000, Heap 0x0000)
├── project/template.uvprojx       Keil MDK-ARM 工程 (ARMCC V5, -O3 -Otime)
├── CHANGELOG.md                   版本变更记录 (工程唯一历史来源)
└── README.md                      当前状态说明 (本文件)
```

---

## 主循环骨架

```c
for (;;) {
    /* ① GNSS1 数据泵
     *    从 USART0 DMA1_CH2 环形缓冲取字节 → 状态机解析 T_HJD_POSDATA 帧 */
    gnss1_rx_pump();

    /* ② 采样事件
     *   TIMER3 ISR 只置 imu_sample_due=1, 主循环消费 */
    if (imu_sample_due == 1U) {
        imu_sample_due = 0U;
        SPI 采样 XV7021 (gyro dps) + ADXL357 (accel g)
        温度读取 (gyro_temp + accel_temp)
        机体系 → 导航系坐标映射
        navi_pipeline_feed_imu(accel_mps2, gyro_radps, gyro_temp_c, accel_temp_c);
        navi_push_div++;
    }

    /* ③ Kalman 链 (flag_navsuc 触发; 与采样分支同层级, 不嵌在 if 内) */
    navi_pipeline_run_kalman();

    /* ④ 导航上报 (navi_push_div >= NAVI_PUSH_DIV)
     *   NAVI_PUSH_DIV=10 → 200Hz/10 = 20Hz 上报, 在 Kalman 之后发以保证融合值 */
    if (navi_push_div >= 10U) {
        navi_push_div = 0U;
        navi_pipeline_get_state(att_nav, vn_nav, pos_nav);
        nav_send_frame(att_nav, vn_nav, pos_nav);
    }
}
```

> 初始化阶段任一传感器初始化失败 → `main_fault_halt(code)`：记录 `g_main_fault_code`
> 后关闭总中断并死循环停机（裸机下 `main()` 返回属未定义行为）。

---

## 导航算法数据流

```
每 5ms 一帧 (200Hz):
  navi_pipeline_feed_imu
         ├─ ① GNSS 数据填充
         │    GNSS1: gnss1_rx_get_result() 有新帧才覆盖 vn1/pos1/flag_GNSS1valid/PDOP_1/sat_num1
         │            (无新帧保持上一帧值)
         │    GNSS2: stub 固定值 (后续接第二路卫导)
         ├─ ② gyro_radps → dps × arcdeg × ts → gyro_ori (rad/帧)
         │          accel_mps2 × ts → acc_ori (m/s·帧)
         ├─ ③ T_gyro[0..2] / T_acc[0..2] 写入
         ├─ ④ GetErrorPara_TC → errpara_TC[30] (温补多项式, pow 已改乘法)
         ├─ ⑤ ErrorComp(errpara_TC, gyro_ori/acc_ori → gyro_TC/acc_TC, ts)  ← 温补级
         │    ErrorComp(para_calerr, gyro_TC/acc_TC → data_gyro/data_acc, ts) ← 标定级
         ├─ ⑥ GetGNSSType(GNSStype=0x0C & 0x08=0x08 → RTK1 分支)
         │    memcpy gnss_info.vn1 → sys.vn, pos1 → sys.pos, PDOP = PDOP_1 (实测帧)
         ├─ ⑦ IsGNSSDataValid(PDOP∈(0,2.1) && flag_GNSSvalid=1 → sys.flag_GNSSvalid=1)
         │    无效分支: cnt_NoGNSS++, 超 GNSSINVALIDTIME(2000)×fs → sys.flag_GNSSfault=1
         ├─ ⑧ 粗对准状态机 (3 步):
         │    step1: calign_started==0 && flag_ValidIMU && flag_Finish_calign==0 → 启动
         │    step2: CoarseAlign_Horizon_RT 每帧累计 (30s × 200Hz = 6000 帧)
         │            + heading=(90-180°)*arcdeg = -π/2 (ENU yaw = -90° = 朝南)
         │    step3: count≥6000 && flag_EnvFactor==1 → 拷贝 att_align 到 g_imu.att
         │            a2mat → cnb, 装订 GNSS vn/pos → flag_Finish_calign=1 (锁存, 永不重进)
         └─ ⑨ flag_HorAlignFinish==1 → ins_update_RT 双子样 (kk 交替, 每 10ms flag_navsuc=1)

while(1) 外层与 feed_imu 同层级:
  navi_pipeline_run_kalman()
    ├─ flag_navsuc==1 → kalman15d_update (flag_navsuc=0 + SSMat + TimeUpdate)
    ├─ sys.flag_GNSSvalid==1 → Kalman15d_MesUpdate
    │    量测残差 zk = [imu.vn - gnss.vn, imu.pos - gnss.pos] (6×1)
    │    RTK 量测噪声 rk = diag(0.05², 0.05², 0.05², 0.01², 0.01², 0.02²)
    │    Hk(6×15) 对应 15d 状态 [dvn_E/N/U, dlat/dlon/dh, ...]
    └─ Kalman_Feedback → 把 KF 估计的 δvn/δpos 写回 g_imu

上报:
  navi_pipeline_get_state(att, vn, pos)
    att 由 m2att(g_imu.cnb) 实时反解 (粗对准未完成时回退 g_imu.att)
```

---

## GNSS1 真机接入 (T_HJD_POSDATA 协议)

USART0 → PA9 TX (导航结果帧) / PA10 RX (GNSS1)，**230400, 8E1 (9bit + 偶校验)**

| 项目 | 值 |
| --- | --- |
| 同步字 | 线上顺序 **0x55 → 0xBB** |
| u8FrameLen | **51** (整帧总字节数, 含同步字/数据区/校验) |
| 校验 | 索引 0~49 的**累加和低 8 位**，与索引 50 比较 |
| 数据字节序 | **小端** (实测帧验证: height `36 0C 00 00` → 312.6m) |
| 经纬度单位 | **1e-6 度** |
| 高度 / 速度单位 | 0.1 m / 0.1 m/s |
| PDOP 单位 | 0.01 |
| 定位有效 | `u8PosVld == 2` → `flag_GNSS1valid = 1` |
| RX 通道 | **DMA1_CH2 / SUBPERI4**, 循环模式, **2048 字节环形缓冲** (≈40 帧 ≈ 87.2ms) |
| TX 通道 | DMA1_CH7 (uart.c, 发导航结果帧) |

关键字段偏移 (帧内字节索引)：

| 偏移 | 字段 | 类型 |
| --- | --- | --- |
| 4 | u8PosVld | u8 |
| 9 | u8TrcSatNum | u8 |
| 19 | s32lon (1e-6°) | s32 |
| 23 | s32lat (1e-6°) | s32 |
| 27 | s32height (0.1m) | s32 |
| 31 / 33 / 35 | s16vn / s16vu / s16ve (0.1m/s) | s16 |
| 37 | u16PDOP (0.01) | u16 |

> **环缓冲建模**：写入位置 = `DMA1_CH2` 圈数（FTF 中断累加 `s_dma_write_laps`）× BUF_SIZE
> + 当前传输剩余量推算；读取侧维护累计读指针，两者之差即可用字节。整圈被覆盖时
> `g_gnss_ovf_count++` 并记录 `g_gnss_ovf_stamp_ms[]` 时刻，随后从最新 2048 字节重新同步。
>
> **GNSS2** 仍为 stub：0.0/0.0/0.0 速度，34.2°N / 108.8°E / 400m，PDOP 1.5，sat 12；
> 单天线协议无航向字段，粗对准航向固定 `GNSS1_DEFAULT_HEADING_DEG = 180°`。
> `GNSStype` 固定 **0x0C** (bit0x08=RTK1)，Kalman 数据通路 (`GetGNSSType → IsGNSSDataValid → MesUpdate`) 未改动。

---

## GNSS 无效判定与故障超时

`IsGNSSDataValid()` 两层门控：

1. **质量门**：`PDOP ∈ (0, PDOP_MAX=2.1)` 且 `flag_GNSSvalid == 1` → `sys.flag_GNSSvalid = 1`，并清零 `gnss.flag_GNSSvalid`（一次性消费，防止 Kalman 重复吃同一帧）
2. **故障超时**：连续无效计数 `cnt_NoGNSS > GNSSINVALIDTIME(2000) × fs(200)` → `sys.flag_GNSSfault = 1`

> 注意：超时只覆盖"**接收机仍在输出但质量差/未定位**"的场景。
> 接收机**彻底停发**时 `flag_GNSS1valid` / `PDOP_1` 保持上次好值，`cnt_NoGNSS` 不增长，不会触发 fault。

---

## 全局变量命名

导航状态全部定义在 `navi_pipeline.c` 内并保持 `static`（外部只能走 `navi_pipeline_*` API）：

| 变量 | 用途 |
| --- | --- |
| `static struct Imu_Infor g_imu` | 惯导数据 (陀螺/加速度/温度/姿态/速度/位置) |
| `static struct Ethi g_ethi` | 地球参数 |
| `static struct Sys_Infor g_sys` | 系统标志位 |
| `static struct AHRS_Infor g_ahrs` | 姿态/对准状态 |
| `static struct GNSS_Infor gnss_info` | GNSS1/GNSS2 接收数据 |
| `static struct Kalman15d_SSMat g_k15s` | 15 维滤波器状态 |
| `static struct Imu_TempCompPara imu_temp_para` | 温补参数 |
| `static uint32_t calign_frame_count` | 粗对准累计帧数 |
| `static uint8_t calign_started` | 粗对准已启动标志 |

> 这些 `static` 变量需断点落在 `navi_pipeline.c` 内才能加入 Keil Watch；
> 需要任意断点可读的诊断量（GNSS 计数、故障码、毫秒时基）均为非 static 全局，见下文。

---

## para_calerr + imu_temp_para 装订

```c
/* navi_pipeline_init() 在 navi_init 之后重新装订 */
memset(&imu_temp_para, 0, ...);
imu_temp_para.aT_E1x[0..2/3] = 1.0;   /* a0=1, a1..a4=0 → 温度多项式退化为常数 */
imu_temp_para.aT_K1x[0..2/3] = 1.0;

memset(&g_imu.para_calerr, 0, ...);
para_calerr[9, 10, 11] = 1.0;         /* 陀螺 E1g 标度恒等 */
para_calerr[24, 25, 26] = 1.0;        /* 加表 K1a 标度恒等 */
```

---

## 内存占用 (v1.5.0 实测, Keil build 报告)

| 区域 | 大小 | 说明 |
| --- | --- | --- |
| Code | 38,232 B | Navi 算法 + gnss1_rx + math 库（含 `mat_multi_f*` float 内核与 `mat_inverse` float 高斯消元，-O3 循环展开） |
| RO-data | 1,088 B | 常量表 |
| RW-data | 180 B | 已初始化变量 (含 `g_main_fault_code` 等) |
| ZI-data | 42,012 B | 零初始化 (Stack 28,672 + Kalman15d 静态矩阵 + 2048 B GNSS 环缓冲) |
| **Flash 合计** | **≈ 38.6KB / 480KB** | Code + RO + RW = 39,500 B |
| **SRAM 合计** | **≈ 41.2KB / 128KB** | RW + ZI = 42,192 B |
| Heap_Size | 0 | startup 0x0000 (零 malloc) |

---

## 性能事实 (v1.3.0 / v1.4.0 实测, 168MHz)

- **Cortex-M4F (FPv4-SP)** 只有单精度 FPU，double 运算靠软件模拟（一次 double MAC ≈226 周期）；
  Kalman 全链路已 float 化（SSMat 成员、`mat_multi_f*` 内核、`mat_inverse` 均 float）。
- **峰值**：-O3 + `__restrict` 后主循环单轮峰值 **4.682 ms**（< 5 ms 采样节拍，余量 ≈0.318 ms）；
  峰值轮构成为「Kalman 时间更新 1.650 ms + 量测更新 1.606 ms + 采样段 1.373 ms」三者同轮。
  v1.4.0 把 `mat_inverse` float 化后量测更新再降到 **1.066 ms**（−34%），预期峰值约 4.1 ms。
- **采样节拍**：5 ms = 840,000 周期；`imu_sample_due` 是标志位而非计数器，单轮若超过 5 ms 会吞节拍。
  连续运行实测 `g_loop_over5ms_count = 0`（该计数器已在 v1.4.0 清理时移除，需复测时临时加回）。
- **环缓冲**：2048 B ≈ 87.2 ms 线上数据，远大于单轮耗时，正常运行时 `g_gnss_ovf_count` 恒为 0。
  此前观察到的非零丢帧/覆盖计数，均由 Keil 断点停机期间 DMA 继续灌满环缓冲造成（调试假象）。
- **栈**：`Stack_Size = 0x7000`（28,672 B）。float 化后 `Kalman15d_MesUpdate` 栈数组约 11 KB
  （原 double 版 22,040 B，0x2000 会溢出并压坏 `.bss`）。
- 更早的逐项耗时表（-O0/-O2/float32 三步对比）见 [CHANGELOG.md](CHANGELOG.md) 的 v1.2.0 / v1.3.0 小节。

---

## 静止稳态预期 (Rebuild 烧录后 Watch 面板验证)

| 变量 | 粗对准 0~30s | INS+Kalman 30s+ |
| --- | --- | --- |
| `g_sys.flag_Finish_calign` | 0 | **1 (永不回 0)** |
| `g_ahrs.Flag_HorAlignFinish` | 0 | **1** |
| `g_imu.flag_navsuc` | 0 | **每 10ms 交替 0↔1** |
| `g_sys.flag_GNSSvalid` | 0 | **随 GNSS 帧到达置 1**，MesUpdate 用后清 0 (IsGNSSDataValid 门控) |
| `g_imu.att[0]` (pitch) | 积累中 | **≈ 0** |
| `g_imu.att[1]` (roll) | 积累中 | **≈ 0** |
| `g_imu.att[2]` (yaw ENU) | 积累中 | **≈ -π/2** (-90° = heading 180°真北 → 朝南) |
| `g_imu.vn[0..2]` | 0 | **≈ 0** |
| `g_imu.pos[0]` (lat rad) | 45°N (navi_init 默认) | **→ 实测 GNSS 纬度 (度 × arcdeg)** |
| `g_imu.pos[1]` (lon rad) | 108°E | **→ 实测 GNSS 经度 (度 × arcdeg)** |
| `g_imu.pos[2]` (height m) | 0 | **→ 实测 GNSS 高度** |

> 上报姿态（`navi_pipeline_get_state`）由 `m2att(g_imu.cnb)` 实时反解，故运动过程中
> 也会随 `cnb` 刷新；`g_imu.att` 仍只在粗对准完成时写一次，作为未对准完成时的回退值。

---

## 可观测性变量 (Watch 面板)

| 变量 | 含义 | 判读 |
| --- | --- | --- |
| `g_main_fault_code` (main.c, 非 static) | 初始化故障码 | 0=正常；1=ADXL357 初始化失败；2=XV7021 初始化失败（出现即已停机） |
| `g_sys_tick_ms` (systick.c) | SysTick 自由运行毫秒计数 | 1ms 递增，约 49.7 天回绕；为溢出快照提供时间基准 |
| `imu_sample_due` (timer.c, volatile) | TIMER3 5ms 采样节拍标志 | 置 1 后由主循环消费清 0 |
| `g_gnss_frame_count` (gnss1_rx.c) | 累计校验通过的帧数 | 持续递增 = 接收正常 |
| `g_gnss_err_count` (gnss1_rx.c) | 校验失败帧数 | 应保持 0；递增 = 有字节但帧错位 |
| `g_gnss_ovf_count` (gnss1_rx.c) | DMA 环缓冲整圈覆盖次数 | 应保持 0；非 0 = 主循环停顿 >87.2ms（或断点停机） |
| `g_gnss_ovf_stamp_ms[8]` / `g_gnss_ovf_stamp_idx` (gnss1_rx.c) | 最近 8 次覆盖发生的毫秒时刻 (环形) | 与 `g_sys_tick_ms` 对照，区分"调试停机"与"真实卡顿" |
| `g_gnss_result` (gnss1_rx.c) | 最新解析结果 (lat/lon/height/vn/PDOP/sat) | 与实测坐标对照 |
| `g_imu` / `g_sys` / `g_ahrs` / `g_k15s` (navi_pipeline.c, static) | 惯导 / 标志位 / 姿态 / 滤波器状态 | 断点须落在 `navi_pipeline.c` 内才可 Watch |

> `g_gnss_*` 刻意不加 `static`，因此**在任意断点**（包括停在 main.c 时）都能直接加入 Watch。
> 反之 `parse_frame()` 内的 `lat_deg` / `pdop` 等是**栈上局部变量**，必须把断点设进该函数内部才能观察。

> **断点假象**：停在断点上时 CPU 停止但 USART + DMA 仍在收数据，环缓冲会持续回绕；
> 要拿真实计数必须**复位后不设断点连续跑 30s，再暂停一次读取**。

---

## 构建

Keil MDK-ARM (ARMCC V5), 优化等级 **Level 3 (-O3 -Otime)** (`<Optim>4</Optim> <oTime>1</oTime>`):

```
Open project/template.uvprojx → Rebuild All
Expected: 0 error, 0 warning
Flash size: ~38.6KB / 480KB  (Code 38,232 + RO 1,088 + RW 180 = 39,500 B)
SRAM size:  ~41.2KB / 128KB  (RW 180 + ZI 42,012 = 42,192 B，ZI 含 Stack 28,672)
Stack      : Stack_Size 0x7000 (startup_gd32f405.s)
```

> 改 `.uvprojx` 的优化等级不会触发全量重编，需先删 `Objects/*.o *.d *.crf *.axf` 再 Rebuild。
> 优化等级与命令行的对应关系：`<Optim>3</Optim>` → `-O2`，`<Optim>4</Optim>` → `-O3`。

---

## 已知未闭环问题

| 问题 | 现状 | 结论 / 待办 |
| --- | --- | --- |
| 静止时 `vn` 缓慢增长 | 已观测（-O2 / -O3 均存在，非优化引入） | `Kalman_Feedback` 只反馈 `xk[3..8]`（速度/位置），失准角 `xk[0..2]` 与零偏 `xk[9..14]` **不回馈**。姿态矩阵的航向误差不被修正，重力经 `cnb` 投影出水平分量持续注入 `vn` |
| 姿态角输出不刷新 | **v1.5.0 已修复** | 上报姿态改由 `m2att(g_imu.cnb, att_rad)` 实时反解；`g_imu.att` 仅在粗对准完成时写一次，现仅作未对准完成的回退值 |
| 粗对准航向固定 180° | 沿用 | T_HJD_POSDATA 无航向字段；接双天线 GNSS 或外部航向源后替换 `GNSS1_DEFAULT_HEADING_DEG` |
| `MAIN_FAULT_NAVI_INIT` 无调用点 | 已定义未使用 | 可删除，或在 `navi_pipeline_init()` 增加失败返回后接入停机 |
