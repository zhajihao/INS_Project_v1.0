# IMU_Project — GD32F405 组合导航固件

> **v1.3.0** · GD32F405 (Cortex-M4F, 168MHz) 组合导航固件。
> 当前状态：**GNSS1 真机接入** (USART0 + DMA1_CH2 循环缓冲，解析 T_HJD_POSDATA 帧)；GNSS2 仍为 stub。

---

## 目录结构

```
IMU_Project/
├── User/                          应用层 (main.c + 板级驱动 + 导航桥接)
│   ├── main.c                     主循环
│   ├── navi_bridge.c              导航桥接层 —— imu_frame_process_all + 粗对准状态机
│   ├── navi_bridge.h              bridge 对外 API
│   ├── version.c / version.h      固件版本号 (v1.3.0) + 上电 printf
│   ├── gnss_hjd.c / gnss_hjd.h    GNSS1 帧接收解析 (USART0 DMA1_CH2 循环 + 环形缓冲)
│   ├── XV7021.c / XV7021.h        三轴陀螺 SPI 驱动
│   ├── ADXL357.c / ADXL357.h      三轴加计 SPI 驱动 (DMA)
│   ├── IMU_FRAME.c                机体系→导航系坐标映射规则
│   └── ...
├── Peripheral/                    外设封装层
│   ├── uart.c / uart.h            UART0 TX DMA1_CH7 (230400, 发导航结果帧)
│   ├── spi.c / spi.h              SPI0 (XV7021) DMA1 CH5/CH0, SPI1 (ADXL357) DMA0 CH4/CH3
│   └── timer.c / timer.h          TIMER3 5ms 采样定时器
├── Navi/                          组合导航算法包
│   ├── navi_api.c / navi_api.h    navi_init / navi_run_one_frame (黑盒路径, 当前不使用)
│   ├── navi_config/               常量/类型/滤波器参数
│   │   ├── navi_constants.h       PI / arcdeg (deg→rad) / 地球参数
│   │   ├── navi_types.h           Imu_Infor / Sys_Infor / AHRS_Infor / GNSS_Infor / Ethi / Kalman15d_SSMat
│   │   └── navi_filter_params.h   Kalman 噪声参数 (RTK/SPP)
│   ├── navi_core/                 核心算法
│   │   ├── navi_ins.c             ins_update_RT 双子样惯导递推
│   │   ├── navi_attitude.c        a2mat (欧拉角→姿态阵) / vect3d_assign
│   │   ├── navi_coarse_align.c    CoarseAlign_Horizon_RT 水平粗对准 (30s)
│   │   └── navi_earth.c           earth() 地球参数计算
│   ├── navi_compensation/         误差补偿
│   │   ├── navi_error_comp.c      ErrorComp×2 级 (温补 + 标定)
│   │   ├── navi_error_comp.h
│   │   ├── GetFlag_GNSS_type.c    GetGNSSType (GNSStype 位判定 + memcpy vn/pos)
│   │   │                          + IsGNSSDataValid (PDOP ∈ (0,2.1) 门控 → sys.flag_GNSSvalid)
│   ├── navi_math/                 矩阵/向量数学库 (double)
│   │   ├── navi_matrix.c          mat_multi / mat_inverse / aug 消元 (static, 零 heap)
│   │   └── navi_vector.c
│   └── navi_kalman/               卡尔曼滤波器 (当前只用 15d)
│       ├── kalman15d/
│       │   ├── kalman15d.c        GetKalman15d_SSMat / TimeUpdate / MesUpdate / Feedback
│       │   ├── Kalman15d_SSMat.h  15 维状态结构体
│       │   └── ...
│       ├── navi_kalman7d.c        7d 精对准 (当前不使用)
│       └── navi_kalman19d.c       19d 组合导航 (当前不使用)
├── Library/                       GD32F4xx CMSIS / StdPeriph
├── startup/                       startup_gd32f405.s (Stack 0x7000, Heap 0x0000)
└── project/template.uvprojx       Keil MDK-ARM 工程 (ARMCC / armclang)
```

---

## 主循环骨架

```c
for (;;) {
    /* ① GNSS1 数据泵
     *    从 USART0 DMA1_CH2 环形缓冲取字节 → 状态机解析 T_HJD_POSDATA 帧 */
    gnss_hjd_pump();

    /* ② 采样事件
     *   Timer3 ISR 只置 xv7021_sample_due=1, 主循环消费 */
    if (xv7021_sample_due == 1U) {
        xv7021_sample_due = 0U;
        SPI 采样 XV7021 (gyro dps) + ADXL357 (accel g)
        温度读取 (gyro_temp + accel_temp)
        机体系 → 导航系坐标映射
        navi_bridge_feed_imu(accel_mps2, gyro_radps, gyro_temp_c, accel_temp_c);
        navi_push_div++;
    }

    /* ③ Kalman 链 (flag_navsuc 触发, while(1) 外层统一跑) */
    navi_bridge_try_run_kalman();

    /* ④ 导航上报 (navi_push_div >= NAVI_PUSH_DIV)
     *   NAVI_PUSH_DIV=10 → 200Hz/10 = 20Hz 上报 */
    if (navi_push_div >= 10U) { navi_push_div = 0U; nav_send_frame(...); }
}
```

---

## 导航算法数据流

```
每 5ms 一帧 (200Hz):
  feed_imu
    └─ imu_frame_process_all
         ├─ ① GNSS 数据填充
         │    GNSS1: gnss_hjd_get_result() 有新帧才覆盖 vn1/pos1/flag_GNSS1valid/PDOP_1/sat_num1
         │            (无新帧保持上一帧值)
         │    GNSS2: stub 固定值 (后续接第二路卫导)
         ├─ ② gyro_radps → dps × arcdeg × ts → gyro_ori (rad/帧)
         │          accel_mps2 × ts → acc_ori (m/s·帧)
         ├─ ③ T_gyro[0..2] / T_acc[0..2] 写入
         ├─ ④ GetErrorPara_TC → errpara_TC[30] (温补多项式)
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
  try_run_kalman()
    ├─ flag_navsuc==1 → kalman15d_update (flag_navsuc=0 + SSMat + TimeUpdate)
    ├─ sys.flag_GNSSvalid==1 → Kalman15d_MesUpdate
    │    量测残差 zk = [imu.vn - gnss.vn, imu.pos - gnss.pos] (6×1)
    │    RTK 量测噪声 rk = diag(0.05², 0.05², 0.05², 0.01², 0.01², 0.02²)
    │    Hk(6×15) 对应 15d 状态 [dvn_E/N/U, dlat/dlon/dh, ...]
    └─ Kalman_Feedback → 把 KF 估计的 δvn/δpos/δatt 写回 g_imu
```

---

## GNSS1 真机接入 (T_HJD_POSDATA 协议, v1.2.0)

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
| RX 通道 | **DMA1_CH2 / SUBPERI4**, 循环模式, **1024 字节环形缓冲** (≈20 帧 ≈ 48.9ms) |
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

> **GNSS2** 仍为 stub：0.0/0.0/0.0 速度，34.2°N / 108.8°E / 400m，heading 45°，PDOP 1.5，sat 12。
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

| 变量 | 用途 |
| --- | --- |
| `static struct Imu_Infor g_imu` | 惯导数据 (陀螺/加速度/温度/姿态/速度/位置) |
| `static struct Ethi g_ethi` | 地球参数 |
| `static struct Sys_Infor g_sys` | 系统标志位 |
| `static struct AHRS_Infor g_ahrs` | 姿态/对准状态 |
| `static struct GNSS_Infor gnss_info` | GNSS1/GNSS2 接收数据 |
| `static struct Kalman15d_SSMat g_k15s` | 15 维滤波器状态 |
| `static struct Kalman19d_State g_k19` | 19 维状态 (未使用) |
| `static struct Kalman19d_Starter g_k19s` | 19 维启动器 (未使用) |
| `static struct Imu_TempCompPara imu_temp_para` | 温补参数 |
| `static uint32_t calign_frame_count` | 粗对准累计帧数 |
| `static uint8_t calign_started` | 粗对准已启动标志 |

---

## para_calerr + imu_temp_para 装订

```c
/* navi_bridge_init() 在 navi_init 之后重新装订 */
memset(&imu_temp_para, 0, ...);
imu_temp_para.aT_E1x[0..2/3] = 1.0;   /* a0=1, a1..a4=0 → 温度多项式退化为常数 */
imu_temp_para.aT_K1x[0..2/3] = 1.0;

memset(&g_imu.para_calerr, 0, ...);
para_calerr[9, 10, 11] = 1.0;         /* 陀螺 E1g 标度恒等 */
para_calerr[24, 25, 26] = 1.0;        /* 加表 K1a 标度恒等 */
```

---

## 内存占用 (v1.3.0 实测, Keil build 报告)

| 区域 | 大小 | 说明 |
| --- | --- | --- |
| Code | 39,272 B | 含 Navi 算法 + gnss_hjd + math 软件库（含 `mat_multi_f*` float 内核，-O3 循环展开） |
| RO-data | 1,088 B | 常量表 |
| RW-data | 252 B | 已初始化变量 |
| ZI-data | 58,212 B | 零初始化 (含 Stack 28KB + Kalman15d 静态矩阵 4.2KB + 1024B GNSS 环缓冲) |
| **Flash 合计** | **≈ 39.7KB / 480KB** | Code + RO + RW = 40,612 B |
| **SRAM 合计** | **≈ 57.1KB / 128KB** | RW + ZI = 58,464 B |
| Heap_Size | 0 | startup 0x0000 (零 malloc) |

---

## 性能事实

- **Cortex-M4F (FPv4-SP)** 只有单精度 FPU，所有 double 运算靠软件模拟（一次 double MAC ≈226 周期）
- `ins_update_RT` 10ms 周期内要跑：双子样补偿 + 15d SSMat + TimeUpdate + MesUpdate (含 mat_inverse) + Feedback
- **实测入口**：`g_loop_cycles_max` / `g_loop_cycles_last` / `g_loop_slow_count` (main.c, DWT CYCCNT)，÷168000 得 ms
- **实测结论**：详见下文「实测耗时分布」——double 版单次迭代峰值 24.97ms；Kalman 全 float 化后 5.39ms；
  **-O3 + `__restrict` 后降到 4.682ms，首次进入 5ms 预算内**（余量 53,339 周期 / 0.318ms）。
  峰值轮构成为「Kalman 时间更新 1.650ms + 量测更新 1.606ms + 采样段 1.373ms」三者同轮。
  已把环缓冲加大到 1024 字节 (48.9ms) 兜住覆盖窗口；实跑零丢帧零覆盖（`g_gnss_drop_count` =
  `g_gnss_ovf_count` = 0），此前非零计数均为调试断点造成的假象。
- **剩余耗时结构**：`mat_multi_f` 内层在 -O3 后约 4.5 条指令/MAC（2 路展开 + 指针走查，累加值常驻
  S 寄存器），但单步折合仍约 32~38 周期/MAC，说明主导项已不在指令数而在
  `GetKalman15d_SSMat` 内的 double `cos()/sin()` 与 15×15 double 数乘（该函数计入
  `g_kf_update_max` 窗口）。进一步提速的方向是把三角函数提出循环并改 `cosf/sinf`。
- **栈**：Kalman 全 float 化之前链接器报告 `Maximum Stack Usage = 22,504 B`（`Kalman15d_MesUpdate` 占 22,040 B，
  因其 21.5KB 双精度临时矩阵全在栈上）；全 float 化后该函数的栈数组降到约 **11.2 KB**
  （含 4 个 `mat_inverse` double 桥接矩阵共 1,360 B）。本项目仍保留 `Stack_Size = 0x7000` (28,672 B) 留足余量；
  原 0x2000 (8,192 B) 会溢出 14KB 并压坏 `.bss`（`s_dma_buf` / `g_gnss_result` / `aug`）。

---

## 静止稳态预期 (Rebuild 烧录后 Watch 面板验证)

| 变量 | 粗对准 0~30s | INS+Kalman 30s+ |
| --- | --- | --- |
| `g_sys.flag_Finish_calign` | 0 | **1 (永不回 0)** |
| `g_ahrs.Flag_HorAlignFinish` | 0 | **1** |
| `calign_frame_count` | 0→6000 | 0 |
| `g_imu.flag_navsuc` | 0 | **每 10ms 交替 0↔1** |
| `g_sys.flag_GNSSvalid` | 0 | **随 GNSS 帧到达置 1**，MesUpdate 用后清 0 (IsGNSSDataValid 门控) |
| `g_imu.att[0]` (pitch) | 积累中 | **≈ 0** |
| `g_imu.att[1]` (roll) | 积累中 | **≈ 0** |
| `g_imu.att[2]` (yaw ENU) | 积累中 | **≈ -π/2** (-90° = heading 180°真北 → 朝南) |
| `g_imu.vn[0]` (东向速度) | 0 | **≈ 0** |
| `g_imu.vn[1]` (北向速度) | 0 | **≈ 0** |
| `g_imu.vn[2]` (天向速度) | 0 | **≈ 0** |
| `g_imu.pos[0]` (lat rad) | 45°N (navi_init 默认) | **→ 实测 GNSS 纬度 (度 × arcdeg)** |
| `g_imu.pos[1]` (lon rad) | 108°E | **→ 实测 GNSS 经度 (度 × arcdeg)** |
| `g_imu.pos[2]` (height m) | 0 | **→ 实测 GNSS 高度** |

---

## 可观测性变量 (Watch 面板)

| 变量 | 含义 | 判读 |
| --- | --- | --- |
| `g_gnss_frame_count` (gnss_hjd.c, 非 static) | 累计校验通过的帧数 | 持续递增 = 接收正常 |
| `g_gnss_err_count` (gnss_hjd.c, 非 static) | 校验失败帧数 | 应保持 0；递增 = 有字节但帧错位 |
| `g_gnss_drop_count` (gnss_hjd.c, 非 static) | **邮箱覆盖丢帧数** | 非 0 说明新帧到达时上一帧未被消费 |
| `g_gnss_ovf_count` (gnss_hjd.c, 非 static) | **DMA 环缓冲整圈覆盖数** | 非 0 说明主循环迭代 > 48.9ms，数据已被 DMA 覆盖 |
| `g_gnss_result` (gnss_hjd.c, 非 static) | 最新解析结果 (lat/lon/height/pdop) | 与实测坐标对照 |
| `g_loop_cycles_last` (main.c) | 最近一次主循环迭代耗时 (CYCCNT) | ÷168000 ≈ ms |
| `g_loop_cycles_max` (main.c) | 上电以来最大迭代耗时 | **应 < 8,215,000** (≈48.9ms, DMA 环缓冲上限) |
| `g_loop_slow_count` (main.c) | 迭代耗时 > 2,050,000 (≈12.2ms) 的次数 | 非 0 = 存在明显停顿，直接指向慢轮次 |
| `g_loop_over5ms_count` (main.c) | 迭代耗时 > 840,000 (**≈5.0ms**) 的次数 | 非 0 = 该轮跨过完整 5ms 采样节拍，`xv7021_sample_due` 存在被吞风险（采样周期抖动）；当前峰值 4.68ms 下应保持 0 |
| `g_kf_run_count` (navi_bridge.c) | Kalman 链执行次数 | 对齐后应每 10ms +1 (=100Hz) |
| `g_kf_update_max` (navi_bridge.c) | `kalman15d_update` 峰值 | ÷168000 ≈ ms |
| `g_kf_mes_max` (navi_bridge.c) | `Kalman15d_MesUpdate` 峰值 (含 mat_inverse) | ÷168000 ≈ ms，**25ms 峰值的头号嫌疑** |
| `g_kf_fb_max` (navi_bridge.c) | `Kalman_Feedback` 峰值 | ÷168000 ≈ ms |
| `g_sample_count` (main.c) | 采样事件累计次数 | 与 `g_kf_run_count` 应约 2:1 |
| `g_sample_total_max` (main.c) | 整个采样分支峰值 | ÷168000 ≈ ms，含下列四项 |
| `g_spi_acc_max` (main.c) | ADXL357 读取 + 换算峰值 | ÷168000 ≈ ms |
| `g_spi_gyro_max` (main.c) | XV7021BB 三颗陀螺采样峰值 | ÷168000 ≈ ms |
| `g_temp_max` (main.c) | 4 路温度读取峰值 | ÷168000 ≈ ms |
| `g_nav_feed_max` (main.c) | `navi_bridge_feed_imu` 峰值 | ÷168000 ≈ ms，采样段主体 |
| `g_send_max` (main.c) | 导航帧打包上报峰值 | ÷168000 ≈ ms |
| `g_feed_pre_max` (navi_bridge.c) | feed 内：GNSS 取数 + 解包 | ÷168000 ≈ ms |
| `g_feed_tc_max` (navi_bridge.c) | feed 内：`GetErrorPara_TC` 温补参数 | ÷168000 ≈ ms（18 次 double `pow`） |
| `g_feed_ec_max` (navi_bridge.c) | feed 内：`ErrorComp` ×2 | ÷168000 ≈ ms |
| `g_feed_chk_max` (navi_bridge.c) | feed 内：`GetGNSSType` + `IsGNSSDataValid` | ÷168000 ≈ ms |
| `g_feed_ins_max` (navi_bridge.c) | feed 内：粗对准 / `ins_update_RT` | ÷168000 ≈ ms |

> `g_loop_slow_count` 取代了最初的滑动平均 `g_loop_cycles_avg`：主循环绝大多数迭代是 1µs 级空转，
> 平均值会被稀释到看不见峰值，而"超阈值次数"能直接指出停顿发生过几次。
> `g_kf_*` 三个峰值对比即可定位 25ms 花在哪一步（`update` / `MesUpdate` / `Feedback`）。

> `g_gnss_*` 刻意不加 `static`，因此**在任意断点**（包括停在 main.c 时）都能直接加入 Watch。
> 反之 `parse_frame()` 内的 `lat_deg` / `pdop` 等是**栈上局部变量**，无论在哪个文件断点都取不到，
> 必须把断点设进该函数内部才能观察。
>
> `g_loop_cycles_max` 是**上电以来的历史最大值**（单调不减），必须**重新烧录并复位**（ZI 段在复位时清零）
> 才会归零；若只是让程序继续跑，旧值会一直保留，容易误判。
> **注意**：停在断点上时 CPU 停止但 USART+DMA 仍在收数据，环缓冲会持续回绕，
> 恢复后 `g_gnss_ovf_count` / `g_gnss_drop_count` 会被放大 —— 要拿真实值必须
> **清零后不设断点连续跑 30s，再暂停一次读取**。

> `g_loop_cycles_max` 若接近或超过 8,215,000 cycles (≈48.9ms)，说明 `gnss_hjd_pump()` 调用间隔过大，
> 1024 字节 DMA 环缓冲会被覆盖（此时 `g_gnss_ovf_count` 同步增长）—— 需优化 Kalman 链耗时或继续增大
> `GNSS_RX_DMA_BUF_SIZE`。

---

## 实测耗时分布 (v1.2.0, 168MHz)

测量方式：`DWT->CYCCNT` 打点，**所有数值为峰值（max），÷168000 得毫秒**。
测试条件：水平粗对准完成后的稳态运行（Kalman 已进主循环），GNSS 已定位（39°N/116°E）。
表中同时给出 -O0 与 -O2 两次实测，用于说明优化等级到底改变了什么。

> 本表数据均为 **double 版本** 的实测基线（-O2）。v1.2.0 已把 Kalman 的矩阵乘法改为
> float32 内部运算（`mat_multi_f32`），Kalman 两步的耗时需重新烧录后以 `g_kf_*` 复测填入。

### 主循环总账

| 环节 | 变量 | 周期 (-O2) | 时间 (-O2) | -O0 基线 | 降幅 | 占比 |
| --- | --- | --- | --- | --- | --- | --- |
| 单轮最坏 | `g_loop_cycles_max` | 4,194,545 | **24.97 ms** | 26.77 ms | −6.7% | 100% |
| Kalman 时间更新 | `g_kf_update_max` | 1,869,139 | **11.13 ms** | 12.02 ms | −7.4% | 44.6% |
| Kalman 量测更新 | `g_kf_mes_max` | 1,869,639 | **11.13 ms** | 11.97 ms | −7.0% | 44.6% |
| Kalman 反馈 | `g_kf_fb_max` | 1,168 | 6.95 µs | 7.3 µs | — | 0.03% |
| 采样段（含 feed + SPI + 上报） | `g_sample_total_max` | 453,015 | **2.70 ms** | 2.798 ms | −3.7% | 10.8% |
| 空转轮（无采样无 Kalman） | `g_loop_cycles_last` | 297,288 | 1.77 ms | 1.46 µs | — | — |

### 采样段内部

| 环节 | 变量 | 周期 (-O2) | 时间 (-O2) | -O0 基线 | 降幅 |
| --- | --- | --- | --- | --- | --- |
| `navi_bridge_feed_imu` 合计 | `g_nav_feed_max` | 425,510 | **2.533 ms** | 2.604 ms | −2.7% |
| └ 温补参数 `GetErrorPara_TC` | `g_feed_tc_max` | 227,262 | **1.353 ms** | 1.360 ms | −0.5% |
| └ 粗对准 / `ins_update_RT` | `g_feed_ins_max` | 158,741 | 0.945 ms | 0.988 ms | −4.5% |
| └ `ErrorComp` ×2 | `g_feed_ec_max` | 36,377 | 0.217 ms | 0.234 ms | −7.3% |
| └ GNSS 取数 + 原始量解包 | `g_feed_pre_max` | 2,375 | **14.1 µs** | 213 µs | **−93%** |
| └ `GetGNSSType` + `IsGNSSDataValid` | `g_feed_chk_max` | 425 | **2.5 µs** | 49 µs | **−95%** |
| SPI0 三颗陀螺 | `g_spi_gyro_max` | 10,072 | 60.0 µs | 73.8 µs | −19% |
| ADXL357 + 4 路温度 | — | ≈19,000 | ≈114 µs | ≈114 µs | — |
| 导航帧打包上报 | `g_send_max` | 838 | 5.0 µs | 5.9 µs | −16% |

> feed 内五段之和略大于 `g_nav_feed_max`，因各段峰值取自不同轮次，非同一轮相加。

### float32 化后实测 (2026-10-27 烧录, Kalman 已换 `mat_multi_f32`)

| 环节 | 变量 | 周期 | 时间 | double 基线 | 变化 |
| --- | --- | --- | --- | --- | --- |
| 单轮最坏 | `g_loop_cycles_max` | 1,308,970 | **7.79 ms** | 24.97 ms | **−68.8%** |
| Kalman 时间更新 | `g_kf_update_max` | 409,178 | **2.44 ms** | 11.13 ms | **4.57×** |
| Kalman 量测更新 | `g_kf_mes_max` | 446,274 | **2.66 ms** | 11.13 ms | **4.19×** |
| Kalman 反馈 | `g_kf_fb_max` | 1,128 | 6.7 µs | 6.95 µs | 持平 |
| 采样段 | `g_sample_total_max` | 452,903 | 2.70 ms | 2.696 ms | 持平（对照组） |
| `navi_bridge_feed_imu` 合计 | `g_nav_feed_max` | 425,400 | 2.53 ms | 2.533 ms | 持平 |
| └ 温补 `GetErrorPara_TC` | `g_feed_tc_max` | 227,239 | 1.353 ms | 1.353 ms | 持平（pow 替换前） |
| └ `ins_update_RT` | `g_feed_ins_max` | 158,930 | 0.946 ms | 0.945 ms | 持平 |
| SPI0 三颗陀螺 | `g_spi_gyro_max` | 9,968 | 59.3 µs | 60.0 µs | 持平 |

- `1,308,970 ≈ 409,178 + 446,274 + 452,903`：峰值轮同时含 Kalman 两步与采样，与设计（Kalman 每 2 帧）一致。
- 采样段逐项与改动前逐项吻合（<0.5%），**佐证本次改动只影响 Kalman 内部乘法**，未误伤补偿/采样/GNSS 链路。
- **只拿到 4.6 倍而非理论 24 倍**：15×15×15 每次只有 3,375 次 MAC，其余开销来自
  ①每调用一次要逐元素做 double↔float 转换（单次 TimeUpdate ≈2,300 次 `__aeabi_d2f/__aeabi_f2d` 库调用）；
  ②`mat_inverse(6)`、`mat_transpose/mat_plus/mat_minus/mat_assign` 仍是 double；
  ③内层循环编译成约 13 周期/MAC（`s_f32_b[k*p+j]` 每步重算索引 + 2 次 `VLDR`）。
- `g_feed_tc_max` 一行的 1.353 ms 是 **pow 替换前**的数值；pow→乘法已完成（Code 34,280→34,064），
  预期降到 ≈0.12 ms、采样段 ≈1.34 ms、峰值 ≈6.4 ms，待下次烧录回填。

### Kalman 全 float 化后实测 (B, 2026-10-27 第三次烧录)

| 环节 | 变量 | 周期 | 时间 | 上一轮 | 变化 |
| --- | --- | --- | --- | --- | --- |
| 单轮最坏 | `g_loop_cycles_max` | 906,421 | **5.39 ms** | 6.53 ms | **−17.4%** |
| Kalman 更新时间 | `g_kf_update_max` | 335,695 | **2.00 ms** | 2.36 ms | −15.4% |
| Kalman 量测更新 | `g_kf_mes_max` | 329,243 | **1.96 ms** | 2.66 ms | −26.2% |
| Kalman 反馈 | `g_kf_fb_max` | 1,220 | 7.3 µs | 6.7 µs | 持平 |
| 采样段 | `g_sample_total_max` | 240,915 | 1.434 ms | 1.435 ms | 持平（对照组） |
| `feed` 合计 | `g_nav_feed_max` | 213,407 | 1.270 ms | 1.272 ms | 持平 |
| └ 温补 `GetErrorPara_TC` | `g_feed_tc_max` | 14,655 | 87.2 µs | 87.2 µs | 持平 |
| └ `ins_update_RT` | `g_feed_ins_max` | 158,841 | 945.5 µs | 946.0 µs | 持平 |

- `906,421 ≈ 335,695 + 329,243 + 240,915`：峰值轮仍是「Kalman 两步 + 采样」同轮，结构未变。
- **收益远小于预期**（原估 Kalman 降到 1.1~1.3 ms/步，实际 2.00/1.96 ms）。原因由反汇编确认：
  `mat_multi_f` 内层 11 条指令，且**每个 MAC 都执行一次 `VSTR`**（不把累加值留在寄存器），
  再叠加每步重算索引（1 次 `MUL` + 3 次 `ADD`），单 MAC 约 13~14 周期；
  而实测窗口折合 32~38 周期/MAC → 说明**转换开销只占约 7 周期/MAC**，
  其余大头在 `g_kf_update_max` 窗口内的 `GetKalman15d_SSMat`：
  它含 15×15 double 数乘 + 225 次 double→float 转换 + 3 次 `cos()` / 2 次 `sin()`（double 软件三角函数）。
- 结论：**下一步的最优解不是方案 C（`mat_inverse` float 化，约 0.29 ms），而是**
  ①把 `GetKalman15d_SSMat` 里重复计算的 `cos(2·lat)`/`sin(2·lat)` 提出来只算一次、并改用 `cosf/sinf`；
  ②把 `mat_multi_f` 内层改为寄存器累加 + 指针走查（或先把 b 转置使内层连续访存）。
  两者都只是纯计算优化，不涉及时序/协议。

### Kalman 全 float + -O3 + `__restrict` 实测 (C, 2026-10-27 第四次烧录)

在 B 的基础上做两件事：6 个 float 矩阵内核形参加 `const` + `__restrict`；Keil 优化等级
Level 2 → Level 3（`<Optim>4</Optim> <oTime>1</oTime>`，命令行 `-O3 -Otime`）。算法、时序、
协议、结构体全部未动。

| 环节 | 变量 | 周期 | 时间 | B 轮 (-O2) | 变化 |
| --- | --- | --- | --- | --- | --- |
| 单轮最坏 | `g_loop_cycles_max` | 786,661 | **4.682 ms** | 5.395 ms | **−13.2%** |
| Kalman 时间更新 | `g_kf_update_max` | 277,125 | **1.650 ms** | 2.00 ms | −17.5% |
| Kalman 量测更新 | `g_kf_mes_max` | 269,862 | **1.606 ms** | 1.96 ms | −18.1% |
| Kalman 反馈 | `g_kf_fb_max` | 1,205 | 7.2 µs | 7.3 µs | 持平 |
| 采样段 | `g_sample_total_max` | 232,456 | 1.384 ms | 1.434 ms | 持平（对照组） |
| `feed` 合计 | `g_nav_feed_max` | 213,407 | 1.270 ms | 1.270 ms | 持平 |
| └ 温补 `GetErrorPara_TC` | `g_feed_tc_max` | 13,278 | 79.0 µs | 87.2 µs | 持平 |
| └ `ins_update_RT` | `g_feed_ins_max` | 154,026 | 916.8 µs | 945.5 µs | 持平 |

- `786,661 ≈ 277,125 + 269,862 + 232,456 = 779,443`：峰值轮仍是「Kalman 两步 + 采样」同轮，
  差值约 0.7% 属各峰值取自不同轮次的正常误差。
- **收益全部来自 Kalman 两步**：合计 −148,951 周期（−0.887ms），采样/feed 各段逐项持平。
- 反汇编（`fromelf --text -c navi_matrix.o`）确认内层由 11 条指令/MAC 降到约 4.5 条：
  k 循环 2 路展开、指针走查替代每次重算 `i*n+k`、相邻两次 MAC 共用同一 S 寄存器（不再逐次 `VSTR`）。
- **数值语义**：ARMCC V5 的 `-O3` 不改浮点语义，`--fpmode` 默认仍为 `std`（严格 IEEE 754，
  不做重结合），故累加顺序不变；与 `-O2` 的差别只在指令调度层。
- **代价**：Code 34,512 → 39,272 B（+4,760 B，循环展开 + 诊断计数器），RO/ZI 不变，RW +8 B。

### 采样节拍完整性验证 (2026-10-28 烧录)

新增 `g_loop_over5ms_count`（阈值 840,000 周期 = 5.0ms，正好等于采样节拍）后复位连跑：

| 变量 | 实测 | 结论 |
| --- | --- | --- |
| `g_loop_over5ms_count` | **0** | 5ms 采样节拍零吞帧 |
| `g_loop_slow_count` | **0** | 无 >12.2ms 慢迭代 |
| `g_loop_cycles_max` | 781,969 (4.654 ms) | 峰值稳定 <5ms |
| `g_gnss_ovf_count` / `g_gnss_drop_count` | 2 / 37 | 见下 |

**断点假象铁证**：`over5ms = 0` 表示主循环从未有一轮超过 5ms，而环缓冲真实覆盖需要停顿
>48.9ms —— 逻辑上不可能。因此 `ovf=2 / drop=37`（2 次 × ~20 帧 ≈ 37，自洽）只能来自本次
调试的 2 次断点停机。readme 此前"非零计数均为调试假象"的判断由此坐实。

### 结论

- **-O2 生效但只解决纯 C 逻辑**：`g_feed_pre` −93%、`g_feed_chk` −95%、SPI −19%、Code 42,152→34,008
  （pow 替换后为 34,064），证明优化器正常；但 Kalman 只降 7%，因为其 226 周期/次 MAC 基本都花在
  `__aeabi_dmul` / `__aeabi_dadd` / `_double_epilogue` 三个**库函数调用**上，优化器压不动。
- **瓶颈是 double 软件浮点**：Kalman 两步合计 22.26 ms，对应约 24,000 次 double MAC；
  `mat_inverse` 只占 MesUpdate 的 3~4%，不是元凶，`mat_multi` 的 15×15×15 稠密乘法（每周期 4 次）占 74%。
- **`pow()` 是采样段最大单项且 -O2 完全无效**（1.360 → 1.353 ms）：`GetErrorPara_TC` 每周期
  调 18 次 double `pow(x,2/3/4)`，实测 ≈12,700 周期/次（≈75µs）；改为 `x*x` / `x*x*x` /
  `(x*x)*(x*x)` 可省约 1.2 ms。
- **SPI 完全不是瓶颈**：三颗陀螺 60µs + 加速度/温度约 114µs + 上报 5µs，合计仅占采样段 7%。
- **致命后果不是"慢"而是采样被吞**：`xv7021_sample_due` 是标志位而非计数器，
  24.97ms 迭代期间约 5 次 5ms 定时事件只被取走 1 次。实测 `g_kf_run_count`/`g_gnss_frame_count`
  推算 Kalman ≈24Hz、采样 ≈48Hz（设计 100/200Hz），算法仍按 `ts=0.005`/`nts=0.01` 积分
  → 姿态转动量、速度、位移系统性偏小 4 倍。
- **达标账**：5ms = 840,000 周期。double 版 4,191,793（超 5 倍）→ 仅乘法下沉 float32 1,308,970（7.79ms）
  → Kalman 全 float 化 **906,421（5.39ms，仍超 8%）**，还需再砍 66,421 周期（0.395 ms）。
- **float32 化两步实测**：第一步把乘法下沉 float（`mat_multi_f32`）使 Kalman 两步 11.13/11.13 → 2.44/2.66ms；
  第二步把 SSMat 与全部临时矩阵改 float（`mat_multi_f*`，方案 B）→ **2.00 / 1.96ms**，累计约 5.6×。
  实测证明剩余瓶颈是内层循环（每个 MAC 一次 `VSTR`）与 `GetKalman15d_SSMat` 的 double 三角函数，
  **不是** double↔float 转换（其只占约 7 周期/MAC）。
- **pow 已消除**：`GetErrorPara_TC` 的 18 次 `pow(x,2/3/4)` 改为乘法，预期再省约 1.25ms（采样段 2.70 → ≈1.34ms）。

---

## 构建

Keil MDK-ARM (armclang / ARMCC), 优化等级 **Level 3 (-O3 -Otime)** (`<Optim>4</Optim> <oTime>1</oTime>`):

```
Open project/template.uvprojx → Rebuild All
Expected: 0 error, 0 warning
Flash size: ~39.7KB / 480KB  (Code 39,272 + RO 1,088 + RW 252 = 40,612 B)
SRAM size:  ~57.1KB / 128KB  (RW 252 + ZI 58,212 = 58,464 B，ZI 含 Stack 28,672)
Stack      : Stack_Size 0x7000 (startup_gd32f405.s)；全 float 化后 MesUpdate 栈数组
             约 11.2KB (估算，含 4 个 mat_inverse double 桥接矩阵 1,360 B)，余量充足
```

> 改 `.uvprojx` 的优化等级不会触发全量重编，需先删 `Objects/*.o *.d *.crf *.axf` 再 Rebuild。
> 优化等级与命令行的对应关系：`<Optim>3</Optim>` → `-O2`，`<Optim>4</Optim>` → `-O3`。

---

## v1.0.7 → v1.1.0 改动清单

| 类别 | 改动 |
| --- | --- |
| **新文件** | Navi/ 43 个算法文件 (21 .c / 22 .h), User/navi_bridge.c, User/navi_bridge.h |
| **修改** | version.h (v1.0.7→v1.1.0), main.c (加导航调用链), navi_api.c (nn=2, nts=0.01), navi_coarse_align.c (补 flag_EnvFactor=1), navi_error_comp.c (malloc→static) |
| **startup** | Stack 0x400→0x2000, Heap 0x400→0x0000 |
| **常量** | 所有 π/角度转换统一走 navi_constants.h (arcdeg / 1.0/arcdeg), 零字面量 PI |
| **坐标系** | gyro: navi_x=+gyro[1], navi_y=-gyro[2], navi_z=-gyro[0] |
| | accel: navi_x=-adxl.y_g, navi_y=-adxl.x_g, navi_z=-adxl.z_g |

---

## v1.1.0 → v1.2.0 改动清单

| 类别 | 改动 |
| --- | --- |
| **新文件** | User/gnss_hjd.c, User/gnss_hjd.h (T_HJD_POSDATA 接收解析) |
| **协议解析** | 同步字 0x55→0xBB、整帧 51 字节、累加和低 8 位 (索引 0~49)、**小端**、经纬度 **1e-6** |
| **USART0** | 复用为 GNSS1 RX 专用：230400 8E1，DMA1_CH2 循环 + 256B 环形缓冲；TX 保留 DMA1_CH7 发导航结果帧 |
| **DMA 冲突** | SPI0_RX 由 **DMA1_CH2 → DMA1_CH0 (SUBPERI3)**，避让 USART0_RX；`spi0_dma_transfer_bytes` 更名 `spi0_dma_transfer` |
| **删除** | UART0 RX 环形缓冲 + RBNE 中断、`version_protocol_poll()` 及版本查询应答协议 |
| **导航接入** | navi_bridge.c GNSS1 由 `gnss_hjd_get_result()` 取实测帧，替换原 stub；GNSS2 仍 stub |
| **超时参数** | `GNSSINVALIDTIME` 10 → **2000** (`cnt_NoGNSS > 2000 × fs` 置 `flag_GNSSfault`) |
| **可观测性** | gnss_hjd 增 `g_gnss_drop_count` / `g_gnss_ovf_count` 丢帧计数 + `gnss_hjd_get_drop_count()`；诊断变量改为非 static 全局 `g_gnss_*` 供 Watch 任意断点观察；main 增 DWT CYCCNT 主循环耗时测量（last / max / slow_count） |
| **环缓冲** | `GNSS_RX_DMA_BUF_SIZE` 256 → **1024** (12.2ms → 48.9ms 覆盖窗口)，并引入 DMA 圈数计数 + 累计读指针，使回绕覆盖可检测 (`g_gnss_ovf_count`) 且不再喂入已被覆盖的旧字节 |
| **文档** | 协议文档纠正为**小端 + 1e-6**（原"大端 + 1e-7"与实测帧不符） |
| **修复 (致命)** | `Stack_Size` 0x2000 → **0x7000**：15d Kalman 的 `MesUpdate` 栈需求 22,040 B 远超原 8,192 B，溢出 14KB 压坏 `.bss`（`s_dma_buf` / `g_gnss_result` / `aug` / `imu_temp_para`），导致粗对准一结束 GNSS 结果即被清零、求逆结果不可信 |
| **性能** | 优化等级 Level 0 → **Level 2 (-O2)**；删除 `GetKalman15d_SSMat.c` 中 4 个从未引用的局部数组（`mat_temp10/11/12`、`v_temp3`）以消除 4 个 #177-D 警告 |
| **新增打点** | main.c 增 `g_sample_*` 采样段打点（acc/gyro/temp/feed/send/total）；navi_bridge.c 增 `g_feed_*` feed 内 5 段打点（pre/tc/ec/chk/ins） |
| **实测数据** | 耗时分布与结论写入「实测耗时分布」章节（-O0 基线，供 -O2 后对照） |
| **float32 化** | `Navi/navi_math` 新增 `mat_multi_f32`：对外签名与 `mat_multi` 一致（double 入参/出参），内部先下沉 float 再累加后回写；暂存区为文件级 `static float[225]×3`（**2,700 B，零 heap**），超出 15×15 规模自动回退 double；输入先复制为私有副本，故输出与入参同址安全 |
| **float32 化 - 调用点** | `Kalman15d_TimeUpdate.c` 5 处 + `Kalman15d_MeaUpdate.c` 单点定位分支 8 处 `mat_multi` → `mat_multi_f32`；**未改动**：`Kalman15d_SSMat` 结构体成员类型、`mat_transpose/mat_plus/mat_minus/mat_assign/mat_unit`、`mat_inverse(6)`、GA 双天线分支 8 处（`flag_azimuthvalid` 恒 0，实际不执行）、`GetKalman15d_SSMat` 的 3×3 乘法 |
| **float32 化 - 验证** | 反汇编确认内层为 `VLDR s1/s2` + **`VMLA.F32`**（≈6 周期/MAC，对比 double 226 周期，理论 ≈24 倍），不再调用 `__aeabi_dmul/__aeabi_dadd`；Code 34,008→34,280，ZI 62,436→65,140，栈峰值 22,448→22,504（余量 6,168 B） |
| **float32 化 - 风险** | 精度由 double(≈15 位) 降至 float(≈7 位)，`P` 阵动态范围大时可能出现可见漂移；需与双精度同源算例对照姿态/位置，回退只需把调用改回 `mat_multi`（无需动结构体） |
| **float32 实测** | Kalman 时间/量测更新 11.13/11.13ms → **2.44/2.66ms**（4.57×/4.19×），主循环峰值 24.97 → **7.79ms**（−68.8%）；采样段逐项与改动前一致（对照组，证明未误伤别处）。未达理论 24× 的剩余开销：每步约 2,300 次 double↔float 转换、`mat_inverse(6)`/转置/加减仍 double、内层约 13 周期/MAC |
| **消除 pow** | `GetErrorPara_TC` 温度回归向量 18 处 `pow(x,2/3/4)` 改为乘法（`T^2=T*T`、`T^3=T^2*T`、`T^4=T^2*T^2`，复用已算的 `T` 与 `T^2`）；标量求值的 12 处 `mat_multi(1,5,1)` 及 `errpara_TC` 写入布局未动；Code 34,280→34,064 |
| **Kalman 全 float (B)** | `Kalman15d_SSMat` 8 个成员 → `float`；`navi_math` 新增 6 个 float 原生内核 `mat_multi_f/mat_plus_f/mat_minus_f/mat_assign_f/mat_transpose_f/mat_unit_f`，**删除**上一版 `mat_multi_f32` 及其 2,700B 静态暂存区；`TimeUpdate` 9 处、`MeaUpdate` 19 处调用改 `_f` 版；`mat_inverse` 仍 double，用 `inv_in/out` 桥接（每步 72 次转换，原约 2,300 次）；`GetKalman15d_SSMat` 的 `Phikk_1=I+Ft*nts` 改 15×15 转换循环。**未动**：3×3 的 `Ft/Gt/Maa…` 构造、`mat_inverse` 本体、`Init`/`Feedback`、采样与 GNSS 链路 |
| **B 实测** | 更新时间 2.36→**2.00ms**、量测更新 2.66→**1.96ms**、峰值 6.53→**5.39ms**；采样/feed 各段持平（对照组）。距 5ms 预算还差 66,421 周期（0.395ms） |
| **B 构建** | Code 34,064→**34,512**、ZI 65,140→**58,212**（−6,928 = 暂存区 2,700 + 结构体 4,224 + 对齐）；`Kalman15d_MesUpdate` 栈数组 21.5KB→约 11.2KB |

---

## v1.2.0 → v1.3.0 改动清单

| 类别 | 改动 |
| --- | --- |
| **指针限定符** | `navi_math` 6 个 float 内核（`mat_multi_f/mat_plus_f/mat_minus_f/mat_assign_f/mat_transpose_f/mat_unit_f`）形参加 `const` + `__restrict`，`math_matrix.h` 声明与 `navi_matrix.c` 定义同步。告知编译器 a/b/c 互不重叠，消除别名检查与冗余重载。**未动**：函数体算法、double 版 `mat_multi` 等接口、全部调用点 |
| **优化等级** | Level 2 → **Level 3**（`<Optim>4</Optim> <oTime>1</oTime>`，命令行 `-O3 -Otime`）；仅改 `.uvprojx` 优化选项，零代码逻辑改动 |
| **-O3 实测** | 主循环峰值 906,421→**786,661 周期 (5.39→4.682ms, −13.2%)**；Kalman 时间更新 2.00→**1.650ms**、量测更新 1.96→**1.606ms**；采样/feed 各段逐项持平（对照组）。**首次进入 5ms 预算内，余量 53,339 周期 (0.318ms)** |
| **反汇编验证** | `mat_multi_f` 内层由 11 条指令/MAC 降至约 4.5 条：k 循环 2 路展开、指针走查替代每次重算 `i*n+k`、相邻两次 MAC 共用同一 S 寄存器（不再逐次 `VSTR`） |
| **构建** | Code 34,512→**39,272**（+4,760，循环展开 + 诊断计数器），RO 1,088 / ZI **58,212** 不变，RW 244→**252**；Flash **≈39.7KB / 480KB**，SRAM **≈57.1KB / 128KB**；0 Error 0 Warning |
| **新增诊断** | main.c 增 `g_loop_over5ms_count`（阈值 840,000 周期 = 5.0ms 采样节拍，吞节拍判据）；原 `g_loop_slow_count`（12.2ms）保留。纯诊断打点，零业务影响 |
| **节拍验证** | 复位连跑：`g_loop_over5ms_count = 0`、`g_loop_slow_count = 0` → **5ms 采样节拍零吞帧实锤**；峰值 781,969 周期（4.654ms）稳定。同次 `ovf=2/drop=37` 与 `over5ms=0` 并存，而真实覆盖需停顿 >48.9ms，逻辑不可能 → 坐实断点假象结论 |
| **数值语义** | ARMCC V5 的 `-O3` 不改浮点语义（`--fpmode` 默认 `std`，严格 IEEE 754，不做重结合），累加顺序不变，与 `-O2` 的差异只在指令调度层 |
| **GNSS 链路** | 连续运行观察首次得到**零丢帧零覆盖**：`g_gnss_drop_count = 0`、`g_gnss_ovf_count = 0`、`g_gnss_err_count = 0`，收发帧计数一致。此前所有非零计数均为 Keil 断点停机期间 DMA 续灌环缓冲的**调试假象** |
| **风险识别 (未改代码)** | `xv7021_sample_due`（`Peripheral/Timer/timer.c`）为普通 `uint8_t`，无 `volatile`，却被 TIMER3 ISR 写、主循环读。`-O3` 理论上可将该读取提到循环外导致采样冻结；实测因跨编译单元 + 主循环调用大量外部函数而未触发。建议后续加 `volatile` 加固（与优化等级无关的正确性改进） |
| **调试代价** | `-O3` 下局部变量常被优化进寄存器，Keil Watch 显示 `<cannot evaluate>`，单步跳转不连续 |

---

## 已知未闭环问题 (与优化等级无关)

| 问题 | 现状 | 静态分析结论 |
| --- | --- | --- |
| 静止时 `vn` 缓慢增长 | 已观测（-O2 / -O3 均存在，非优化引入） | `Kalman_Feedback` 只反馈 `xk[3..8]`（速度/位置），失准角 `xk[0..2]` 与零偏 `xk[9..14]` **不回馈**。姿态矩阵的航向误差不被修正，重力经 `cnb` 投影出水平分量持续注入 `vn`；1Hz 的 Feedback 修 `vn` 只能追平累积量，无法消除误差源 |
| 姿态角输出不刷新 | 已确认 | `navi_bridge_get_state()` 直接取 `g_imu.att[]`，而 `g_imu.att` 只在粗对准完成时写一次；`ins_update_RT` 递推更新的是 `g_imu.cnb`。修复方式为改由 `m2att(g_imu.cnb, att_rad)` 反解，或在 Feedback 修正 `cnb` 后同步刷新 `att` |
