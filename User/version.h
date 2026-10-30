#ifndef __VERSION_H
#define __VERSION_H

#include <stdint.h>

/* 固件版本号 */
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 5
#define FW_VERSION_PATCH 0
#define FW_VERSION_STR "IMU+NAV v1.5.0"

/* 版本更新记录
 * 2026-09-14 使用外部 16 MHz 晶振。
 * 2026-09-17 修正陀螺仪和加速度计的轴映射与方向。
 * 2026-09-22 添加 MMC5983MA 地磁传感器驱动（当前采集与输出暂不使用）。
 * 2026-09-23 [v1.0.5] ADXL357 的 SPI1 数据读写改为 DMA 传输。
 * 2026-09-23 [v1.0.5] UART0 增加 DMA 发送，波特率设为 230400。
 * 2026-09-23 [v1.0.5] 加速度计、三个陀螺仪及温度数据按 5 ms 周期采集。
 * 2026-09-23 [v1.0.5] 串口帧陀螺仪角速度单位为 °/s，加速度单位为 m/s²。
 * 2026-09-23 [v1.0.5] 精简未使用的 SPI、UART 和定时器接口，统一相关变量命名。
 * 2026-09-23 [v1.0.6] 增加 UART0 接收、版本查询命令解析及二进制版本应答。
 * 2026-09-23 [v1.0.7] 传感器初始化失败时直接返回区分加速度计/陀螺仪的故障码。
 * 2026-10-23 [v1.1.0] 移植复旦微 A7 平台组合导航算法包 (Navi/)：
 *   - navi_bridge.c 逐行复刻 A7 FM_QL_lwip/main.c 的导航处理链
 *     (imu_frame_process_all + ErrorComp×2 + 粗对准状态机 + ins_update_RT)
 *   - 15 维 Kalman 滤波器全链路 (GetKalman15d_SSMat + TimeUpdate + MesUpdate + Feedback)
 *   - GNSS1/GNSS2 stub 模拟 (34.2°N/108.8°E/400m, heading1=180°, heading2=45°)
 *   - 变量命名全面对齐 A7 (g_imu/g_sys/g_ahrs/gnss_info/g_k15s)
 *   - 所有角度/弧度转换统一走 navi_constants.h 常量, 零字面量 PI
 *   - 双子样 5ms×2=10ms 一次完整 INS + Kalman 周期
 *   - 粗对准只在上电跑一次 (flag_Finish_calign 锁存)
 *   - mat_inverse.c malloc 改 static 数组 (零 heap)
 *   - startup Stack_Size 0x400→0x2000, Heap_Size 0x400→0x0000
 *   - 后续可接 MCU 422 串口替换 GNSS1 stub
 * 2026-10-27 [v1.2.0] USART0 改为 GNSS1 卫导数据接收专用口 (真机接入):
 *   - 新增 User/gnss_hjd.c/.h, 解析 T_HJD_POSDATA 帧:
 *     同步字 0x55 0xBB, 整帧 51 字节, 累加和低 8 位校验 (索引 0~49)
 *   - 数据字段为小端字节序; 经纬度单位 1e-6 度, 高度/速度 0.1 单位
 *   - RX 走 DMA1_CH2 循环模式 + 应用层环形缓冲, 主循环逐字节解析
 *   - navi_bridge.c 由 gnss_hjd_get_result() 取实测帧, 替换 GNSS1 stub
 *   - SPI0_RX 的 DMA 由 DMA1_CH2 迁至 DMA1_CH0 (SUBPERI3), 避让 USART0_RX
 *   - 删除 UART0 接收环形缓冲、RBNE 中断及版本查询协议 (version_protocol_poll)
 *   - GNSSINVALIDTIME 10 → 2000 (无效计数阈值, 与 200Hz 调用节拍匹配)
 *   - gnss_hjd 增加丢帧计数 getter (邮箱覆盖可观测)
 *   - main 增加 DWT CYCCNT 主循环耗时测量 (评估 DMA 环缓冲覆盖风险)
 *   - 诊断变量改为非 static 全局 g_gnss_* (Keil Watch 任意断点可读)
 *   - GNSS RX 环缓冲 256 → 1024 字节 (12.2ms → 48.9ms 覆盖窗口):
 *     粗对准完成、15d Kalman 全链路接入后单次迭代可达 ~25ms, 原缓冲被 DMA 覆盖
 *   - 环缓冲改为 DMA 圈数 + 累计读指针建模, 新增 g_gnss_ovf_count 显式统计覆盖
 *   - main 增 g_loop_slow_count (迭代 >12.2ms 的次数); navi_bridge 增 Kalman
 *     三个子步骤峰值打点 (g_kf_update_max / g_kf_mes_max / g_kf_fb_max)
 *   - 修复致命栈溢出: Stack_Size 0x2000 → 0x7000 (MesUpdate 栈需求 22,040B,
 *     原 8KB 溢出压坏 s_dma_buf / g_gnss_result / aug 等 .bss 对象)
 *   - main 增采样段打点 (g_sample_*), navi_bridge 增 feed 内 5 段打点 (g_feed_*)
 *   - 优化等级 Level 0 → Level 2 (-O2); 删除 GetKalman15d_SSMat.c 中 4 个
 *     未引用局部数组 (mat_temp10/11/12, v_temp3)
 *   - 15d Kalman float32 化: navi_math 新增 mat_multi_f32 (对外 double 接口,
 *     内部下沉 float 累加, 3×225 float 静态暂存 = 2,700B, 零 heap), 反汇编确认
 *     内层循环为 VLDR + VMLA.F32 硬件乘加, 不再调用 __aeabi_dmul/__aeabi_dadd
 *   - Kalman15d_TimeUpdate.c 5 处 / Kalman15d_MeaUpdate.c 单点定位分支 8 处
 *     mat_multi → mat_multi_f32; SSMat 结构体成员、mat_inverse(6)、mat_transpose
 *     及 GA 双天线分支 (实际不执行) 仍保持 double
 *   - 精度由 double (≈15 位有效) 降为 float (≈7 位有效), 需与 A7 平台双精度
 *     算例做同源对照, 回退方式为把调用改回 mat_multi (无需改结构体)
 *   - Code 34,008 → 34,280B, ZI 62,436 → 65,140B; 链接器栈峰值 22,504B
 *   - float32 化实测 (2026-10-27 烧录): Kalman 时间更新 11.13 → 2.44ms (4.57x),
 *     量测更新 11.13 → 2.66ms (4.19x); 主循环峰值 24.97 → 7.79ms (-68.8%);
 *     采样段逐项与改动前一致 (2.70ms), 佐证仅 Kalman 乘法被替换
 *   - 未达理论 24x 的原因: 每次调用需逐元素 double↔float 转换 (单步约 2,300 次
 *     __aeabi_d2f/__aeabi_f2d 库调用), mat_inverse(6)/转置/加减仍为 double,
 *     且内层循环编译为约 13 周期/MAC (索引每步重算)
 *   - GetErrorPara_TC 温度回归向量 pow(x,2/3/4) 共 18 处改为乘法
 *     (T^2=T*T, T^3=T^2*T, T^4=T^2*T^2), 消除每周期 18 次 double pow
 *     (实测 1.353ms, 其中 pow 约占 1.25ms); 标量求值的 12 处 mat_multi 不动
 *   - Code 34,280 → 34,064 (-216B)
 *   - 15d Kalman 全 float 化 (方案B): SSMat 8 个成员 double→float (ZI -4,224B);
 *     navi_math 新增 6 个 float 原生内核 mat_multi_f/mat_plus_f/mat_minus_f/
 *     mat_assign_f/mat_transpose_f/mat_unit_f, 并删除上一版的 mat_multi_f32
 *     及其 2,700B double→float 静态暂存区; TimeUpdate 9 处、MeaUpdate 19 处
 *     调用改 _f 版; mat_inverse 仍为 double 接口, 用 inv_in/out 桥接
 *     (每步仅 72 次转换, 原约 2,300 次); GetKalman15d_SSMat 的
 *     Phikk_1=I+Ft*nts 改为 15x15 转换循环
 *   - 全 float 化实测: Kalman 更新时间 2.36→2.00ms, 量测更新 2.66→1.96ms,
 *     主循环峰值 6.53→5.39ms (-17.4%); 采样段与 feed 各段持平 (对照组)
 *   - 关键结论: 消除转换只省约 7 周期/MAC, 单步 MAC 仍有 32~38 周期。
 *     mat_multi_f 反汇编内层 11 条指令且每个 MAC 有一次 VSTR(不累积在寄存器),
 *     而 g_kf_update_max 窗口还包含 GetKalman15d_SSMat 的 double cos/sin
 *     与 15x15 double 数乘, 故剩余瓶颈不在转换而在内层循环与三角函数
 *   - Code 34,064 → 34,512B, ZI 65,140 → 58,212B;
 *     Kalman15d_MesUpdate 栈数组 21.5KB → 约 11.2KB
 * 2026-10-27 [v1.3.0] Kalman 矩阵内层优化 + 优化等级升到 Level 3 (真机实测):
 *   - navi_math 6 个 float 原生内核 (mat_multi_f / mat_plus_f / mat_minus_f /
 *     mat_assign_f / mat_transpose_f / mat_unit_f) 形参加 const + __restrict,
 *     声明 (math_matrix.h) 与定义 (navi_matrix.c) 同步修改; 告知编译器 a/b/c
 *     三指针互不重叠, 消除别名检查与冗余重载。工程内无同址调用, 零行为影响
 *   - Keil 优化等级 Level 2 → Level 3 (<Optim>4</Optim> <oTime>1</oTime>,
 *     实际命令行 -O3 -Otime); 仅改优化选项与指针限定符, 未动任何算法/时序/协议
 *   - -O3 实测 (2026-10-27 烧录, 168MHz, ÷168000 得 ms):
 *     主循环峰值 906,421 → 786,661 周期 (5.39 → 4.682ms, -13.2%);
 *     Kalman 时间更新 335,695 → 277,125 (2.00 → 1.650ms, -17.5%);
 *     量测更新 329,243 → 269,862 (1.96 → 1.606ms, -18.1%);
 *     采样段与 feed 各段与上一轮逐项持平 (对照组, 证明只影响矩阵内层)
 *   - 首次把峰值压到 5ms 预算以内: 4.682ms, 余量 53,339 周期 (0.318ms);
 *     与前一轮差值 119,760 周期 (0.713ms) 全部来自 Kalman 两步
 *   - 反汇编确认 mat_multi_f 内层由 11 条指令/MAC 降至约 4.5 条: k 循环 2 路展开 +
 *     指针走查替代每次重算 i*n+k, 相邻两次 MAC 共用同一 S 寄存器 (不逐次 VSTR)
 *   - Code 34,512 → 39,256B (+4,744B, 来自循环展开), RO 1,088 / RW 244 /
 *     ZI 58,212 三项均不变; Flash ≈39.6KB / 480KB, SRAM ≈57.1KB / 128KB
 *   - 新增诊断变量 g_loop_over5ms_count (main.c): 迭代耗时 > 840,000 周期
 *     (= 5.0ms 采样节拍) 的次数, 非 0 即 xv7021_sample_due 存在被吞风险;
 *     原 g_loop_slow_count (12.2ms 阈值) 保留不变。纯诊断, 零业务影响
 *   - 烧录验证 (2026-10-28): 复位连跑后 g_loop_over5ms_count = 0 且
 *     g_loop_slow_count = 0, 实证 5ms 采样节拍零吞帧; 主循环峰值
 *     781,969 周期 (4.654ms) 稳定 <5ms。同次观察中 g_gnss_ovf_count = 2 /
 *     g_gnss_drop_count = 37 与 over5ms = 0 并存, 而真实覆盖需要主循环
 *     停顿 >48.9ms —— 逻辑上不可能, 由此坐实所有 GNSS 覆盖/丢帧非零
 *     计数均为 Keil 断点停机期间 DMA 续灌环缓冲的调试假象
 *   - Code 39,256 → 39,272B (+16B, 新计数器判断), RW 244 → 252B
 *   - GNSS 链路首次观察到零丢帧零覆盖: g_gnss_drop_count = 0,
 *     g_gnss_ovf_count = 0, g_gnss_err_count = 0, 收发帧计数一致;
 *     此前所有非零计数均为 Keil 断点停机期间 DMA 继续灌满环缓冲的调试假象
 *   - 数值语义: ARMCC V5 的 -O3 不改变浮点语义 (--fpmode 默认 std, 严格 IEEE
 *     754, 不做重结合), 与 -O2 的差异只在指令调度层; 内层以单累加器顺序累加,
 *     累加顺序未变。风险仅在于 O3 下局部变量常被优化进寄存器, Keil Watch
 *     会出现 <cannot evaluate>, 单步调试体验下降
 *   - 已识别待处理风险 (本次未改动代码): xv7021_sample_due 在 timer.c 为普通
 *     uint8_t, 未加 volatile, 却被 TIMER3 ISR 写、主循环读。理论上 -O3 可将该
 *     读取提到循环外导致采样冻结; 实测未触发 (跨编译单元 + 主循环调用大量
 *     外部函数, 编译器必须每次重新加载), 建议后续加 volatile 加固
 *   - 已知未闭环 (与优化等级无关, 沿用 v1.2.0 行为): Kalman_Feedback 仅反馈
 *     xk[3..8] 速度/位置, 失准角 xk[0..2] 与零偏 xk[9..14] 不回馈。静态分析
 *     疑似: 静止时姿态矩阵的航向误差经重力投影持续注入水平速度, 表现为 vn
 *     缓慢增长。navi_bridge_get_state 的 att 仍取 g_imu.att[], 而 g_imu.att
 *     仅在粗对准完成时写一次, 故姿态角输出不随运动刷新。两项均待后续处理
 * 2026-10-28 [v1.4.0] 清理全工程打点代码, 合并导航算法最终优化成果:
 *   - 彻底删除 DWT CYCCNT 耗时测量基础设施 (CoreDebug->DEMCR / DWT->CTRL / DWT->CYCCNT):
 *     main.c 删 g_loop_cycles_last/max/slow_count/over5ms_count + LOOP_SLOW/OVER5MS_THRESHOLD
 *     + g_sample_count/g_spi_acc_max/g_spi_gyro_max/g_temp_max/g_nav_feed_max/g_sample_total_max/g_send_max;
 *     navi_bridge.c 删 g_kf_run_count/g_kf_update_max/g_kf_mes_max/g_kf_fb_max +
 *     g_feed_pre_max/g_feed_tc_max/g_feed_ec_max/g_feed_chk_max/g_feed_ins_max;
 *     main() 内删所有 t_a~t_f/prev_loop_cycles 局部变量及对应 DWT->CYCCNT 快照/统计块
 *     (迭代耗时打点 / 采样段 5 段细分 / Kalman 3 步 / feed_imu 5 段细分 / 导航上报耗时);
 *     navi_bridge.c 删 imu_frame_process_all 内 t0~t5/fdt 及分段峰值统计块,
 *     删 try_run_kalman 内 t0~t3/dt 及分步峰值统计块 + g_kf_run_count++;
 *     删除 navi_bridge.c 仅为 DWT 引入的 gd32f4xx.h (其余依赖 Navi/gnss_hjd 头文件保持)
 *   - 保留的诊断变量 (gnss_hjd.c/.h, 仍为 volatile 全局供 Watch 观察):
 *     g_gnss_frame_count (有效帧累计) / g_gnss_err_count (校验失败累计) /
 *     g_gnss_ovf_count (DMA 环缓冲整圈覆盖, 唯一真正指示数据已丢失的标志)
 *     — 以上三者均被业务逻辑直接引用 (g_gnss_result.frame_count /
 *     parse_frame 校验分支 / pump 内 available>BUF_SIZE 时整圈丢弃), 不是打点;
 *     s_dma_write_laps 为 pump 算 write_total 的核心, DMA1_CH2 FTF ISR 保留
 *   - mat_inverse float 化 (2026-10-28 实测验证):
 *     mat_inverse.c 静态数组 double aug[722] → float aug[722] (ZI -2,888B),
 *     函数参数 double* A,A_inv → float* A,A_inv, 内部临时变量全 float,
 *     fabs → fabsf, 1e-10 → 1e-10f; 高斯消元从软件浮点 __aeabi_dmul/__aeabi_dadd
 *     提速为 Cortex-M4F 硬件 FPU vmla.f32 (单周期 MAC);
 *     math_matrix.h 声明同步 double* → float*;
 *     Kalman15d_MeaUpdate.c 删 double inv_in6/inv_in7 声明 + 6×6/7×7 桥接循环
 *     (每步 72~98 次 float↔double 隐式转换), 直接 mat_inverse(6, mat_temp4, mat_temp5)
 *     / mat_inverse(7, mat_temp14, mat_temp15);
 *     MeaUpdate 实测 1.606ms → 1.066ms (-34%), TimeUpdate 1.697ms 持平 (本就全 float)
 *   - DMA1_CH2 FTF 中断计圈 (2026-10-28 实测验证):
 *     pump 里原靠 transfer_number 回装瞬间轮询 FTF 清圈存在竞态 (临界帧读到不稳定值);
 *     改为 DMA1_Channel2_IRQHandler 每圈写满精确触发, ISR 仅清 FTF + ++s_dma_write_laps
 *     (两条语句, 48.9ms 才触发一次, 零负担);
 *     pump 删除 FTF 轮询块 (8 行), init 末尾增 nvic_irq_enable(DMA1_Channel2_IRQn, 2, 0)
 *     + DMA_CHXCTL_FTFIE; gnss_hjd.c 内部 s_dma_write_laps 从 static 改 volatile,
 *     供 ISR 与 pump 共享; gd32f4xx_it.c 新增 DMA1_Channel2_IRQHandler
 *   - 清理 g_gnss_overtake_count (原 g_gnss_drop_count 语义修正后彻底删除):
 *     单槽邮箱覆盖属正常行为 (消费者只取最新帧), 非零不等于故障;
 *     删 gnss_hjd.c 变量定义 + parse_frame 内 s_result_new 判断 + init 清零行
 *     + gnss_hjd_get_drop_count() 函数 (map 显示已被死代码消除, 无调用者),
 *     删 gnss_hjd.h extern 声明 + 函数声明
 *   - 编译尺寸 (ARMCC V5 -O3 -Otime):
 *     mat_inverse float 化: Code 39,272 → 38,544 (-728B), ZI 58,212 → 55,324 (-2,888B);
 *     FTF 中断计圈: Code +56B (ISR + init 使能), ZI 不变;
 *     全删 DWT 打点: Code 38,648 → 38,264 (-384B), RW 244 → 164 (-80B);
 *     清理 overtake_count: Code 不变, RW 不变, ZI 不变
 *   - 烤机验证 (17.5min / 30min 两轮, 168MHz):
 *     g_loop_over5ms_count = 0 (全链路零吞节拍), g_loop_slow_count = 0;
 *     g_gnss_ovf_count = 0 (原 62min 565 次竞态误报根除), g_gnss_err_count = 2 (0.011%, 原 536/0.56%);
 *     kf_update_max 1.698ms / kf_mes_max 1.066ms / sample_total_max 1.463ms 全部稳定
 *   - 业务行为零改动: 主循环骨架 (pump → sample_due → try_run_kalman → nav_send_frame)
 *     完全保留; SPI/DMA/UART/GNSS 解析/导航融合时序未变; 所有打点变量本身就只写
 *     不读, 删除无功能回归; gnss_hjd 三个保留诊断变量 (frame/err/ovf) 仍被业务引用,
 *     不是打点
 * 2026-10-30 [v1.5.0] 健壮性与可观测性收尾:
 *   - 初始化失败不再从 main() 返回 (裸机下 main 返回属未定义行为, 会跳飞):
 *     改为 main_fault_halt() 记录 g_main_fault_code 后关总中断死循环停机,
 *     故障码非 static, 可直接在 Keil Watch 查看
 *   - SysTick 增加自由运行毫秒计数 g_sys_tick_ms (1ms 递增, 约 49.7 天回绕)
 *   - GNSS 环缓冲溢出时刻快照: g_gnss_ovf_stamp_ms[8] + g_gnss_ovf_stamp_idx,
 *     用于事后区分"调试器 halt 造成"与"主循环真实卡顿"; pump 主路径行为不变
 *   - navi_pipeline_get_state 姿态改由 m2att(g_imu.cnb) 实时反解:
 *     原实现直读 g_imu.att, 而 15 维 Kalman 链路不刷新该数组 (仅粗对准写一次),
 *     上报姿态会停留在对准时刻; 粗对准未完成时仍回退 g_imu.att
 *   - 删除死 API navi_pipeline_get_phase / navi_pipeline_phase_str / navi_phase_t
 *   - 删除 s_gnss2_stub 未使用的 heading_deg 字段
 *   - USART0 配置归属收敛: 波特率/字长/校验/DENT 统一由 UART0_DMA_Init 配置
 *     (230400 8E1), gnss1_rx_init 只追加 RX DMA 通道 + DENR + 环缓冲复位,
 *     不再调用 usart_deinit, 消除重复配置与调用顺序依赖
 *   - 编译尺寸 (ARMCC V5 -O3 -Otime): Code 38,224 → 38,232 (+8B),
 *     RW 164 → 180 (+16B, 故障码/溢出快照), ZI 41,980 → 42,012 (+32B);
 *     0 Error 0 Warning
 */

/* 编译时间由编译器自动填入。 */
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__
#define FW_BUILD_DATETIME __DATE__ " " __TIME__

/* 注: v1.0.6 引入的版本查询协议 (55 AA 01 01 02 0D 0A) 已于 v1.2.0 移除,
 *     当前 USART0 固定用于 GNSS1 数据接收 + 导航结果帧发送。 */

void version_print(void);
const char *version_get_str(void);
const char *version_get_build_date(void);
const char *version_get_build_time(void);

#endif /* __VERSION_H */
