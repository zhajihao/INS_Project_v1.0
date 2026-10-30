/**
 * @file navi_pipeline.c
 * @brief 组合导航流水线编排层
 *
 * 数据流:
 *   feed_imu → GNSS 数据填充 → ErrorComp×2 → 粗对准状态机 → ins_update_RT
 *   run_kalman → GetKalman15d_SSMat → TimeUpdate → MesUpdate → Feedback
 *
 * GNSS 数据填充:
 *   gnss_info.vn1/pos1/flag_GNSS1valid   ← GNSS1 (USART0 真机接收)
 *   gnss_info.vn2/pos2/flag_GNSS2valid   ← GNSS2 (stub, 固定位置, PDOP1.5, 无航向)
 *   gnss_info.GNSStype = 0x0C            ← bit0x08=RTK GNSS1, bit0x04=RTK GNSS2
 */

#include "navi_pipeline.h"

/* ---- navi 内部头文件 ---- */
#include "navi_api.h"
#include "navi_config/navi_constants.h"
#include "navi_config/navi_types.h"
#include "navi_config/GNSS_Infor.h"
#include "navi_core/navi_coarse_align.h"
#include "navi_core/navi_attitude.h"
#include "navi_math/navi_vector.h"
#include "navi_core/navi_ins.h"
#include "navi_core/navi_earth.h"
#include "navi_kalman/kalman15d/kalman15d.h"
#include "navi_compensation/navi_error_comp.h"
#include "gnss1_rx.h"
#include <string.h>
#include <math.h>

/* ---- 全局变量 (全部 static, 外部不可见 — 通过本文件 API 访问) ---- */

/* ---- navi 核心状态 ---- */
static struct Imu_Infor g_imu;
static struct Ethi g_ethi;
static struct Sys_Infor g_sys;
static struct AHRS_Infor g_ahrs;
static struct GNSS_Infor gnss_info;
static struct Kalman15d_SSMat g_k15s;
static struct Imu_TempCompPara imu_temp_para;

/* ---- 粗对准计时 ---- */
static uint32_t calign_frame_count; /* 粗对准累计帧数 */
static uint8_t calign_started;      /* 粗对准已启动标志 */

/* ---- pipeline 内部状态 ---- */
static uint8_t s_initialized;

/* ---- GNSS2 stub (GNSS1 已改为 USART0 真实接收) ---- */

/**
 * @brief  GNSS1 粗对准默认航向 (T_HJD_POSDATA 协议无 heading 字段)
 *
 * 单天线 GNSS 无航向输出, 粗对准航向装订为固定值 (真北为 0°, 取值 180°).
 * 后续接双天线 GNSS 或外部航向源时, 替换此常量为实时值.
 */
#define GNSS1_DEFAULT_HEADING_DEG 180.0

/**
 * @brief  GNSS2 stub 静态数据
 *
 * 固定位置/速度 (独立第二路卫导占位), PDOP 1.5; 单天线无航向字段.
 */
static const struct
{
    double vn_E, vn_N, vn_U;
    double lat_deg, lon_deg, height_m;
    uint8_t status;
    uint8_t sat_num;
    float pdop;
} s_gnss2_stub = {
    0.0, 0.0, 0.0,
    34.2, 108.8, 400.0,
    0x03, /* status: bit0定位有效 bit1速度有效 (单天线无航向) */
    12,
    1.5f /* PDOP_2 */
};

/* ---- 单帧 IMU 处理 (feed_imu 内部调用) ---- */

/**
 * @brief  单帧 IMU 完整流水线: 7 步顺序执行
 *
 * 调用方: navi_pipeline_feed_imu (已把 gyro 从 rad/s 转成 deg/s, accel 保持 m/s²).
 * 内部持有所有 Navi 状态 (g_imu / g_ahrs / gnss_info / ...), 外部不可见.
 *
 * 流水线 7 步:
 *   0. GNSS 数据填充     GNSS1=真机 gnss1_rx (若有新帧则覆盖 vn1/pos1/flag),
 *                        GNSS2=stub (固定位置 34.2°N/108.8°E/400m, 无航向字段)
 *   1. IMU 原始量解包    accel → acc_ori = accel * ts, gyro → gyro_ori = gyro(dps) * arcdeg * ts
 *   2. 温补级            GetErrorPara_TC: T_gyro[0..2] / T_acc[0..2] → errpara_TC
 *   3. ErrorComp ×2      温补级 errpara_TC → 标定级 para_calerr → data_gyro / data_acc
 *   4. GNSS 判定         GetGNSSType + IsGNSSDataValid (判定 GNSStype + flag_GNSS*valid)
 *   5. 粗对准状态机      3 步: 启动触发 → 执行 CoarseAlign_Horizon_RT + 装航向 → 完成判定
 *                        (对准锁存门 flag_Finish_calign = 1 后永远不再进)
 *   6. INS 递推          ins_update_RT 双子样 (仅对准完成后执行, 每 10ms 置 flag_navsuc=1)
 *
 * @param  gyro_x_dps/y/z  机体系角速率 (deg/s, 已由调用方从 rad/s 转换)
 * @param  accel_x_mps2/y/z 机体系加速度 (m/s²)
 */
static void imu_frame_process_all(
    double gyro_x_dps, double gyro_y_dps, double gyro_z_dps,
    double accel_x_mps2, double accel_y_mps2, double accel_z_mps2)
{
    g_imu.flag_ValidIMU = 1;

    /* === 0. GNSS 数据填充 ===
     *
     * GNSS1: 从 gnss1_rx 解析结果取 (USART0 DMA 接收 T_HJD_POSDATA 帧)
     *        有新帧则更新 vn1/pos1/flag_GNSS1valid/PDOP_1/sat_num1
     *        无新帧保持上一帧值
     * GNSS2: 暂用 stub (后续可接第二路卫导) */

    /* ---- GNSS1 (从 gnss1_rx 取) ---- */
    {
        gnss1_rx_result_t gnss1;
        if (gnss1_rx_get_result(&gnss1))
        {
            gnss_info.vn1[0] = gnss1.vn_e;
            gnss_info.vn1[1] = gnss1.vn_n;
            gnss_info.vn1[2] = gnss1.vn_u;
            gnss_info.pos1[0] = gnss1.lat_rad;
            gnss_info.pos1[1] = gnss1.lon_rad;
            gnss_info.pos1[2] = gnss1.height_m;
            gnss_info.flag_GNSS1valid = gnss1.pos_valid;
            gnss_info.PDOP_1 = gnss1.pdop;
            gnss_info.sat_num1 = gnss1.sat_num;
        }
        gnss_info.GNSStype = 0x0C; /* bit0x08=RTK1 */
    }

    /* ---- GNSS2 (stub) ---- */
    gnss_info.vn2[0] = s_gnss2_stub.vn_E;
    gnss_info.vn2[1] = s_gnss2_stub.vn_N;
    gnss_info.vn2[2] = s_gnss2_stub.vn_U;
    gnss_info.pos2[0] = s_gnss2_stub.lat_deg * arcdeg;
    gnss_info.pos2[1] = s_gnss2_stub.lon_deg * arcdeg;
    gnss_info.pos2[2] = s_gnss2_stub.height_m;
    gnss_info.flag_GNSS2valid = (s_gnss2_stub.status & 0x01u) ? 1u : 0u;
    gnss_info.PDOP_2 = s_gnss2_stub.pdop;
    gnss_info.sat_num2 = s_gnss2_stub.sat_num;

    /* === 1. IMU 原始帧解包 === */
    g_imu.gyro_ori[0] = gyro_x_dps * arcdeg * g_imu.ts;
    g_imu.gyro_ori[1] = gyro_y_dps * arcdeg * g_imu.ts;
    g_imu.gyro_ori[2] = gyro_z_dps * arcdeg * g_imu.ts;
    g_imu.acc_ori[0] = accel_x_mps2 * g_imu.ts;
    g_imu.acc_ori[1] = accel_y_mps2 * g_imu.ts;
    g_imu.acc_ori[2] = accel_z_mps2 * g_imu.ts;
    /* T_gyro[0..2] / T_acc[0..2] 由 pipeline_feed_imu 提前写入 */

    /* === 2. 温补级: GetErrorPara_TC === */
    GetErrorPara_TC(&imu_temp_para, &g_imu);

    /* === 3. ErrorComp ×2 级 (温补 + 标定) === */
    ErrorComp(g_imu.errpara_TC,
              g_imu.gyro_ori, g_imu.acc_ori,
              g_imu.gyro_TC, g_imu.acc_TC,
              g_imu.ts);
    ErrorComp(g_imu.para_calerr,
              g_imu.gyro_TC, g_imu.acc_TC,
              g_imu.data_gyro, g_imu.data_acc,
              g_imu.ts);

    /* === 4. GNSS 类型判定 + 有效性检查 === */
    GetGNSSType(&gnss_info, &g_sys);
    IsGNSSDataValid(&gnss_info, &g_imu, &g_sys);

    /* === 5. 粗对准状态机 ===
     *
     * 步骤1 触发条件: (未启动) && (IMU有效) && (对准从未完成过)
     *   flag_Finish_calign 是锁存门, 完成后步骤 1 永远不再进 */

    /* 步骤1: 启动触发 */
    if (calign_started == 0 && g_imu.flag_ValidIMU && g_sys.flag_Finish_calign == 0)
    {
        g_ahrs.Flag_HorAlignFinish = 0;
        calign_frame_count = 0;
        calign_started = 1;
    }

    /* 步骤2: 执行水平粗对准 + 固定航向装订 (GNSS1_DEFAULT_HEADING_DEG) */
    if (calign_started && (g_ahrs.Flag_HorAlignFinish == 0))
    {
        CoarseAlign_Horizon_RT(&g_imu, &g_ahrs);
        /* T_HJD_POSDATA 无 heading 字段, 用默认航向 180°
         * 对应公式: att_align[2] = (90.0 - heading) * arcdeg */
        g_ahrs.att_align[2] = (90.0 - GNSS1_DEFAULT_HEADING_DEG) * arcdeg;
        calign_frame_count++;
    }

    /* 步骤3: 完成判定
     *   计时达 t_align*fs 且 环境因子=1 → 拷贝对准结果, 装订卫导位置/速度 */
    if (calign_frame_count >= (uint32_t)(g_sys.t_align * g_imu.fs) && g_ahrs.flag_EnvFactor == 1)
    {
        vect3d_assign(g_ahrs.att_align[0], g_ahrs.att_align[1], g_ahrs.att_align[2],
                      g_imu.att);
        a2mat(g_imu.att, g_imu.cnb);
        /* 位置/速度一次性装订 (用当前最新 GNSS1 值) */
        g_imu.vn[0] = gnss_info.vn1[0];
        g_imu.vn[1] = gnss_info.vn1[1];
        g_imu.vn[2] = gnss_info.vn1[2];
        g_imu.pos[0] = gnss_info.pos1[0];
        g_imu.pos[1] = gnss_info.pos1[1];
        g_imu.pos[2] = gnss_info.pos1[2];

        g_ahrs.Flag_HorAlignFinish = 1;
        g_sys.flag_Finish_calign = 1;
        calign_started = 0;
        calign_frame_count = 0;
    }
    else if (calign_frame_count >= (uint32_t)(g_sys.t_align * g_imu.fs) && g_ahrs.flag_EnvFactor == 0)
    {
        g_ahrs.Flag_HorAlignFinish = 0;
        calign_started = 0;
        calign_frame_count = 0;
    }

    /* === 6. 对准完成进入惯性导航 === */
    if (g_ahrs.Flag_HorAlignFinish)
    {
        ins_update_RT(&g_imu, &g_ethi, &g_sys);
    }
}

/**
 * @brief  15d Kalman 时间更新 (内部调用)
 *
 * 两步: GetKalman15d_SSMat (装订 F/Q/H/R 系统矩阵) → Kalman15d_TimeUpdate (预测步骤).
 * 执行前清零 flag_navsuc, 避免同一帧被 run_kalman 重复消费.
 *
 * @param  imu   [in/out] IMU 状态 (含 vn/pos/att, 用于 SSMat 装订)
 * @param  sys   [in/out] 系统状态 (含 ts/fs, 用于 SSMat 装订)
 * @param  k15s  [out]  15d Kalman 状态空间矩阵 (Pkk_1 / Phikk / Qh)
 * @param  ethi  [in] 地球参数 (RMh/clRNh, 用于 SSMat 装订)
 */
static void kalman15d_update(struct Imu_Infor *imu, struct Sys_Infor *sys,
                             struct Kalman15d_SSMat *k15s, struct Ethi *ethi)
{
    imu->flag_navsuc = 0;
    GetKalman15d_SSMat(imu, k15s, ethi);
    Kalman15d_TimeUpdate(k15s);
}

/* ---- 对外 API ---- */

/**
 * @brief  初始化组合导航流水线
 *
 * 顺序: memset 清零所有状态 → navi_init (内部装订默认参数 + earth() + Kalman15d_Init)
 *        → para_calerr 重新装订 (陀螺 [9..11]=1, 加表 [24..26]=1)
 *        → GNSS 初始值 (GNSS1 清零, GNSS2 从 stub 拷贝).
 *
 * @note   依赖顺序: 必须在所有传感器 init 之后调用 —— navi_init 内部装订 ts=0.005/fs=200
 *         但 para_calerr 等参数需由流水线覆写. GNSStype 初始装订 0x0C (RTK1) 走融合分支.
 */
void navi_pipeline_init(void)
{
    /* 1. 清零所有结构体 */
    memset(&g_imu, 0, sizeof(g_imu));
    memset(&g_ethi, 0, sizeof(g_ethi));
    memset(&g_sys, 0, sizeof(g_sys));
    memset(&g_ahrs, 0, sizeof(g_ahrs));
    memset(&gnss_info, 0, sizeof(gnss_info));
    memset(&g_k15s, 0, sizeof(g_k15s));
    memset(&imu_temp_para, 0, sizeof(imu_temp_para));

    calign_frame_count = 0;
    calign_started = 0;
    s_initialized = 0;

    /* 2. navi_init
     *    内部装订: pos=45°N/108°E/0m, nn=2, nts=0.01, ts=0.005, fs=200Hz
     *              t_align=30s, earth(), Kalman15d_Init()
     *    k19/k19s 传 NULL: 本固件走 15 维 Kalman, navi_init 内部对二者仅 (void) 忽略 */
    navi_init(&g_imu, &g_ethi, &g_sys, NULL, NULL, &g_k15s);

    /* 3. para_calerr 重新装订
     *    navi_init 内部装订了默认值, 随后被 memset 清零, 再按新布局装订 */

    /* 3a. 温补多项式系数: 6 轴标度常数项 a0=1 */
    imu_temp_para.aT_E1x[0] = 1.0; /* X 陀螺 */
    imu_temp_para.aT_E1y[0] = 1.0; /* Y 陀螺 */
    imu_temp_para.aT_E1z[0] = 1.0; /* Z 陀螺 */
    imu_temp_para.aT_K1x[0] = 1.0; /* X 加表 */
    imu_temp_para.aT_K1y[0] = 1.0; /* Y 加表 */
    imu_temp_para.aT_K1z[0] = 1.0; /* Z 加表 */

    /* 3b. 标定级 para_calerr 清零后装订
     *     [9,10,11] = 陀螺 E1g 标度恒等
     *     [24,25,26] = 加表 K1a 标度恒等 */
    memset(&g_imu.para_calerr, 0, sizeof(g_imu.para_calerr));
    g_imu.para_calerr[9] = 1.0;
    g_imu.para_calerr[10] = 1.0;
    g_imu.para_calerr[11] = 1.0;
    g_imu.para_calerr[24] = 1.0;
    g_imu.para_calerr[25] = 1.0;
    g_imu.para_calerr[26] = 1.0;

    /* 4. GNSS1 初始清零 (等 USART0 收到第一帧 T_HJD_POSDATA 后由 gnss1_rx 更新)
     *    GNSStype 仍设 0x0C 以走 RTK 分支 */
    gnss_info.vn1[0] = 0.0;
    gnss_info.vn1[1] = 0.0;
    gnss_info.vn1[2] = 0.0;
    gnss_info.pos1[0] = 0.0;
    gnss_info.pos1[1] = 0.0;
    gnss_info.pos1[2] = 0.0;
    gnss_info.flag_GNSS1valid = 0;
    gnss_info.GNSStype = 0x0C;

    gnss_info.vn2[0] = s_gnss2_stub.vn_E;
    gnss_info.vn2[1] = s_gnss2_stub.vn_N;
    gnss_info.vn2[2] = s_gnss2_stub.vn_U;
    gnss_info.pos2[0] = s_gnss2_stub.lat_deg * arcdeg;
    gnss_info.pos2[1] = s_gnss2_stub.lon_deg * arcdeg;
    gnss_info.pos2[2] = s_gnss2_stub.height_m;
    gnss_info.flag_GNSS2valid = (s_gnss2_stub.status & 0x01u) ? 1u : 0u;

    s_initialized = 1;
}

/**
 * @brief  喂一帧 IMU + 温度 (每 5ms 调一次, 物理采样率 200Hz)
 *
 * 内部走完整 7 步流水线 (GNSS 填充 → 解包 → 温补 → ErrorComp×2 → GNSS 判定 → 粗对准 → INS).
 * 粗对准完成前只跑到步骤 5, 对准完成后跑步骤 6 ins_update_RT 双子样并置 flag_navsuc.
 *
 * @param  accel_mps2[3]   导航系加速度 [x=东, y=北, z=天] (m/s², float)
 * @param  gyro_radps[3]   导航系角速率 [x=东, y=北, z=天] (rad/s, float, 内部转 deg/s 进 Navi)
 * @param  gyro_temp_c[3]  三轴陀螺温度 (°C)
 * @param  accel_temp_c    加计温度 (°C)
 * @note   gyro 单位转换: Navi 算法包期望 gyro_ori 单位为 deg/s × arcdeg × ts (= rad),
 *         MCU 输出 rad/s, 故先除以 arcdeg 得到 deg/s, 再传进 imu_frame_process_all.
 */
void navi_pipeline_feed_imu(const float accel_mps2[3], const float gyro_radps[3],
                            const float gyro_temp_c[3], float accel_temp_c)
{
    if (!s_initialized)
        return;

    /* 每帧写 T_gyro/T_acc —— GetErrorPara_TC 内部用它算温度多项式 */
    g_imu.T_gyro[0] = gyro_temp_c[0];
    g_imu.T_gyro[1] = gyro_temp_c[1];
    g_imu.T_gyro[2] = gyro_temp_c[2];
    g_imu.T_acc[0] = accel_temp_c;
    g_imu.T_acc[1] = accel_temp_c;
    g_imu.T_acc[2] = accel_temp_c;

    /* imu_frame_process_all 期望 gyro 单位为 deg/s,
     * MCU 输入是 rad/s, 先转 deg/s = rad_s * (1/arcdeg) */
    double gyro_x_dps = gyro_radps[0] * (1.0 / arcdeg);
    double gyro_y_dps = gyro_radps[1] * (1.0 / arcdeg);
    double gyro_z_dps = gyro_radps[2] * (1.0 / arcdeg);

    imu_frame_process_all(gyro_x_dps, gyro_y_dps, gyro_z_dps,
                          accel_mps2[0], accel_mps2[1], accel_mps2[2]);
}

/**
 * @brief  跑一次 15d Kalman 全链路 (每 10ms 调一次)
 *
 * 守卫条件: flag_navsuc==0 返回 0 (没跑, INS 尚未准备好或刚被 update 清零).
 * 内部 3 步:
 *   kalman15d_update      GetKalman15d_SSMat (F/Q/H/R 系统矩阵装订) → TimeUpdate (预测步骤)
 *   → Kalman15d_MesUpdate 含 mat_inverse(6×6 / 7×7) 观测更新
 *   → Kalman_Feedback     反馈校正后验状态到 g_imu
 *
 * @retval 1 本次跑了 Kalman (有新导航结果输出)
 * @retval 0 没跑 (flag_navsuc==0)
 * @note   调用时机: 在 feed_imu 之后、主循环外层同层级 (不在 sample_due 分支内).
 *         main.c 按 navi_push_div >= 10U 节流上报: 本函数返回 1 后
 *         navi_pipeline_get_state + nav_send_frame 才取融合后的 pos/vn/att
 *         (返回值本身在上报分支中不使用).
 */
uint8_t navi_pipeline_run_kalman(void)
{
    if (!s_initialized)
        return 0;

    if (!g_imu.flag_navsuc)
        return 0;

    kalman15d_update(&g_imu, &g_sys, &g_k15s, &g_ethi);
    Kalman15d_MesUpdate(&g_k15s, &gnss_info, &g_imu, &g_sys);
    Kalman_Feedback(&g_k15s, &g_sys, &g_imu);

    return 1;
}

/* ---- 对外查询接口 ---- */

/**
 * @brief  查询导航状态量 (att/vn/pos 全部为最新值)
 *
 * att 由 g_imu.cnb 经 m2att 实时反解 (顺序 [pitch, roll, yaw] rad, 与导航帧一致);
 * 仅在粗对准尚未完成时回退到 g_imu.att (此时 cnb 还未进入惯导递推).
 * vn 取自 g_imu.vn[3] (m/s 导航系), pos 取自 g_imu.pos[3] ([纬度rad, 经度rad, 高度m]).
 * 任一参数传 NULL 则跳过该组拷贝, 调用方可按需取子集.
 *
 * @param  att_rad[3]   [out] 姿态角 [俯仰, 横滚, 航向] (rad), NULL 可跳过
 * @param  vn_mps[3]    [out] 导航系速度 [东, 北, 天] (m/s), NULL 可跳过
 * @param  pos[3]       [out] 位置 [纬度(rad), 经度(rad), 高度(m)], NULL 可跳过
 * @note   15 维 Kalman 链路不刷新 g_imu.att (只有粗对准与 7d/12d 链路写它),
 *         故此处必须用 m2att(cnb) 反解, 否则上报的姿态会停留在粗对准时刻.
 */
void navi_pipeline_get_state(double att_rad[3], double vn_mps[3], double pos[3])
{
    if (att_rad)
    {
        if (g_sys.flag_Finish_calign != 0)
        {
            /* cnb 已由 ins_update_RT/Kalman_Feedback 持续修正, 直接反解姿态 */
            m2att(g_imu.cnb, att_rad);
        }
        else
        {
            att_rad[0] = g_imu.att[0];
            att_rad[1] = g_imu.att[1];
            att_rad[2] = g_imu.att[2];
        }
    }
    if (vn_mps)
    {
        vn_mps[0] = g_imu.vn[0];
        vn_mps[1] = g_imu.vn[1];
        vn_mps[2] = g_imu.vn[2];
    }
    if (pos)
    {
        pos[0] = g_imu.pos[0];
        pos[1] = g_imu.pos[1];
        pos[2] = g_imu.pos[2];
    }
}
