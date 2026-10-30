/**
 * @file navi_api.c
 * @brief 组合导航对外统一接口实现
 *
 * 从原 navigation/main.c 的初始化段与主循环体提取,
 * 移除 PC 端 fopen/printf,IMU 数据改由上层填充 imu->gyro_ori/acc_ori。
 */
#include "navi_api.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_vector.h"             /* vect3d_assign */
#include "navi_core/navi_earth.h"              /* earth */
#include "navi_compensation/navi_error_comp.h" /* ErrorComp_Cali */
#include "navi_core/navi_coarse_align.h"       /* coarsealign_inertialRT */
#include "navi_kalman/navi_kalman7d.h"         /* PreciseAlign_Kalman7d */
#include "navi_kalman/navi_kalman19d.h"        /* nav_integ */
#include "navi_kalman/kalman15d/kalman15d.h"   /* Kalman15d_Init (内部已 include Kalman15d_SSMat.h) */

/**
 * @brief 组合导航初始化
 *
 * 完成惯导初始位置装订、采样参数配置、地球参数计算、
 * 15维卡尔曼滤波器状态初始化。
 *
 * @param[out] imu   惯导数据结构体(初始位置/采样率/标定参数)
 * @param[out] ethi  地球参数结构体(由 earth() 填充)
 * @param[out] sys   系统信息结构体(对准时间等)
 * @param[in]  k19   19维滤波器状态(保留，内部自初始化)
 * @param[in]  k19s  19维滤波器启动器(保留)
 * @param[out] k15   15维滤波器状态(由 Kalman15d_Init 初始化)
 *
 * @note 初始位置硬编码为 45°N/108°E/0m，实际应用应由 GNSS 装订。
 */
void navi_init(struct Imu_Infor *imu,
			   struct Ethi *ethi,
			   struct Sys_Infor *sys,
			   struct Kalman19d_State *k19,
			   struct Kalman19d_Starter *k19s,
			   struct Kalman15d_SSMat *k15)
{
	/* 组合导航 19 维滤波器在 nav_integ 内部按 flag_NavIntegInit 初始化,
	   此处仅保留指针供 navi_run_one_frame 使用。 */
	(void)k19;
	(void)k19s;

	/* 位置:纬度 45°、经度 108°、高度 0 */
	vect3d_assign(45 * arcdeg, 108 * arcdeg, 0, (*imu).pos);

	/* 采样配置:fs=200Hz(物理采样不变),双子样(nn=2),
	   nts=ts×nn=0.01s(导航 100Hz) */
	(*imu).fs = 200;
	(*imu).nn = 2;
	(*imu).nts = 0.01;
	(*imu).ts = 0.005;

	/* 温度补偿测试场景: 写入 6 个标度因数占位值(等价于不补偿).
	   注: 按 ErrorComp 打包布局, [0]/[1]/[2] 并非陀螺 dEg 对角([0,4,8] 才是),
	   [12..14] 实为陀螺零偏 eb; 且此处写入的初值会被 navi_pipeline_init()
	   的 memset(&g_imu.para_calerr) 清零, 再按 [9..11]=陀螺 E1g / [24..26]=加表 K1a
	   重新装订, 故实际生效的是流水线里那组. */
	(*imu).para_calerr[0] = 1.0;
	(*imu).para_calerr[1] = 1.0;
	(*imu).para_calerr[2] = 1.0;
	(*imu).para_calerr[12] = 1.0;
	(*imu).para_calerr[13] = 1.0;
	(*imu).para_calerr[14] = 1.0;

	/* 对准时间: 30s (粗对准时长). 本工程粗对准终点由 navi_pipeline 判定
	   (calign_frame_count >= t_align * fs, 见 navi_pipeline.c),
	   CoarseAlign_Horizon_RT 本身不接收 sys 结构体. */
	(*sys).t_align = 30.0;

	/* 计算地球参数 */
	earth(imu, ethi);
	// 15维卡尔曼滤波状态初始化
	Kalman15d_Init(k15, (*imu).nts);
}

/**
 * @brief 组合导航单帧处理
 *
 * 三阶段状态机：
 * 1. 粗对准(惯性基座对准) → flag_Finish_calign=1 后退出
 * 2. 精对准(7维卡尔曼)   → flag_Finish_palign=1 后退出
 * 3. 组合导航(19维卡尔曼) → 持续运行
 *
 * @param[in,out] imu   惯导数据(上层已填充 gyro_ori/acc_ori)
 * @param[in]     ethi  地球参数
 * @param[in,out] sys   系统信息(阶段标志)
 * @param[in]     k19   19维滤波器状态(阶段3使用)
 * @param[in]     k19s  19维滤波器启动器(阶段3使用)
 * @param[in]     k7    7维滤波器状态(阶段2使用)
 * @param[in]     k7s   7维滤波器启动器(阶段2使用)
 *
 * @note 每帧调用一次，采样率 200Hz(nts=0.01s)。
 */
void navi_run_one_frame(struct Imu_Infor *imu,
						struct Ethi *ethi,
						struct Sys_Infor *sys,
						struct Kalman19d_State *k19,
						struct Kalman19d_Starter *k19s,
						struct Kalman7d_State *k7,
						struct Kalman7d_Starter *k7s)
{
	/* 上层已填充本帧 imu->gyro_ori / acc_ori */
	(*imu).flag_ValidIMU = 1;

	/* 误差补偿(标定/温度补偿) */
	//    ErrorComp_Cali(imu);

	if ((*sys).flag_Finish_calign == 0)
	{
		/* 阶段1:粗对准(惯性基座对准,实时版) */
		coarsealign_inertialRT(imu, ethi, sys);
	}
	else if ((*sys).flag_Finish_palign == 0)
	{
		/* 阶段2:精对准(7 维卡尔曼)——需上层传入有效的 k7/k7s */
		if (k7 != 0 && k7s != 0)
		{
			PreciseAlign_Kalman7d(imu, ethi, k7s, k7, sys);
		}
	}
	else
	{
		/* 阶段3:组合导航(19 维卡尔曼)——需上层传入有效的 k19/k19s */
		if (k19 != 0 && k19s != 0)
		{
			nav_integ(imu, ethi, sys, k19, k19s);
		}
	}
}
