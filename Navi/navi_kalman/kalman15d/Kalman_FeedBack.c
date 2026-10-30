/**
 * @file Kalman_FeedBack.c
 * @brief 15维卡尔曼滤波闭环反馈实现
 *
 * 将滤波估计出的速度误差、位置误差反馈到惯导递推状态中，
 * 实现 INS/GNSS 组合导航的误差校正闭环。
 *
 * @note 当前仅反馈 xk[3..8](速度+位置)，失准角和零偏未反馈。
 */

#include "navi_config/navi_types.h"
#include <math.h>
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"
#include "navi_math/math_matrix.h"
#include "Kalman15d_SSMat.h"

/**
 * @brief 卡尔曼滤波结果反馈到惯导状态
 *
 * 仅在量测更新成功(flag_KalmanSuc=1)时执行：
 * 1. 从速度 vn 中扣除估计的速度误差 xk[3..5]
 * 2. 从位置 pos 中扣除估计的位置误差 xk[6..8]
 * 3. 清零对应状态分量，防止重复反馈
 *
 * @param[out] p_Kalman15d_SSMat  15维滤波器状态矩阵(读写)
 * @param[out] p_sys_infor        系统信息(读写，清零 flag_KalmanSuc)
 * @param[out] p_imu_infor        惯导数据(读写，修正 vn/pos)
 *
 * @note 反馈后 flag_KalmanSuc 清零，等待下一次量测更新置位。
 */
void Kalman_Feedback(struct Kalman15d_SSMat *p_Kalman15d_SSMat, struct Sys_Infor *p_sys_infor, struct Imu_Infor *p_imu_infor)
{
	if ((*p_sys_infor).flag_KalmanSuc == 1)
	{

		/* 滤波成功标志清零，防止重复反馈 */
		(*p_sys_infor).flag_KalmanSuc = 0;
		

		/* 速度误差反馈：vn = vn - δvn */
		(*p_imu_infor).vn[0] = (*p_imu_infor).vn[0] - (*p_Kalman15d_SSMat).xk[3];
		(*p_imu_infor).vn[1] = (*p_imu_infor).vn[1] - (*p_Kalman15d_SSMat).xk[4];
		(*p_imu_infor).vn[2] = (*p_imu_infor).vn[2] - (*p_Kalman15d_SSMat).xk[5];

		/* 位置误差反馈：pos = pos - δpos */
		(*p_imu_infor).pos[0] = (*p_imu_infor).pos[0] - (*p_Kalman15d_SSMat).xk[6];
		(*p_imu_infor).pos[1] = (*p_imu_infor).pos[1] - (*p_Kalman15d_SSMat).xk[7];
		(*p_imu_infor).pos[2] = (*p_imu_infor).pos[2] - (*p_Kalman15d_SSMat).xk[8];

		/* 清零已反馈的状态分量 */
		(*p_Kalman15d_SSMat).xk[3] = 0;
		(*p_Kalman15d_SSMat).xk[4] = 0;
		(*p_Kalman15d_SSMat).xk[5] = 0;
		(*p_Kalman15d_SSMat).xk[6] = 0;
		(*p_Kalman15d_SSMat).xk[7] = 0;
		(*p_Kalman15d_SSMat).xk[8] = 0;
	}
}
