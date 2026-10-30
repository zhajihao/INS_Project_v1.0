/**
 * @file navi_time_sync.c
 * @brief IMU/GNSS 时间不同步补偿
 *
 * 按时间差对运动状态一阶外推，解算时间不同步引入的速度和位置误差。
 */

#include "navi_time_sync.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include <math.h>

/**
 * @brief  IMU/GNSS时间不同步补偿:按时间差deltat对运动状态一阶外推,解算时间不同步引入的速度误差dvndt与位置误差dpdt
 * @param  p_imu_infor  IMU信息:输入an(加速度m/s^2)、vn(速度m/s)
 * @param  p_sys_infor  系统信息:输入deltat时间不同步量(s);输出dvndt(m/s)、dpdt([纬度增量rad,经度增量rad,高度增量m])
 * @param  p_ethi       地球参数(RMh/clRNh)
 * @retval void
 * @note   dvndt=an*deltat;dpdt=Mpv*vn*deltat,Mpv为速度-位置增量转换矩阵(1/RMh、1/clRNh)
 */
void TimeAsync_ImuGNSS(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi)
{

	double Mpv[3][3] = {0};
	double v_temp1[3] = {0};

	// 计算需要的Mpv矩阵
	Mpv[0][1] = 1.0 / (*p_ethi).RMh;
	Mpv[1][0] = 1.0 / (*p_ethi).clRNh;
	Mpv[2][2] = 1.0;

	// 计算时间不同步造成的速度和位置误差
	mat_multi_num((*p_imu_infor).an, (*p_sys_infor).deltat, (*p_sys_infor).dvndt, 3, 1); // dvndt=an*deltat

	mat_multi_num((*p_imu_infor).vn, (*p_sys_infor).deltat, v_temp1, 3, 1); // v_temp1=vn*deltat
	mat_multi(Mpv[0], v_temp1, (*p_sys_infor).dpdt, 3, 3, 1);				// dpdt=Mpv*vn*deltat
}
