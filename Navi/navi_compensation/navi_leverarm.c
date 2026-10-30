/**
 * @file navi_leverarm.c
 * @brief 杆臂效应补偿
 *
 * 内杆臂：补偿 IMU 内部加速度计相对陀螺的杆臂误差；
 * 外杆臂：补偿 GNSS 天线相对 IMU 的杆臂误差。
 */

#include "navi_leverarm.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include <math.h>

/**
 * @brief  内杆臂补偿:由相邻角增量差分求角加速度,按向心项(ω×ω)与切向项(角加速度)计算加表内杆臂引入的速度增量误差并从dvbm扣除
 * @param  p_imu_infor IMU信息:输入phim/phimk_1(rad)、wbeb(rad/s)、r_inlearm内杆臂(m,9元素)、nts(s);输出补偿后的dvbm(m/s)
 * @retval void
 * @note   角加速度dwbeb=(phim-phimk_1)/nts^2(rad/s^2);dfbsf为ω、dwbeb与杆臂构成的比力补偿量(m/s^2),dvbm=dvbm-dfbsf*nts
 */
void LeverArm_Inner(struct Imu_Infor *p_imu_infor)
{
	double dwbeb[3] = {0};

	double v_temp1[3] = {0}, v_temp2[3] = {0}, v_temp3[3] = {0}, dfbsf[3] = {0};

	double wbebx2 = 0, wbeby2 = 0, wbebz2 = 0, wbebxy = 0, wbebxz = 0, wbebyz = 0;
	double a11 = 0, a12 = 0, a13 = 0, a21 = 0, a22 = 0, a23 = 0, a31 = 0, a32 = 0, a33 = 0;

	// 计算角加速度矢量
	mat_minus((*p_imu_infor).phim, (*p_imu_infor).phimk_1, v_temp1, 3, 1); // v_temp1=phim-phimk_1
	mat_multi_num(v_temp1, 1.0 / (*p_imu_infor).nts, v_temp2, 3, 1);	   // v_temp2=(phim-phimk_1)/nts
	mat_multi_num(v_temp2, 1.0 / (*p_imu_infor).nts, dwbeb, 3, 1);		   // dwbeb=(phim-phimk_1)/nts/nts

	// 计算加速度补偿量

	wbebx2 = pow((*p_imu_infor).wbeb[0], 2); // wbebx2=wbeb[0]^2
	wbeby2 = pow((*p_imu_infor).wbeb[1], 2); // wbeby2=wbeb[1]^2
	wbebz2 = pow((*p_imu_infor).wbeb[2], 2); // wbebz2=wbeb[2]^2

	wbebxy = (*p_imu_infor).wbeb[0] * (*p_imu_infor).wbeb[1]; // wbebxy =wbeb[0] * wbeb[1]
	wbebxz = (*p_imu_infor).wbeb[0] * (*p_imu_infor).wbeb[2]; // wbebxz =wbeb[0] * wbeb[2]
	wbebyz = (*p_imu_infor).wbeb[1] * (*p_imu_infor).wbeb[2]; // wbebyz =wbeb[1] * wbeb[2]

	a11 = -wbeby2 - wbebz2;
	a12 = -dwbeb[2] + wbebxy;
	a13 = -dwbeb[1] + wbebxz;

	a21 = dwbeb[2] + wbebxy;
	a22 = -wbebz2 - wbebx2;
	a23 = -dwbeb[0] + wbebyz;

	a31 = -dwbeb[1] + wbebxz;
	a32 = dwbeb[0] + wbebyz;
	a33 = -wbebx2 - wbeby2;

	dfbsf[0] = a11 * (*p_imu_infor).r_inlearm[0] + a12 * (*p_imu_infor).r_inlearm[1] + a13 * (*p_imu_infor).r_inlearm[2];
	dfbsf[1] = a21 * (*p_imu_infor).r_inlearm[3] + a22 * (*p_imu_infor).r_inlearm[4] + a23 * (*p_imu_infor).r_inlearm[5];
	dfbsf[2] = a31 * (*p_imu_infor).r_inlearm[6] + a32 * (*p_imu_infor).r_inlearm[7] + a33 * (*p_imu_infor).r_inlearm[8];

	mat_multi_num(dfbsf, (*p_imu_infor).nts, v_temp3, 3, 1);
	mat_minus((*p_imu_infor).dvbm, v_temp3, (*p_imu_infor).dvbm, 3, 1);
}

/**
 * @brief  外杆臂(GNSS空间杆臂)补偿:计算外杆臂引起的速度测量误差dvnL与位置测量误差dpGL,供GNSS量测补偿
 * @param  p_imu_infor  IMU信息:输入wbeb(rad/s)、cnb
 * @param  p_sys_infor  系统信息:输入r_outlearm外杆臂(m);输出dvnL(m/s)、dpGL([纬度增量rad,经度增量rad,高度增量m])
 * @param  p_ethi       地球参数(RMh/clRNh)
 * @retval void
 * @note   dvnL=-cnb*(wbeb×r_outlearm);位置项代码先算cnb*r_outlearm随即被Mpv*r_outlearm覆盖,故dpGL实际=-Mpv*r_outlearm,Mpv仅含1/RMh、1/clRNh的速度-位置增量转换
 */
void LeverArm_Spatial(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi)
{

	double Mpv[3][3] = {0};
	double mat_temp1[3][3] = {0};
	double v_temp1[3] = {0}, v_temp2[3] = {0}, v_temp3[3] = {0};

	// 计算需要的Mpv矩阵
	Mpv[0][1] = 1.0 / (*p_ethi).RMh;
	Mpv[1][0] = 1.0 / (*p_ethi).clRNh;
	Mpv[2][2] = 1.0;

	// 外杆臂误差造成的速度和位置测量误差
	vect3d_askew((*p_imu_infor).wbeb, mat_temp1);						  // mat_temp1=wbeb*
	mat_multi(mat_temp1[0], (*p_sys_infor).r_outlearm, v_temp1, 3, 3, 1); // v_temp1=cross(wbeb,r_outlearm)
	mat_multi((*p_imu_infor).cnb[0], v_temp1, v_temp2, 3, 3, 1);		  // v_temp2=cnb*cross(wbeb,r_outlearm)
	mat_multi_num(v_temp2, -1.0, (*p_sys_infor).dvnL, 3, 1);			  // dvnL=-cnb*cross(wbeb,r_outlearm)

	mat_multi((*p_imu_infor).cnb[0], (*p_sys_infor).r_outlearm, v_temp3, 3, 3, 1); // v_temp3=cnb*r_outlearm (下一行即被覆盖)
	mat_multi(Mpv[0], (*p_sys_infor).r_outlearm, v_temp3, 3, 3, 1);				   // v_temp3=Mpv*r_outlearm (覆盖上一行)
	mat_multi_num(v_temp3, -1.0, (*p_sys_infor).dpGL, 3, 1);					   // dpGL=-Mpv*r_outlearm
}
