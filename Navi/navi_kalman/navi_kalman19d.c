/**
 * @file navi_kalman19d.c
 * @brief 19维卡尔曼滤波器实现（组合导航阶段）
 *
 * 状态量：失准角3 + 速度误差3 + 位置误差3 + 陀螺零偏3 + 加表零偏3
 *         + 外杆臂误差3 + 时间不同步1
 * 用于 INS/GNSS 松组合导航，持续估计并反馈各类误差。
 */

#include "navi_kalman19d.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include "navi_core/navi_ins.h"
#include "navi_compensation/navi_leverarm.h"
#include "navi_compensation/navi_time_sync.h"
#include <math.h>

/**
 * @brief  19维组合导航主滤波器初始化：装订Pk/qk/rk初值，xk清零
 * @param  p_Kalman19d_Starter 滤波器初始参数：各状态标准差、随机游走系数、量测噪声
 * @param  p_Kalman19d_State   滤波器状态结构体（出参）：写入xk、Pk、qk、rk
 * @param  p_imu_infor         IMU信息，取nts（s）作为离散时间间隔
 * @param  p_sys_infor         系统信息
 * @retval void
 * @note   19维状态xk=[失准角3(rad)、速度误差3(m/s)、位置误差3(纬经rad/高度m)、
 *         陀螺零偏3(rad/s)、加表零偏3(m/s²)、外杆臂误差3(m)、时间不同步1(s)]；
 *         Pk按各标准差平方装订对角阵；qk=噪声系数^2*nts；
 *         rk 6维分块对角：速度3(m/s)、水平位置2(rad)、高度1(m)
 */
void Kalman19d_Init(struct Kalman19d_Starter *p_Kalman19d_Starter, struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	(*p_Kalman19d_Starter).pk_phipr = 0.5 * arcdeg; // 水平失准角标准差初值
	(*p_Kalman19d_Starter).pk_phiy = 5 * arcdeg;	// 航向失准角标准差初值
	(*p_Kalman19d_Starter).pk_dvn = 0.1;			// 速度误差标准差初值
	(*p_Kalman19d_Starter).pk_dposLL = 2.0e-7;		// 水平位置误差标准差初值
	(*p_Kalman19d_Starter).pk_dposH = 10;			// 高度位置误差标准差初值
	(*p_Kalman19d_Starter).pk_eb = 0.5 * dph;		// 陀螺常值漂移标准差初值
	(*p_Kalman19d_Starter).pk_db = 1.0e3 * ug;		// 加表零偏标准差初值
	(*p_Kalman19d_Starter).pk_dl = 0.5;				// 外杆臂误差标准差初值
	(*p_Kalman19d_Starter).pk_dt = 0.1;				// 时间不同步误差标准差初值
	(*p_Kalman19d_Starter).qt_gyro = 1.0e-2 * dpsh; // 陀螺随机游走系数
	(*p_Kalman19d_Starter).qt_acc = 50.0 * ugpsHz;	// 加表随机游走系数
	(*p_Kalman19d_Starter).rk_dvn = 0.1;			// 速度误差量测噪声
	(*p_Kalman19d_Starter).rk_dposLL = 2.0e-7;		// 水平位置误差量测噪声
	(*p_Kalman19d_Starter).rk_dposH = 1;			// 高度位置误差量测噪声

	// xk初始化为零
	for (int i = 0; i < 19; i++)
	{

		(*p_Kalman19d_State).xk[i] = 0;
	}

	// Pk初始化为零
	for (int i = 0; i < 19; i++)
	{
		for (int j = 0; j < 19; j++)
			(*p_Kalman19d_State).Pk[i][j] = 0;
	}

	// qk初始化为零
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
			(*p_Kalman19d_State).qk[i][j] = 0;
	}
	// rk初始化为零
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
			(*p_Kalman19d_State).rk[i][j] = 0;
	}
	// Hk初始化为零
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 19; j++)
			(*p_Kalman19d_State).Hk[i][j] = 0;
	}

	// 设置系统状态方差量矩阵初值Pk

	(*p_Kalman19d_State).Pk[0][0] = pow((*p_Kalman19d_Starter).pk_phipr, 2);
	(*p_Kalman19d_State).Pk[1][1] = pow((*p_Kalman19d_Starter).pk_phipr, 2);
	(*p_Kalman19d_State).Pk[2][2] = pow((*p_Kalman19d_Starter).pk_phiy, 2);

	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).Pk[i][j] = pow((*p_Kalman19d_Starter).pk_dvn, 2);
		}
	}

	for (int i = 6; i < 8; i++)
	{
		for (int j = 6; j < 8; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).Pk[i][j] = pow((*p_Kalman19d_Starter).pk_dposLL, 2);
		}
	}

	(*p_Kalman19d_State).Pk[8][8] = pow((*p_Kalman19d_Starter).pk_dposH, 2);

	for (int i = 9; i < 12; i++)
	{
		for (int j = 9; j < 12; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).Pk[i][j] = pow((*p_Kalman19d_Starter).pk_eb, 2);
		}
	}

	for (int i = 12; i < 15; i++)
	{
		for (int j = 12; j < 15; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).Pk[i][j] = pow((*p_Kalman19d_Starter).pk_db, 2);
		}
	}

	for (int i = 15; i < 18; i++)
	{
		for (int j = 15; j < 18; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).Pk[i][j] = pow((*p_Kalman19d_Starter).pk_dl, 2);
		}
	}

	(*p_Kalman19d_State).Pk[18][18] = pow((*p_Kalman19d_Starter).pk_dt, 2);

	// 设置系统噪声矩阵qk

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).qk[i][j] = pow((*p_Kalman19d_Starter).qt_gyro, 2) * (*p_imu_infor).nts;
		}
	}
	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).qk[i][j] = pow((*p_Kalman19d_Starter).qt_acc, 2) * (*p_imu_infor).nts;
		}
	}

	// 设置系统量测噪声矩阵rk

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).rk[i][j] = pow((*p_Kalman19d_Starter).rk_dvn, 2);
		}
	}

	for (int i = 3; i < 5; i++)
	{
		for (int j = 3; j < 5; j++)
		{
			if (i == j)
				(*p_Kalman19d_State).rk[i][j] = pow((*p_Kalman19d_Starter).rk_dposLL, 2);
		}
	}

	(*p_Kalman19d_State).rk[5][5] = pow((*p_Kalman19d_Starter).rk_dposH, 2);
}

/**
 * @brief  19维滤波器实时状态矩阵更新：构造误差方程分块矩阵，得Ft/Gt、Phikk_1、Tauk及6维Hk
 * @param  p_imu_infor       IMU信息：取cnb、vn、pos、wbeb、fnsf、an、nts
 * @param  p_sys_infor       系统信息
 * @param  p_Kalman19d_State 滤波器状态（出参）：写入Tauk、Phikk_1、Hk
 * @param  p_ethi            地球参数：取wnie/wnin/wnen、RMh/RNh、sl/cl/tl等
 * @retval void
 * @note   Ft按psi角误差方程分块组装：Maa=-askew(wnin)、Mav/Mpv为地球曲率项、
 *         Map/Mvp含地球自转与重力随位置变化项、Mva=askew(fnsf)、Mvv含哥氏项、Mpp位置耦合项；
 *         Phikk_1=I+Ft*nts（一阶离散）；Tauk=Gt（陀螺块-cnb、加表块cnb）；
 *         Hk为6维量测阵：速度3行对速度状态为I、杆臂列为-cnb*wbeb*、时间同步列为an；
 *         位置3行对位置状态为I、杆臂列为-Mpv*cnb、时间同步列为Mpv*vn
 */
void Kalman19d_staRT(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Kalman19d_State *p_Kalman19d_State, struct Ethi *p_ethi)
{

	double beta = 1.0 / 198.0, beta1 = 1.0 / 8.0 * (2.0 * beta * ff + ff * ff), beta2 = 3.08e-6, beta3 = 8.08e-9;
	double Maa[3][3] = {0}, Mav[3][3] = {0}, M1[3][3] = {0}, M2[3][3] = {0}, M3[3][3] = {0}, Map[3][3] = {0};
	double Mva[3][3] = {0}, Mvv[3][3] = {0}, Mvp[3][3] = {0}, Mpv[3][3] = {0}, Mpp[3][3] = {0};

	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0}, mat_temp3[3][3] = {0}, mat_temp4[3][3] = {0}, mat_temp5[3][3] = {0}, mat_temp6[3][3] = {0};
	double mat_temp7[3][3] = {0}, mat_temp8[19][19] = {0}, mat_temp9[19][19] = {0};
	double v_temp1[3] = {0}, v_temp2[3] = {0};

	double mat_temp10[3][3] = {0}, mat_temp11[3][3] = {0}, mat_temp12[3][3] = {0};
	double v_temp3[3] = {0};

	double Ft[19][19] = {0}, Gt[19][6] = {0};
	int i = 0, j = 0;

	// 计算Maa
	vect3d_askew((*p_ethi).wnin, mat_temp1);
	mat_multi_num(mat_temp1[0], -1, Maa[0], 3, 3);

	// 计算Mav
	Mav[0][1] = -1.0 / (*p_ethi).RMh;
	Mav[1][0] = 1.0 / (*p_ethi).RNh;
	Mav[2][0] = (*p_ethi).tl / (*p_ethi).RNh;

	// 计算M1
	M1[1][0] = -wie * (*p_ethi).sl;
	M1[2][0] = wie * (*p_ethi).cl;

	// 计算M2
	M2[0][2] = (*p_imu_infor).vn[1] / ((*p_ethi).RMh * (*p_ethi).RNh);
	M2[1][2] = -(*p_imu_infor).vn[2] / ((*p_ethi).RNh * (*p_ethi).RNh);
	M2[2][0] = (*p_imu_infor).vn[0] / ((*p_ethi).cl * (*p_ethi).cl) / (*p_ethi).RNh;
	M2[2][2] = -(*p_imu_infor).vn[0] * (*p_ethi).tl / ((*p_ethi).RNh * (*p_ethi).RNh);

	// 计算M3

	M3[1][0] = -2 * beta3 * (*p_imu_infor).pos[2] * cos(2 * (*p_imu_infor).pos[0]);
	M3[1][2] = -beta3 * sin(2 * (*p_imu_infor).pos[0]);
	M3[2][0] = -g0 * (beta - 4 * beta1 * cos(2 * (*p_imu_infor).pos[0]) * sin(2 * (*p_imu_infor).pos[0]));
	M3[2][2] = beta2;

	// 计算Map
	mat_plus(M1[0], M2[0], Map[0], 3, 3);

	// 计算Mva
	vect3d_askew((*p_imu_infor).fnsf, Mva);

	// 计算Mvv

	vect3d_askew((*p_imu_infor).vn, mat_temp2); // mat_temp2为vn的反对称阵
	mat_multi(mat_temp2[0], Mav[0], mat_temp3[0], 3, 3, 3);

	mat_multi_num((*p_ethi).wnie, 2, v_temp1, 3, 1);
	mat_plus(v_temp1, (*p_ethi).wnen, v_temp2, 3, 1);
	vect3d_askew(v_temp2, mat_temp4);
	mat_multi_num(mat_temp4[0], -1, mat_temp5[0], 3, 3);

	mat_plus(mat_temp3[0], mat_temp5[0], Mvv[0], 3, 3);

	// 计算Mvp

	mat_plus(M1[0], Map[0], mat_temp6[0], 3, 3);
	mat_multi(mat_temp2[0], mat_temp6[0], mat_temp7[0], 3, 3, 3);
	mat_plus(mat_temp7[0], M3[0], Mvp[0], 3, 3);

	// 计算Mpv
	Mpv[0][1] = 1.0 / (*p_ethi).RMh;
	Mpv[1][0] = 1.0 / (*p_ethi).clRNh;
	Mpv[2][2] = 1.0;

	// 计算Mpp

	Mpp[0][2] = -(*p_imu_infor).vn[1] / ((*p_ethi).RMh * (*p_ethi).RMh);
	Mpp[1][0] = (*p_imu_infor).vn[0] * (*p_ethi).tl / (*p_ethi).clRNh;
	Mpp[1][2] = -(*p_imu_infor).vn[0] / ((*p_ethi).clRNh * (*p_ethi).RNh);

	// 计算Ft
	Ft[0][0] = Maa[0][0];
	Ft[0][1] = Maa[0][1];
	Ft[0][2] = Maa[0][2];
	Ft[1][0] = Maa[1][0];
	Ft[1][1] = Maa[1][1];
	Ft[1][2] = Maa[1][2];
	Ft[2][0] = Maa[2][0];
	Ft[2][1] = Maa[2][1];
	Ft[2][2] = Maa[2][2];

	Ft[0][3] = Mav[0][0];
	Ft[0][4] = Mav[0][1];
	Ft[0][5] = Mav[0][2];
	Ft[1][3] = Mav[1][0];
	Ft[1][4] = Mav[1][1];
	Ft[1][5] = Mav[1][2];
	Ft[2][3] = Mav[2][0];
	Ft[2][4] = Mav[2][1];
	Ft[2][5] = Mav[2][2];

	Ft[0][6] = Map[0][0];
	Ft[0][7] = Map[0][1];
	Ft[0][8] = Map[0][2];
	Ft[1][6] = Map[1][0];
	Ft[1][7] = Map[1][1];
	Ft[1][8] = Map[1][2];
	Ft[2][6] = Map[2][0];
	Ft[2][7] = Map[2][1];
	Ft[2][8] = Map[2][2];

	Ft[0][9] = -(*p_imu_infor).cnb[0][0];
	Ft[0][10] = -(*p_imu_infor).cnb[0][1];
	Ft[0][11] = -(*p_imu_infor).cnb[0][2];
	Ft[1][9] = -(*p_imu_infor).cnb[1][0];
	Ft[1][10] = -(*p_imu_infor).cnb[1][1];
	Ft[1][11] = -(*p_imu_infor).cnb[1][2];
	Ft[2][9] = -(*p_imu_infor).cnb[2][0];
	Ft[2][10] = -(*p_imu_infor).cnb[2][1];
	Ft[2][11] = -(*p_imu_infor).cnb[2][2];

	Ft[3][0] = Mva[0][0];
	Ft[3][1] = Mva[0][1];
	Ft[3][2] = Mva[0][2];
	Ft[4][0] = Mva[1][0];
	Ft[4][1] = Mva[1][1];
	Ft[4][2] = Mva[1][2];
	Ft[5][0] = Mva[2][0];
	Ft[5][1] = Mva[2][1];
	Ft[5][2] = Mva[2][2];

	Ft[3][3] = Mvv[0][0];
	Ft[3][4] = Mvv[0][1];
	Ft[3][5] = Mvv[0][2];
	Ft[4][3] = Mvv[1][0];
	Ft[4][4] = Mvv[1][1];
	Ft[4][5] = Mvv[1][2];
	Ft[5][3] = Mvv[2][0];
	Ft[5][4] = Mvv[2][1];
	Ft[5][5] = Mvv[2][2];

	Ft[3][6] = Mvp[0][0];
	Ft[3][7] = Mvp[0][1];
	Ft[3][8] = Mvp[0][2];
	Ft[4][6] = Mvp[1][0];
	Ft[4][7] = Mvp[1][1];
	Ft[4][8] = Mvp[1][2];
	Ft[5][6] = Mvp[2][0];
	Ft[5][7] = Mvp[2][1];
	Ft[5][8] = Mvp[2][2];

	Ft[3][12] = (*p_imu_infor).cnb[0][0];
	Ft[3][13] = (*p_imu_infor).cnb[0][1];
	Ft[3][14] = (*p_imu_infor).cnb[0][2];
	Ft[4][12] = (*p_imu_infor).cnb[1][0];
	Ft[4][13] = (*p_imu_infor).cnb[1][1];
	Ft[4][14] = (*p_imu_infor).cnb[1][2];
	Ft[5][12] = (*p_imu_infor).cnb[2][0];
	Ft[5][13] = (*p_imu_infor).cnb[2][1];
	Ft[5][14] = (*p_imu_infor).cnb[2][2];

	Ft[6][3] = Mpv[0][0];
	Ft[6][4] = Mpv[0][1];
	Ft[6][5] = Mpv[0][2];
	Ft[7][3] = Mpv[1][0];
	Ft[7][4] = Mpv[1][1];
	Ft[7][5] = Mpv[1][2];
	Ft[8][3] = Mpv[2][0];
	Ft[8][4] = Mpv[2][1];
	Ft[8][5] = Mpv[2][2];

	Ft[6][6] = Mpp[0][0];
	Ft[6][7] = Mpp[0][1];
	Ft[6][8] = Mpp[0][2];
	Ft[7][6] = Mpp[1][0];
	Ft[7][7] = Mpp[1][1];
	Ft[7][8] = Mpp[1][2];
	Ft[8][6] = Mpp[2][0];
	Ft[8][7] = Mpp[2][1];
	Ft[8][8] = Mpp[2][2];

	// 计算Tauk

	Gt[0][0] = -(*p_imu_infor).cnb[0][0];
	Gt[0][1] = -(*p_imu_infor).cnb[0][1];
	Gt[0][2] = -(*p_imu_infor).cnb[0][2];
	Gt[1][0] = -(*p_imu_infor).cnb[1][0];
	Gt[1][1] = -(*p_imu_infor).cnb[1][1];
	Gt[1][2] = -(*p_imu_infor).cnb[1][2];
	Gt[2][0] = -(*p_imu_infor).cnb[2][0];
	Gt[2][1] = -(*p_imu_infor).cnb[2][1];
	Gt[2][2] = -(*p_imu_infor).cnb[2][2];

	Gt[3][3] = (*p_imu_infor).cnb[0][0];
	Gt[3][4] = (*p_imu_infor).cnb[0][1];
	Gt[3][5] = (*p_imu_infor).cnb[0][2];
	Gt[4][3] = (*p_imu_infor).cnb[1][0];
	Gt[4][4] = (*p_imu_infor).cnb[1][1];
	Gt[4][5] = (*p_imu_infor).cnb[1][2];
	Gt[5][3] = (*p_imu_infor).cnb[2][0];
	Gt[5][4] = (*p_imu_infor).cnb[2][1];
	Gt[5][5] = (*p_imu_infor).cnb[2][2];

	for (i = 0; i < 19; i++)
	{
		for (j = 0; j < 6; j++)
		{
			(*p_Kalman19d_State).Tauk[i][j] = Gt[i][j];
		}
	}

	// 计算Phikk_1

	mat_unit(mat_temp8[0], 19);
	mat_multi_num(Ft[0], (*p_imu_infor).nts, mat_temp9[0], 19, 19);
	mat_plus(mat_temp8[0], mat_temp9[0], (*p_Kalman19d_State).Phikk_1[0], 19, 19);

	// 计算Hk

	vect3d_askew((*p_imu_infor).wbeb, mat_temp10);							 // mat_temp10=wbeb*
	mat_multi((*p_imu_infor).cnb[0], mat_temp10[0], mat_temp11[0], 3, 3, 3); // mat_temp11=cnb*wbeb*

	(*p_Kalman19d_State).Hk[0][3] = 1.0;
	(*p_Kalman19d_State).Hk[1][4] = 1.0;
	(*p_Kalman19d_State).Hk[2][5] = 1.0;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			(*p_Kalman19d_State).Hk[i][j + 15] = -mat_temp11[i][j];
		}
	}

	(*p_Kalman19d_State).Hk[0][18] = (*p_imu_infor).an[0];
	(*p_Kalman19d_State).Hk[1][18] = (*p_imu_infor).an[1];
	(*p_Kalman19d_State).Hk[2][18] = (*p_imu_infor).an[2];

	mat_multi(Mpv[0], (*p_imu_infor).cnb[0], mat_temp12[0], 3, 3, 3); // mat_temp12=Mpv*cnb
	mat_multi(Mpv[0], (*p_imu_infor).vn, v_temp3, 3, 3, 1);			  // v_temp3=Mpv*vn

	(*p_Kalman19d_State).Hk[3][6] = 1.0;
	(*p_Kalman19d_State).Hk[4][7] = 1.0;
	(*p_Kalman19d_State).Hk[5][8] = 1.0;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			(*p_Kalman19d_State).Hk[i + 3][j + 15] = -mat_temp12[i][j];
		}
	}

	(*p_Kalman19d_State).Hk[3][18] = v_temp3[0];
	(*p_Kalman19d_State).Hk[4][18] = v_temp3[1];
	(*p_Kalman19d_State).Hk[5][18] = v_temp3[2];
}

/**
 * @brief  19维滤波器实时时间更新：xkk_1=Phi*xk，Pkk_1=Phi*Pk*Phi'+Tau*q*Tau'
 * @param  p_Kalman19d_State 滤波器状态（出参）：读xk/Pk，写xkk_1、Pkk_1并回存xk、Pk
 * @param  p_imu_infor       IMU信息
 * @retval void
 * @note   更新完成后以预测值xkk_1、Pkk_1作为当前xk、Pk，等待序贯量测更新
 */
void Kalman19d_timeRT(struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor)
{

	double mat_temp1[19][19] = {0}, mat_temp2[19][19] = {0}, mat_temp3[19][19] = {0}, mat_temp4[19][6] = {0};
	double mat_temp5[6][19] = {0}, mat_temp6[19][19] = {0};

	// 计算xkk_1
	mat_multi((*p_Kalman19d_State).Phikk_1[0], (*p_Kalman19d_State).xk, (*p_Kalman19d_State).xkk_1, 19, 19, 1); // xkk_1=Phikk_1*xk

	// 计算Pkk_1

	mat_multi((*p_Kalman19d_State).Phikk_1[0], (*p_Kalman19d_State).Pk[0], mat_temp1[0], 19, 19, 19); // mat_temp1=Phikk_1*Pk
	mat_transpose((*p_Kalman19d_State).Phikk_1[0], mat_temp2[0], 19, 19);							  // mat_temp2=Phikk_1'
	mat_multi(mat_temp1[0], mat_temp2[0], mat_temp3[0], 19, 19, 19);								  // mat_temp3=Phikk_1*Pk*Phikk_1'

	mat_multi((*p_Kalman19d_State).Tauk[0], (*p_Kalman19d_State).qk[0], mat_temp4[0], 19, 6, 6); // mat_temp4=Tauk*qk
	mat_transpose((*p_Kalman19d_State).Tauk[0], mat_temp5[0], 19, 6);							 // mat_temp5=Tauk'
	mat_multi(mat_temp4[0], mat_temp5[0], mat_temp6[0], 19, 6, 19);								 // mat_temp6=Tauk*qk*Tauk'

	mat_plus(mat_temp3[0], mat_temp6[0], (*p_Kalman19d_State).Pkk_1[0], 19, 19); // Pkk_1=Phikk_1*Pk*Phikk_1'+Tauk*qk*Tauk'

	// 计算xk
	mat_assign((*p_Kalman19d_State).xkk_1, (*p_Kalman19d_State).xk, 19, 1);

	// 计算Pk

	mat_assign((*p_Kalman19d_State).Pkk_1[0], (*p_Kalman19d_State).Pk[0], 19, 19);
}

/**
 * @brief  19维滤波器序贯6维量测更新：速度3维+位置3维分两组依次量测，写xk、Pk
 * @param  p_Kalman19d_State 滤波器状态（出参）：读Hk/rk/xkk_1/Pkk_1，写xk、Pk
 * @param  p_imu_infor       IMU信息：取vn（m/s）、pos（纬经rad、高度m）构造量测
 * @param  p_sys_infor       系统信息：取vn_GNSS/pos_GNSS并存放杆臂/时间同步补偿量；成功后置flag_KalmanSuc=1
 * @param  p_ethi            地球参数
 * @retval void
 * @note   先调LeverArm_Spatial、TimeAsync_ImuGNSS计算空间杆臂与时间不同步补偿，
 *         z1k=vn-(vn_GNSS+dvnL+dvndt)（速度3维，m/s），z2k=pos-(pos_GNSS+dpGL+dpdt)（位置3维）；
 *         第一组：K1=P0*H1'*(H1*P0*H1'+r1)^-1，x1=x0+K1*(z1-H1*x0)，P1=(I-K1*H1)*P0；
 *         第二组序贯：K2=P1*H2'*(H2*P1*H2'+r2)^-1，x2=x1+K2*(z2-H2*x1)，P2=(I-K2*H2)*P1；
 *         最终xk=x2、Pk=P2
 */
void Kalman19d_mes_seq6dRT(struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi)

{
	// 声明经过杆臂和时间不同步误差补偿后的卫导速度和位置
	double vn_GNSSLA[3] = {0}, pos_GNSSLA[3] = {0};

	// 声明计算K1k,x1k,P1k所需要的临时矩阵
	double mat_temp1[19][3] = {0}, mat_temp2[19][3] = {0}, mat_temp3[3][3] = {0}, mat_temp4[3][3] = {0};
	double mat_temp5[3][3] = {0}, mat_temp6[3][1] = {0}, mat_temp7[3][1] = {0}, mat_temp8[19][1] = {0};
	double mat_temp9[19][19] = {0}, mat_temp10[19][19] = {0}, mat_temp11[19][19] = {0};

	// 声明计算K2k,x2k,P2k所需要的临时矩阵
	double mat_temp21[19][3] = {0}, mat_temp22[19][3] = {0}, mat_temp23[3][3] = {0}, mat_temp24[3][3] = {0};
	double mat_temp25[3][3] = {0}, mat_temp26[3][1] = {0}, mat_temp27[3][1] = {0}, mat_temp28[19][1] = {0};
	double mat_temp29[19][19] = {0}, mat_temp30[19][19] = {0}, mat_temp31[19][19] = {0};

	double z1k[3] = {0}, z2k[3] = {0}, H1k[3][19] = {0}, H2k[3][19] = {0}, r1k[3][3] = {0}, r2k[3][3] = {0};
	double x0k[19] = {0}, P0k[19][19] = {0}, K1k[19][3] = {0}, x1k[19] = {0}, P1k[19][19] = {0}, K2k[19][3] = {0}, x2k[19] = {0}, P2k[19][19] = {0};

	int i = 0, j = 0;

	// H1k赋值
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 19; j++)
		{
			H1k[i][j] = (*p_Kalman19d_State).Hk[i][j];
		}
	}

	// H2k赋值
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 19; j++)
		{
			H2k[i][j] = (*p_Kalman19d_State).Hk[i + 3][j];
		}
	}

	// r1k赋值
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			r1k[i][j] = (*p_Kalman19d_State).rk[i][j];
		}
	}

	// r2k赋值
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			r2k[i][j] = (*p_Kalman19d_State).rk[i + 3][j + 3];
		}
	}

	// 计算空间杆臂和时间不同步误差
	LeverArm_Spatial(p_imu_infor, p_sys_infor, p_ethi);
	TimeAsync_ImuGNSS(p_imu_infor, p_sys_infor, p_ethi);

	// 卫导数据补偿
	vn_GNSSLA[0] = (*p_sys_infor).vn_GNSS[0] + (*p_sys_infor).dvnL[0] + (*p_sys_infor).dvndt[0];
	vn_GNSSLA[1] = (*p_sys_infor).vn_GNSS[1] + (*p_sys_infor).dvnL[1] + (*p_sys_infor).dvndt[1];
	vn_GNSSLA[2] = (*p_sys_infor).vn_GNSS[2] + (*p_sys_infor).dvnL[2] + (*p_sys_infor).dvndt[2];

	pos_GNSSLA[0] = (*p_sys_infor).pos_GNSS[0] + (*p_sys_infor).dpGL[0] + (*p_sys_infor).dpdt[0];
	pos_GNSSLA[1] = (*p_sys_infor).pos_GNSS[1] + (*p_sys_infor).dpGL[1] + (*p_sys_infor).dpdt[1];
	pos_GNSSLA[2] = (*p_sys_infor).pos_GNSS[2] + (*p_sys_infor).dpGL[2] + (*p_sys_infor).dpdt[2];

	// 赋新量测值
	z1k[0] = (*p_imu_infor).vn[0] - vn_GNSSLA[0];
	z1k[1] = (*p_imu_infor).vn[1] - vn_GNSSLA[1];
	z1k[2] = (*p_imu_infor).vn[2] - vn_GNSSLA[2];

	z2k[0] = (*p_imu_infor).pos[0] - pos_GNSSLA[0];
	z2k[1] = (*p_imu_infor).pos[1] - pos_GNSSLA[1];
	z2k[2] = (*p_imu_infor).pos[2] - pos_GNSSLA[2];

	// 状态赋值

	mat_assign((*p_Kalman19d_State).xkk_1, x0k, 19, 1);

	// 方差矩阵赋值
	mat_assign((*p_Kalman19d_State).Pkk_1[0], P0k[0], 19, 19);

	// 计算K1k
	mat_transpose(H1k[0], mat_temp1[0], 3, 19);				  // mat_temp1=H1k'
	mat_multi(P0k[0], mat_temp1[0], mat_temp2[0], 19, 19, 3); // mat_temp2=P0k*H1k'
	mat_multi(H1k[0], mat_temp2[0], mat_temp3[0], 3, 19, 3);  // mat_temp3=H1k*mat_temp2
	mat_plus(mat_temp3[0], r1k[0], mat_temp4[0], 3, 3);		  // mat_temp4=mat_temp3+r1k
	mat3d_inverse(mat_temp4, mat_temp5);					  // mat_temp5=(mat_temp3+r1k)^-1
	mat_multi(mat_temp2[0], mat_temp5[0], K1k[0], 19, 3, 3);  // K1k=P0k*H1k'*(mat_temp3+r1k)^-1

	// 计算x1k
	mat_multi(H1k[0], x0k, mat_temp6[0], 3, 19, 1);			 // mat_temp6=H1k*x0k
	mat_minus(z1k, mat_temp6[0], mat_temp7[0], 3, 1);		 // mat_temp7=z1k-mat_temp6
	mat_multi(K1k[0], mat_temp7[0], mat_temp8[0], 19, 3, 1); // mat_temp8=K1k*mat_temp7
	mat_plus(x0k, mat_temp8[0], x1k, 19, 1);				 // x1k=x0k+(z1k-H1k*x0k)

	// 计算P1k
	mat_unit(mat_temp9[0], 19);									   // mat_temp9=eye(19)
	mat_multi(K1k[0], H1k[0], mat_temp10[0], 19, 3, 19);		   // mat_temp10=K1k*H1k
	mat_minus(mat_temp9[0], mat_temp10[0], mat_temp11[0], 19, 19); // mat_temp11=eye(19)-K1k*H1k
	mat_multi(mat_temp11[0], P0k[0], P1k[0], 19, 19, 19);		   // P1k=(eye(19)-K1k*H1k)*P0k

	// 序贯处理
	// 计算K2k
	mat_transpose(H2k[0], mat_temp21[0], 3, 19);				// mat_temp21=H2k'
	mat_multi(P1k[0], mat_temp21[0], mat_temp22[0], 19, 19, 3); // mat_temp22=P1k*H2k'
	mat_multi(H2k[0], mat_temp22[0], mat_temp23[0], 3, 19, 3);	// mat_temp23=H2k*mat_temp22
	mat_plus(mat_temp23[0], r2k[0], mat_temp24[0], 3, 3);		// mat_temp24=mat_temp23+r2k
	mat3d_inverse(mat_temp24, mat_temp25);						// mat_temp25=(mat_temp23+r2k)^-1
	mat_multi(mat_temp22[0], mat_temp25[0], K2k[0], 19, 3, 3);	// K2k=P1k*H2k'*(mat_temp23+r2k)^-1

	// 计算x2k
	mat_multi(H2k[0], x1k, mat_temp26[0], 3, 19, 1);		   // mat_temp26=H2k*x1k
	mat_minus(z2k, mat_temp26[0], mat_temp27[0], 3, 1);		   // mat_temp27=z2k-mat_temp26
	mat_multi(K2k[0], mat_temp27[0], mat_temp28[0], 19, 3, 1); // mat_temp28=K2k*mat_temp27
	mat_plus(x1k, mat_temp28[0], x2k, 19, 1);				   // x2k=x1k+(z2k-H2k*x1k)

	// 计算P2k
	mat_unit(mat_temp29[0], 19);									// mat_temp29=eye(19)
	mat_multi(K2k[0], H2k[0], mat_temp30[0], 19, 3, 19);			// mat_temp30=K2k*H2k
	mat_minus(mat_temp29[0], mat_temp30[0], mat_temp31[0], 19, 19); // mat_temp31=eye(19)-K2k*H2k
	mat_multi(mat_temp31[0], P1k[0], P2k[0], 19, 19, 19);			// P2k=(eye(19)-K2k*H2k)*P1k

	// 计算xk,Pk
	mat_assign(x2k, (*p_Kalman19d_State).xk, 19, 1);
	mat_assign(P2k[0], (*p_Kalman19d_State).Pk[0], 19, 19);

	(*p_sys_infor).flag_KalmanSuc = 1;
}

/**
 * @brief  19维滤波器状态反馈：用xk速度/位置误差修正惯导vn、pos，并清零对应误差状态
 * @param  p_Kalman19d_State 滤波器状态（出参）：xk[3..8]读出后清零
 * @param  p_sys_infor       系统信息（出参）：flag_KalmanSuc置0
 * @param  p_imu_infor       IMU信息（出参）：vn（m/s）、pos（纬经rad、高度m）被修正
 * @retval void
 * @note   执行vn-=xk[3..5]、pos-=xk[6..8]的闭环反馈；失准角/零偏/杆臂/时间同步状态不在此反馈，
 *         保持继续估计
 */
void Kalman19d_Feedback(struct Kalman19d_State *p_Kalman19d_State, struct Sys_Infor *p_sys_infor, struct Imu_Infor *p_imu_infor)
{

	// 系统的速度、位置等于惯导的速度、位置减去估计误差
	(*p_imu_infor).vn[0] = (*p_imu_infor).vn[0] - (*p_Kalman19d_State).xk[3];
	(*p_imu_infor).vn[1] = (*p_imu_infor).vn[1] - (*p_Kalman19d_State).xk[4];
	(*p_imu_infor).vn[2] = (*p_imu_infor).vn[2] - (*p_Kalman19d_State).xk[5];

	(*p_imu_infor).pos[0] = (*p_imu_infor).pos[0] - (*p_Kalman19d_State).xk[6];
	(*p_imu_infor).pos[1] = (*p_imu_infor).pos[1] - (*p_Kalman19d_State).xk[7];
	(*p_imu_infor).pos[2] = (*p_imu_infor).pos[2] - (*p_Kalman19d_State).xk[8];

	// 速度、位置估计误差清零
	(*p_Kalman19d_State).xk[3] = 0;
	(*p_Kalman19d_State).xk[4] = 0;
	(*p_Kalman19d_State).xk[5] = 0;
	(*p_Kalman19d_State).xk[6] = 0;
	(*p_Kalman19d_State).xk[7] = 0;
	(*p_Kalman19d_State).xk[8] = 0;

	// 滤波成功标志位置零
	(*p_sys_infor).flag_KalmanSuc = 0;
}

/**
 * @brief  19维组合导航主流程：初始化、惯导更新、时间更新、序贯量测更新与状态反馈
 * @param  p_imu_infor         IMU信息
 * @param  p_ethi              地球参数
 * @param  p_sys_infor         系统信息：各流程标志位
 * @param  p_Kalman19d_State   滤波器状态
 * @param  p_Kalman19d_Starter 滤波器初始参数
 * @retval void
 * @note   首次调用执行Kalman19d_Init；IMU有效时ins_update_RT，navsuc后做staRT+timeRT；
 *         每累计1s置卫导有效执行一次序贯6维量测更新；滤波成功后调Kalman19d_Feedback完成闭环修正
 */
void nav_integ(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor, struct Kalman19d_State *p_Kalman19d_State, struct Kalman19d_Starter *p_Kalman19d_Starter)
{
	static double t1s = 0;
	/* v_temp1 / mat_temp1 / mat_temp2 — 未使用, 已移除声明 (同文件其他函数有同名局部变量, 不受影响) */

	// 如果组合导航滤波器未初始化
	if ((*p_sys_infor).flag_NavIntegInit == 0)
	{
		Kalman19d_Init(p_Kalman19d_Starter, p_Kalman19d_State, p_imu_infor, p_sys_infor);
		(*p_sys_infor).flag_NavIntegInit = 1;
	}
	// 如果惯导数据有效
	if ((*p_imu_infor).flag_ValidIMU == 1)

	{
		ins_update_RT(p_imu_infor, p_ethi, p_sys_infor); // 导航
		(*p_imu_infor).flag_ValidIMU = 0;				 // 用完惯导数据标志置0

		// 如果双子样导航成功
		if ((*p_imu_infor).flag_navsuc == 1)
		{
			t1s = t1s + (*p_imu_infor).nts; // 1秒标志位累加计数

			// 滤波器状态矩阵更新
			Kalman19d_staRT(p_imu_infor, p_sys_infor, p_Kalman19d_State, p_ethi);
			// 滤波器时间更新
			Kalman19d_timeRT(p_Kalman19d_State, p_imu_infor);
		}
		// 如果1秒标志位累加计数==1
		if (fabs(t1s - 1.0) < 1e-10)
		{
			t1s = 0;						   // 计数清零
			(*p_sys_infor).flag_ValidGNSS = 1; // 卫导标志位置有效
		}

		// 如果卫导有效
		if ((*p_sys_infor).flag_ValidGNSS == 1)
		{
			Kalman19d_mes_seq6dRT(p_Kalman19d_State, p_imu_infor, p_sys_infor, p_ethi); // 量测更新
		}

		// 如果滤波成功
		if ((*p_sys_infor).flag_KalmanSuc == 1)
		{
			(*p_sys_infor).flag_KalmanSuc = 0; // 滤波成功标志位置0
			(*p_sys_infor).flag_ValidGNSS = 0; // 卫导有效标志位置0
			// 进行滤波估计对预测值的修正
			Kalman19d_Feedback(p_Kalman19d_State, p_sys_infor, p_imu_infor);
		}
	}
}
