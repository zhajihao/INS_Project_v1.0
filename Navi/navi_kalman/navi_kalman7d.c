/**
 * @file navi_kalman7d.c
 * @brief 7维卡尔曼滤波器实现（精对准阶段）
 *
 * 状态量：水平失准角2 + 航向失准角1 + 水平速度误差2 + 陀螺零偏2
 * 用于粗对准完成后的精对准，收敛水平姿态和航向。
 */

#include "navi_kalman7d.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include "navi_core/navi_attitude.h"
#include "navi_core/navi_ins.h"
#include <math.h>

/**
 * @brief  7维精对准滤波器初始化：装订Pk/qk/rk/Hk初值，xk清零
 * @param  p_Kalman7d_Starter 滤波器初始参数：失准角/速度/陀螺漂移标准差、随机游走系数、量测噪声
 * @param  p_Kalman7d_State   滤波器状态结构体（出参）：写入xk、Pk、qk、rk、Hk
 * @param  p_imu_infor        IMU信息，取nts（s）作为离散时间间隔
 * @param  p_sys_infor        系统信息
 * @retval void
 * @note   7维状态xk=[水平失准角2+航向失准角1(rad)、水平速度误差2(m/s)、陀螺零偏2(rad/s)]；
 *         Pk按各标准差平方装订对角阵；qk=噪声系数^2*nts；rk=(rk0)^2；Hk量测2维水平速度误差
 */
void Kalman7d_Init(struct Kalman7d_Starter *p_Kalman7d_Starter, struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	(*p_Kalman7d_Starter).pk_phipr = 0.1 * arcdeg; // 水平失准角标准差初值
	(*p_Kalman7d_Starter).pk_phiy = 0.5 * arcdeg;  // 航向失准角标准差初值
	(*p_Kalman7d_Starter).pk_dvn = 1.0;			   // 速度误差标准差初值
	(*p_Kalman7d_Starter).pk_eb = 1.0e-2 * dph;	   // 陀螺常值漂移标准差初值

	(*p_Kalman7d_Starter).qt_gyro = 1.0e-3 * dpsh; // 陀螺随机游走系数
	(*p_Kalman7d_Starter).qt_acc = 10.0 * ugpsHz;  // 加表随机游走系数
	(*p_Kalman7d_Starter).rk0 = 1.0e-4;			   // 量测噪声

	// xk初始化为零
	for (int i = 0; i < 7; i++)
	{

		(*p_Kalman7d_State).xk[i] = 0;
	}

	// Pk初始化为零
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 7; j++)
			(*p_Kalman7d_State).Pk[i][j] = 0;
	}

	// qk初始化为零
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
			(*p_Kalman7d_State).qk[i][j] = 0;
	}
	// rk初始化为零
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			(*p_Kalman7d_State).rk[i][j] = 0;
	}
	// Hk初始化为零
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 7; j++)
			(*p_Kalman7d_State).Hk[i][j] = 0;
	}

	// 设置系统状态方差量矩阵初值Pk

	(*p_Kalman7d_State).Pk[0][0] = pow((*p_Kalman7d_Starter).pk_phipr, 2);
	(*p_Kalman7d_State).Pk[1][1] = pow((*p_Kalman7d_Starter).pk_phipr, 2);
	(*p_Kalman7d_State).Pk[2][2] = pow((*p_Kalman7d_Starter).pk_phiy, 2);

	for (int i = 3; i < 5; i++)
	{
		for (int j = 3; j < 5; j++)
		{
			if (i == j)
				(*p_Kalman7d_State).Pk[i][j] = pow((*p_Kalman7d_Starter).pk_dvn, 2);
		}
	}
	for (int i = 5; i < 7; i++)
	{
		for (int j = 5; j < 7; j++)
		{
			if (i == j)
				(*p_Kalman7d_State).Pk[i][j] = pow((*p_Kalman7d_Starter).pk_eb, 2);
		}
	}

	// 设置系统噪声矩阵qk

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j)
				(*p_Kalman7d_State).qk[i][j] = pow((*p_Kalman7d_Starter).qt_gyro, 2) * (*p_imu_infor).nts;
		}
	}
	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			if (i == j)
				(*p_Kalman7d_State).qk[i][j] = pow((*p_Kalman7d_Starter).qt_acc, 2) * (*p_imu_infor).nts;
		}
	}

	// 设置系统量测噪声矩阵rk

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			if (i == j)
				(*p_Kalman7d_State).rk[i][j] = pow((*p_Kalman7d_Starter).rk0, 2);
		}
	}

	// 设置系统量测矩阵Hk

	(*p_Kalman7d_State).Hk[0][3] = 1;
	(*p_Kalman7d_State).Hk[1][4] = 1;
}

/**
 * @brief  7维滤波器状态矩阵更新：计算Ft、Gt并离散化得Phikk_1、Tauk
 * @param  p_imu_infor      IMU信息：取cnb姿态阵、nts离散间隔（s）
 * @param  p_sys_infor      系统信息
 * @param  p_Kalman7d_State 滤波器状态（出参）：写入Phikk_1、Tauk
 * @param  p_ethi           地球参数：取wnie地球自转角速度（rad/s）、gn重力矢量（m/s²）
 * @retval void
 * @note   Ft含-askew(wnie)姿态项、gn引起的失准角-速度耦合项及陀螺零偏耦合项；
 *         Gt为噪声驱动阵（陀螺项-cnb、加表项cnb）；Phikk_1=I+Ft*nts（一阶离散）；Tauk=Gt
 */
void Kalman7d_sta_sta(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Kalman7d_State *p_Kalman7d_State, struct Ethi *p_ethi)
{
	double Ft[7][7] = {0}, Gt[7][6] = {0};
	double mat_temp1[3][3], mat_temp2[3][3], mat_temp3[7][7], mat_temp4[7][7];

	int i = 0, j = 0;

	// 计算Ft
	vect3d_askew((*p_ethi).wnie, mat_temp1);			   // mat_temp1=askew(wnie)
	mat_multi_num(mat_temp1[0], -1.0, mat_temp2[0], 3, 3); // mat_temp2=-askew(wnie)

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Ft[i][j] = mat_temp2[i][j];
		}
	}
	Ft[1][5] = -1.0;
	Ft[2][6] = -1.0;

	Ft[3][1] = (*p_ethi).gn[2];
	Ft[4][0] = -(*p_ethi).gn[2];

	// 计算Gt

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Gt[i][j] = -(*p_imu_infor).cnb[i][j];
		}
	}

	for (j = 3; j < 5; j++)
	{
		Gt[3][j] = (*p_imu_infor).cnb[0][j - 3];
	}

	for (j = 3; j < 5; j++)
	{
		Gt[4][j] = (*p_imu_infor).cnb[1][j - 3];
	}

	// 计算Phikk_1
	mat_unit(mat_temp3[0], 7);
	mat_multi_num(Ft[0], (*p_imu_infor).nts, mat_temp4[0], 7, 7);
	mat_plus(mat_temp3[0], mat_temp4[0], (*p_Kalman7d_State).Phikk_1[0], 7, 7);

	// 计算Tauk
	mat_assign(Gt[0], (*p_Kalman7d_State).Tauk[0], 7, 6);
}

/**
 * @brief  7维滤波器时间更新：xkk_1=Phi*xk，Pkk_1=Phi*Pk*Phi'+Tau*q*Tau'
 * @param  p_Kalman7d_State 滤波器状态（出参）：读xk/Pk，写xkk_1、Pkk_1并回存xk、Pk
 * @param  p_imu_infor      IMU信息
 * @param  p_sys_infor      系统信息
 * @retval void
 * @note   更新完成后以预测值xkk_1、Pkk_1作为当前xk、Pk，等待量测更新
 */
void Kalman7d_time_sta(struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	double mat_temp1[7][7] = {0}, mat_temp2[7][7] = {0}, mat_temp3[7][7] = {0}, mat_temp4[7][6] = {0};
	double mat_temp5[6][7] = {0}, mat_temp6[7][7] = {0};

	// 计算xkk_1
	mat_multi((*p_Kalman7d_State).Phikk_1[0], (*p_Kalman7d_State).xk, (*p_Kalman7d_State).xkk_1, 7, 7, 1); // xkk_1=Phikk_1*xk

	// 计算Pkk_1

	mat_multi((*p_Kalman7d_State).Phikk_1[0], (*p_Kalman7d_State).Pk[0], mat_temp1[0], 7, 7, 7); // mat_temp1=Phikk_1*Pk
	mat_transpose((*p_Kalman7d_State).Phikk_1[0], mat_temp2[0], 7, 7);							 // mat_temp2=Phikk_1'
	mat_multi(mat_temp1[0], mat_temp2[0], mat_temp3[0], 7, 7, 7);								 // mat_temp3=Phikk_1*Pk*Phikk_1'

	mat_multi((*p_Kalman7d_State).Tauk[0], (*p_Kalman7d_State).qk[0], mat_temp4[0], 7, 6, 6); // mat_temp4=Tauk*qk
	mat_transpose((*p_Kalman7d_State).Tauk[0], mat_temp5[0], 7, 6);							  // mat_temp5=Tauk'
	mat_multi(mat_temp4[0], mat_temp5[0], mat_temp6[0], 7, 6, 7);							  // mat_temp6=Tauk*qk*Tauk'

	mat_plus(mat_temp3[0], mat_temp6[0], (*p_Kalman7d_State).Pkk_1[0], 7, 7); // Pkk_1=Phikk_1*Pk*Phikk_1'+Tauk*qk*Tauk'

	// 计算xk
	mat_assign((*p_Kalman7d_State).xkk_1, (*p_Kalman7d_State).xk, 7, 1);

	// 计算Pk

	mat_assign((*p_Kalman7d_State).Pkk_1[0], (*p_Kalman7d_State).Pk[0], 7, 7);
}

/**
 * @brief  7维滤波器量测更新：K=Pkk_1*H'*(H*Pkk_1*H'+r)^-1，修正xk、Pk
 * @param  p_Kalman7d_State 滤波器状态（出参）：读Pkk_1/xkk_1/Hk/rk，写xk、Pk
 * @param  p_imu_infor      IMU信息：取vn水平速度（m/s）作为量测zk
 * @param  p_sys_infor      系统信息：量测成功后置flag_KalmanSuc=1
 * @retval void
 * @note   静基座下zk=vn即速度误差量测（2维，m/s）；xk=xkk_1+K*(zk-H*xkk_1)；
 *         Pk=(I-K*H)*Pkk_1
 */
void Kalman7d_mes_sta(struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)

{

	double mat_temp1[7][2], mat_temp2[7][2], mat_temp3[2][7], mat_temp4[2][2], mat_temp5[2][2];
	double mat_temp6[2][2], mat_temp7[7][2], mat_temp8[7][7], mat_temp9[7][7], mat_temp10[7][7];

	double vect1[2], vect2[2], vect3[7];
	double zk[2];

	// 计算xk
	zk[0] = (*p_imu_infor).vn[0];
	zk[1] = (*p_imu_infor).vn[1]; // zk=vn

	mat_transpose((*p_Kalman7d_State).Hk[0], mat_temp1[0], 2, 7); // mat_temp1=Hk'

	mat_multi((*p_Kalman7d_State).Pkk_1[0], mat_temp1[0], mat_temp2[0], 7, 7, 2); // mat_temp2=Pkk_1*Hk'

	mat_multi((*p_Kalman7d_State).Hk[0], (*p_Kalman7d_State).Pkk_1[0], mat_temp3[0], 2, 7, 7); // mat_temp3=Hk*Pkk_1

	mat_multi(mat_temp3[0], mat_temp1[0], mat_temp4[0], 2, 7, 2); // mat_temp4=Hk*Pkk_1*Hk'

	mat_plus(mat_temp4[0], (*p_Kalman7d_State).rk[0], mat_temp5[0], 2, 2); // mat_temp5=Hk*Pkk_1*Hk'+rk
	mat2d_inverse(mat_temp5, mat_temp6);								   // mat_temp6=(Hk*Pkk_1*Hk'+rk)^-1

	mat_multi(mat_temp2[0], mat_temp6[0], mat_temp7[0], 7, 2, 2); // mat_temp7=Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1

	mat_multi((*p_Kalman7d_State).Hk[0], (*p_Kalman7d_State).xkk_1, vect1, 2, 7, 1); // vect1=Hk*xkk_1

	mat_minus(zk, vect1, vect2, 2, 1); // vect2=zk-Hk*xkk_1

	mat_multi(mat_temp7[0], vect2, vect3, 7, 2, 1); // vect3=Kk*(zk-Hk*xkk_1)

	mat_plus((*p_Kalman7d_State).xkk_1, vect3, (*p_Kalman7d_State).xk, 7, 1); // xk=xkk_1+Kk*(zk-Hk*xkk_1)

	// 计算Pk
	mat_multi(mat_temp7[0], (*p_Kalman7d_State).Hk[0], mat_temp8[0], 7, 2, 7); // mat_temp8=Kk*Hk

	mat_unit(mat_temp9[0], 7); // mat_temp9=eye(7)

	mat_minus(mat_temp9[0], mat_temp8[0], mat_temp10[0], 7, 7); // mat_temp10=eye(7)-Kk*Hk

	mat_multi(mat_temp10[0], (*p_Kalman7d_State).Pkk_1[0], (*p_Kalman7d_State).Pk[0], 7, 7, 7); // Pk=(eye(7)-Kk*Hk)*Pkk_1

	(*p_sys_infor).flag_KalmanSuc = 1;
}

/**
 * @brief  7维Kalman精对准主流程：初始化、惯导更新、时间/量测更新，完成后反馈修正姿态
 * @param  p_imu_infor        IMU信息：取nts、vn、cnb，完成时重写cnb、att
 * @param  p_ethi             地球参数
 * @param  p_Kalman7d_Starter 滤波器初始参数
 * @param  p_Kalman7d_State   滤波器状态
 * @param  p_sys_infor        系统信息：各流程标志位及对准总时长t_align（s）
 * @retval void
 * @note   首次调用执行Kalman7d_Init；每个IMU周期ins_update_RT并做状态/时间更新；
 *         每累计1s置卫导有效执行一次量测更新；累计对准3/4*t_align（s）后判精对准完成，
 *         用xk失准角（rad）构造小角阵a2mat左乘cnb修正姿态，并由m2att重算att（rad）
 */
void PreciseAlign_Kalman7d(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman7d_Starter *p_Kalman7d_Starter, struct Kalman7d_State *p_Kalman7d_State, struct Sys_Infor *p_sys_infor)
{

	static double t1s = 0, t_paligncount = 0;
	double v_temp1[3] = {0};
	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0};

	// 如果精对准未初始化
	if ((*p_sys_infor).flag_PreciseAlignInit == 0)
	{
		// 初始化滤波器
		Kalman7d_Init(p_Kalman7d_Starter, p_Kalman7d_State, p_imu_infor, p_sys_infor);
		(*p_sys_infor).flag_PreciseAlignInit = 1;
	}
	// 如果惯导数据有效
	if ((*p_imu_infor).flag_ValidIMU == 1)

	{
		ins_update_RT(p_imu_infor, p_ethi, p_sys_infor); // 导航
		(*p_imu_infor).flag_ValidIMU = 0;				 // 用完惯导数据标志置0

		// 如果双子样导航成功
		if ((*p_imu_infor).flag_navsuc == 1)
		{
			t1s = t1s + (*p_imu_infor).nts;						// 1秒标志位累加计数
			t_paligncount = t_paligncount + (*p_imu_infor).nts; // 精对准时间计数
			// 滤波器状态矩阵更新
			Kalman7d_sta_sta(p_imu_infor, p_sys_infor, p_Kalman7d_State, p_ethi);
			// 滤波器时间更新
			Kalman7d_time_sta(p_Kalman7d_State, p_imu_infor, p_sys_infor);
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
			Kalman7d_mes_sta(p_Kalman7d_State, p_imu_infor, p_sys_infor); // 量测更新
		}

		// 如果滤波成功
		if ((*p_sys_infor).flag_KalmanSuc == 1)
		{
			(*p_sys_infor).flag_KalmanSuc = 0; // 滤波成功标志位置0
			(*p_sys_infor).flag_ValidGNSS = 0; // 卫导有效标志位置0
		}

		// 如果精度准计数等于精对准时长
		if (fabs(t_paligncount - 3.0 / 4.0 * (*p_sys_infor).t_align) < 1e-10)
		{
			(*p_sys_infor).flag_Finish_palign = 1; // 精对准完成
		}

		// 如果精对准完成
		if ((*p_sys_infor).flag_Finish_palign == 1)
		{
			// 进行滤波估计对预测值的修正
			vect3d_assign((*p_Kalman7d_State).xk[0], (*p_Kalman7d_State).xk[1], (*p_Kalman7d_State).xk[2], v_temp1);
			a2mat(v_temp1, mat_temp1);
			mat_multi(mat_temp1[0], (*p_imu_infor).cnb[0], mat_temp2[0], 3, 3, 3);

			mat_assign(mat_temp2[0], (*p_imu_infor).cnb[0], 3, 3);

			m2att((*p_imu_infor).cnb, (*p_imu_infor).att);

			// 为了避免接下来的函数重复使用精对准的IMU数据，精对准完成后将IMU数据有效位置0，重新读取或产生IMU数据
			(*p_imu_infor).flag_ValidIMU = 0;
		}
	}
}
