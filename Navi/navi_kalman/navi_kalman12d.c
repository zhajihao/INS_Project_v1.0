/**
 * @file navi_kalman12d.c
 * @brief 12维卡尔曼滤波器实现（扩展精对准）
 *
 * 状态量：失准角3 + 速度误差3 + 陀螺零偏3 + 加表零偏3
 * 相比 7d 增加加表零偏估计，适用于更高精度对准需求。
 */

#include "navi_kalman12d.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include "navi_core/navi_attitude.h"
#include "navi_core/navi_earth.h"
#include "navi_core/navi_ins.h"
#include <math.h>

/**
 * @brief  12维精对准滤波器初始化：装订Pk/qk/rk/Hk初值，xk清零
 * @param  p_Kalman12d_Starter 滤波器初始参数：失准角/速度/陀螺漂移/加表零偏标准差、随机游走系数、量测噪声
 * @param  p_Kalman12d_State   滤波器状态结构体（出参）：写入xk、Pk、qk、rk、Hk
 * @param  p_imu_infor         IMU信息，取nts（s）作为离散时间间隔
 * @param  p_sys_infor         系统信息：初始化完成后置flag_PreciseAlignInit=1
 * @retval void
 * @note   仅在flag_PreciseAlignInit==0时执行，否则直接返回；
 *         12维状态xk=[失准角3(rad)、速度误差3(m/s)、陀螺零偏3(rad/s)、加表零偏3(m/s²)]；
 *         Pk按各标准差平方装订对角阵；qk=噪声系数^2*nts；rk=(rk0)^2；Hk量测3维速度误差
 */
void Kalman12d_Init(struct Kalman12d_Starter *p_Kalman12d_Starter, struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	if ((*p_sys_infor).flag_PreciseAlignInit == 0)
	{
		(*p_Kalman12d_Starter).pk_phipr = 0.1 * arcdeg; // 水平失准角标准差初值
		(*p_Kalman12d_Starter).pk_phiy = 0.5 * arcdeg;	// 航向失准角标准差初值
		(*p_Kalman12d_Starter).pk_dvn = 1.0;			// 速度误差标准差初值
		(*p_Kalman12d_Starter).pk_eb = 1.0e-2 * dph;	// 陀螺常值漂移标准差初值
		(*p_Kalman12d_Starter).pk_db = 1.0e3 * ug;		// 加表零偏标准差初值
		(*p_Kalman12d_Starter).qt_gyro = 1.0e-3 * dpsh; // 陀螺随机游走系数
		(*p_Kalman12d_Starter).qt_acc = 10.0 * ugpsHz;	// 加表随机游走系数
		(*p_Kalman12d_Starter).rk0 = 1.0e-4;			// 量测噪声

		// xk初始化为零
		for (int i = 0; i < 12; i++)
		{

			(*p_Kalman12d_State).xk[i] = 0;
		}

		// Pk初始化为零
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
				(*p_Kalman12d_State).Pk[i][j] = 0;
		}

		// qk初始化为零
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
				(*p_Kalman12d_State).qk[i][j] = 0;
		}
		// rk初始化为零
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				(*p_Kalman12d_State).rk[i][j] = 0;
		}
		// Hk初始化为零
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 12; j++)
				(*p_Kalman12d_State).Hk[i][j] = 0;
		}

		// 设置系统状态方差量矩阵初值Pk

		(*p_Kalman12d_State).Pk[0][0] = pow((*p_Kalman12d_Starter).pk_phipr, 2);
		(*p_Kalman12d_State).Pk[1][1] = pow((*p_Kalman12d_Starter).pk_phipr, 2);
		(*p_Kalman12d_State).Pk[2][2] = pow((*p_Kalman12d_Starter).pk_phiy, 2);

		for (int i = 3; i < 6; i++)
		{
			for (int j = 3; j < 6; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).Pk[i][j] = pow((*p_Kalman12d_Starter).pk_dvn, 2);
			}
		}
		for (int i = 6; i < 9; i++)
		{
			for (int j = 6; j < 9; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).Pk[i][j] = pow((*p_Kalman12d_Starter).pk_eb, 2);
			}
		}
		for (int i = 9; i < 12; i++)
		{
			for (int j = 9; j < 12; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).Pk[i][j] = pow((*p_Kalman12d_Starter).pk_db, 2);
			}
		}

		// double a1 = Pk[6][6];
		// double a2 = Pk[7][7];
		// double a3 = Pk[8][8];

		// 设置系统噪声矩阵qk

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).qk[i][j] = pow((*p_Kalman12d_Starter).qt_gyro, 2) * (*p_imu_infor).nts;
			}
		}
		for (int i = 3; i < 6; i++)
		{
			for (int j = 3; j < 6; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).qk[i][j] = pow((*p_Kalman12d_Starter).qt_acc, 2) * (*p_imu_infor).nts;
			}
		}

		// 设置系统量测噪声矩阵rk

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (i == j)
					(*p_Kalman12d_State).rk[i][j] = pow((*p_Kalman12d_Starter).rk0, 2);
			}
		}

		// 设置系统量测矩阵Hk

		(*p_Kalman12d_State).Hk[0][3] = 1;
		(*p_Kalman12d_State).Hk[1][4] = 1;
		(*p_Kalman12d_State).Hk[2][5] = 1;

		(*p_sys_infor).flag_PreciseAlignInit = 1;
	}
	else
		return;
}

/**
 * @brief  12维滤波器状态矩阵更新：计算Ft、Gt并离散化得Phikk_1、Tauk
 * @param  p_imu_infor       IMU信息：取cnb姿态阵、nts离散间隔（s）
 * @param  p_Kalman12d_State 滤波器状态（出参）：写入Phikk_1、Tauk
 * @param  p_ethi            地球参数：取wnie（rad/s）、gn重力矢量（m/s²）
 * @retval void
 * @note   仅在flag_navsuc==1时执行：先调earth更新地球参数；
 *         Ft含-askew(wnie)姿态项、askew(-gn)加表重力耦合项、陀螺零偏(-cnb)与加表零偏(cnb)耦合项；
 *         Gt噪声驱动阵陀螺块为-cnb、加表块为cnb；Phikk_1=I+Ft*nts（一阶离散）；Tauk=Gt
 */
void kalman12d_sta_sta(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State, struct Ethi *p_ethi)
{

	double mat_temp1[3][3], mat_temp2[3][3], mat_temp3[3][3], mat_temp4[3][3], mat_temp5[12][12], mat_temp6[12][12];
	double vect_temp1[3];

	if ((*p_imu_infor).flag_navsuc == 1)
	{
		earth(p_imu_infor, p_ethi);

		vect3d_askew((*p_ethi).wnie, mat_temp1);

		mat_multi_num(mat_temp1[0], -1.0, mat_temp2[0], 3, 3);

		mat_multi_num((*p_imu_infor).cnb[0], -1.0, mat_temp3[0], 3, 3);

		mat_multi_num((*p_ethi).gn, -1.0, vect_temp1, 3, 1);
		vect3d_askew(vect_temp1, mat_temp4);

		double Ft[12][12] = {0};
		Ft[0][0] = mat_temp2[0][0];
		Ft[0][1] = mat_temp2[0][1];
		Ft[0][2] = mat_temp2[0][2];
		Ft[1][0] = mat_temp2[1][0];
		Ft[1][1] = mat_temp2[1][1];
		Ft[1][2] = mat_temp2[1][2];
		Ft[2][0] = mat_temp2[2][0];
		Ft[2][1] = mat_temp2[2][1];
		Ft[2][2] = mat_temp2[2][2];

		Ft[0][6] = mat_temp3[0][0];
		Ft[0][7] = mat_temp3[0][1];
		Ft[0][8] = mat_temp3[0][2];
		Ft[1][6] = mat_temp3[1][0];
		Ft[1][7] = mat_temp3[1][1];
		Ft[1][8] = mat_temp3[1][2];
		Ft[2][6] = mat_temp3[2][0];
		Ft[2][7] = mat_temp3[2][1];
		Ft[2][8] = mat_temp3[2][2];

		Ft[3][0] = mat_temp4[0][0];
		Ft[3][1] = mat_temp4[0][1];
		Ft[3][2] = mat_temp4[0][2];
		Ft[4][0] = mat_temp4[1][0];
		Ft[4][1] = mat_temp4[1][1];
		Ft[4][2] = mat_temp4[1][2];
		Ft[5][0] = mat_temp4[2][0];
		Ft[5][1] = mat_temp4[2][1];
		Ft[5][2] = mat_temp4[2][2];

		Ft[3][9] = (*p_imu_infor).cnb[0][0];
		Ft[3][10] = (*p_imu_infor).cnb[0][1];
		Ft[3][11] = (*p_imu_infor).cnb[0][2];
		Ft[4][9] = (*p_imu_infor).cnb[1][0];
		Ft[4][10] = (*p_imu_infor).cnb[1][1];
		Ft[4][11] = (*p_imu_infor).cnb[1][2];
		Ft[5][9] = (*p_imu_infor).cnb[2][0];
		Ft[5][10] = (*p_imu_infor).cnb[2][1];
		Ft[5][11] = (*p_imu_infor).cnb[2][2];

		double Gt[12][6] = {0};
		Gt[0][0] = mat_temp3[0][0];
		Gt[0][1] = mat_temp3[0][1];
		Gt[0][2] = mat_temp3[0][2];
		Gt[1][0] = mat_temp3[1][0];
		Gt[1][1] = mat_temp3[1][1];
		Gt[1][2] = mat_temp3[1][2];
		Gt[2][0] = mat_temp3[2][0];
		Gt[2][1] = mat_temp3[2][1];
		Gt[2][2] = mat_temp3[2][2];

		Gt[3][3] = (*p_imu_infor).cnb[0][0];
		Gt[3][4] = (*p_imu_infor).cnb[0][1];
		Gt[3][5] = (*p_imu_infor).cnb[0][2];
		Gt[4][3] = (*p_imu_infor).cnb[1][0];
		Gt[4][4] = (*p_imu_infor).cnb[1][1];
		Gt[4][5] = (*p_imu_infor).cnb[1][2];
		Gt[5][3] = (*p_imu_infor).cnb[2][0];
		Gt[5][4] = (*p_imu_infor).cnb[2][1];
		Gt[5][5] = (*p_imu_infor).cnb[2][2];

		mat_unit(mat_temp5[0], 12);

		mat_multi_num(Ft[0], (*p_imu_infor).nts, mat_temp6[0], 12, 12);

		mat_plus(mat_temp5[0], mat_temp6[0], (*p_Kalman12d_State).Phikk_1[0], 12, 12);

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				(*p_Kalman12d_State).Tauk[i][j] = Gt[i][j];
			}
		}
	}
	else
		return;
}

/**
 * @brief  12维滤波器时间更新：xkk_1=Phi*xk，Pkk_1=Phi*Pk*Phi'+Tau*q*Tau'
 * @param  p_Kalman12d_State 滤波器状态（出参）：读xk/Pk，写xkk_1、Pkk_1并回存xk、Pk
 * @param  p_imu_infor       IMU信息
 * @param  p_sys_infor       系统信息：更新成功后置flag_KalmanSuc=1
 * @retval void
 * @note   仅在flag_navsuc==1时执行；更新完成后以预测值xkk_1、Pkk_1作为当前xk、Pk，等待量测更新
 */
void kalman12d_time_sta(struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	double mat_temp1[12][12], mat_temp2[12][12], mat_temp3[12][12], mat_temp4[12][6], mat_temp5[6][12], mat_temp6[12][12];

	if ((*p_imu_infor).flag_navsuc == 1)
	{
		mat_multi((*p_Kalman12d_State).Phikk_1[0], (*p_Kalman12d_State).xk, (*p_Kalman12d_State).xkk_1, 12, 12, 1);

		mat_multi((*p_Kalman12d_State).Phikk_1[0], (*p_Kalman12d_State).Pk[0], mat_temp1[0], 12, 12, 12);

		mat_transpose((*p_Kalman12d_State).Phikk_1[0], mat_temp2[0], 12, 12);

		mat_multi(mat_temp1[0], mat_temp2[0], mat_temp3[0], 12, 12, 12);

		mat_multi((*p_Kalman12d_State).Tauk[0], (*p_Kalman12d_State).qk[0], mat_temp4[0], 12, 6, 6);

		mat_transpose((*p_Kalman12d_State).Tauk[0], mat_temp5[0], 12, 6);

		mat_multi(mat_temp4[0], mat_temp5[0], mat_temp6[0], 12, 6, 12);

		mat_plus(mat_temp3[0], mat_temp6[0], (*p_Kalman12d_State).Pkk_1[0], 12, 12);

		for (int i = 0; i < 12; i++)
		{
			(*p_Kalman12d_State).xk[i] = (*p_Kalman12d_State).xkk_1[i];
		}

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(*p_Kalman12d_State).Pk[i][j] = (*p_Kalman12d_State).Pkk_1[i][j];
			}
		}

		(*p_sys_infor).flag_KalmanSuc = 1;
	}
	else
		return;
}

/**
 * @brief  12维滤波器量测更新：K=Pkk_1*H'*(H*Pkk_1*H'+r)^-1，Joseph形式更新Pk
 * @param  p_Kalman12d_State 滤波器状态（出参）：读Pkk_1/xkk_1/Hk/rk，写xk、Pk
 * @param  p_imu_infor       IMU信息：取vn速度（m/s）作为量测zk
 * @param  p_sys_infor       系统信息：以flag_ValidGNSS==1作为量测更新条件
 * @retval void
 * @note   3维速度误差量测zk=vn（m/s）；xk=xkk_1+K*(zk-H*xkk_1)；
 *         Pk=(I-K*H)*Pkk_1*(I-K*H)'+K*r*K'（Joseph稳定形式，保证对称正定）
 */
void kalman12d_mes_sta(struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)

{

	double mat_temp1[12][3], mat_temp2[12][3], mat_temp3[3][12], mat_temp4[3][3], mat_temp5[3][3], mat_temp6[3][3], mat_temp7[12][3], mat_temp8[12][12], mat_temp9[12][12];
	double mat_temp10[12][12], mat_temp11[12][12], mat_temp12[12][12], mat_temp13[12][12], mat_temp14[3][12], mat_temp15[12][3], mat_temp16[12][12];

	double vect1[3], vect2[3], vect3[12];
	double zk[3];
	// if ((*p_NavSuc_Flag) == 1 && (*p_MesUpdateSuc_Flag)==1)
	if ((*p_sys_infor).flag_ValidGNSS == 1)
	{
		mat_assign((*p_imu_infor).vn, zk, 3, 1); // zk=vn

		mat_transpose((*p_Kalman12d_State).Hk[0], mat_temp1[0], 3, 12); // mat_temp1=Hk'

		mat_multi((*p_Kalman12d_State).Pkk_1[0], mat_temp1[0], mat_temp2[0], 12, 12, 3); // mat_temp2=Pkk_1*Hk'

		mat_multi((*p_Kalman12d_State).Hk[0], (*p_Kalman12d_State).Pkk_1[0], mat_temp3[0], 3, 12, 12); // mat_temp3=Hk*Pkk_1

		mat_multi(mat_temp3[0], mat_temp1[0], mat_temp4[0], 3, 12, 3); // mat_temp4=Hk*Pkk_1*Hk'

		mat_plus(mat_temp4[0], (*p_Kalman12d_State).rk[0], mat_temp5[0], 3, 3); // mat_temp5=Hk*Pkk_1*Hk'+rk
		mat3d_inverse(mat_temp5, mat_temp6);									// mat_temp6=(Hk*Pkk_1*Hk'+rk)^-1

		mat_multi(mat_temp2[0], mat_temp6[0], mat_temp7[0], 12, 3, 3); // mat_temp7=Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1

		mat_multi((*p_Kalman12d_State).Hk[0], (*p_Kalman12d_State).xkk_1, vect1, 3, 12, 1); // vect1=Hk*xkk_1

		mat_minus(zk, vect1, vect2, 3, 1); // vect2=zk-Hk*xkk_1

		mat_multi(mat_temp7[0], vect2, vect3, 12, 3, 1); // vect3=Kk*(zk-Hk*xkk_1)

		mat_plus((*p_Kalman12d_State).xkk_1, vect3, (*p_Kalman12d_State).xk, 12, 1); // xk=xkk_1+Kk*(zk-Hk*xkk_1)

		mat_multi(mat_temp7[0], (*p_Kalman12d_State).Hk[0], mat_temp8[0], 12, 3, 12); // mat_temp8=Kk*Hk

		mat_unit(mat_temp9[0], 12); // mat_temp9=eye(12)

		mat_minus(mat_temp9[0], mat_temp8[0], mat_temp10[0], 12, 12); // mat_temp10=eye(12)-Kk*Hk

		mat_transpose(mat_temp10[0], mat_temp11[0], 12, 12); // mat_temp11=(eye(12)-Kk*Hk)'

		mat_multi(mat_temp10[0], (*p_Kalman12d_State).Pkk_1[0], mat_temp12[0], 12, 12, 12); // mat_temp12=(eye(12)-Kk*Hk)*Pkk_1

		mat_multi(mat_temp12[0], mat_temp11[0], mat_temp13[0], 12, 12, 12); // mat_temp13 = (eye(12) - Kk * Hk) * Pkk_1*(eye(12)-Kk*Hk)'

		mat_transpose(mat_temp7[0], mat_temp14[0], 12, 3); // mat_temp14=(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)'

		mat_multi(mat_temp7[0], (*p_Kalman12d_State).rk[0], mat_temp15[0], 12, 3, 3); // mat_temp15=(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)*rk

		mat_multi(mat_temp15[0], mat_temp14[0], mat_temp16[0], 12, 3, 12); // mat_temp16=(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)*rk*(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)'

		mat_plus(mat_temp13[0], mat_temp16[0], (*p_Kalman12d_State).Pk[0], 12, 12); // Pk=(eye(12) - Kk * Hk) * Pkk_1*(eye(12)-Kk*Hk)'+(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)*rk*(Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1)'
	}
	else
		return;
}

/**
 * @brief  12维单位置精对准单周期：初始化+惯导更新+状态/时间/量测更新
 * @param  p_imu_infor       IMU信息
 * @param  p_ethi            地球参数
 * @param  p_Kalman12d_Starter 滤波器初始参数
 * @param  p_Kalman12d_State 滤波器状态
 * @param  p_sys_infor       系统信息
 * @retval void
 * @note   依次调用Kalman12d_Init、ins_update_RT、kalman12d_sta_sta、kalman12d_time_sta、kalman12d_mes_sta，
 *         适用于静止单位置条件下的精对准
 */
void PreciseAlignKalman12d_SinglePos(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman12d_Starter *p_Kalman12d_Starter, struct Kalman12d_State *p_Kalman12d_State, struct Sys_Infor *p_sys_infor)
{

	Kalman12d_Init(p_Kalman12d_Starter, p_Kalman12d_State, p_imu_infor, p_sys_infor);

	ins_update_RT(p_imu_infor, p_ethi, p_sys_infor);

	kalman12d_sta_sta(p_imu_infor, p_Kalman12d_State, p_ethi);
	kalman12d_time_sta(p_Kalman12d_State, p_imu_infor, p_sys_infor);
	kalman12d_mes_sta(p_Kalman12d_State, p_imu_infor, p_sys_infor);
}

/**
 * @brief  12维旋转精对准的时间更新段：惯导更新+状态矩阵更新+时间更新（无量测）
 * @param  p_imu_infor       IMU信息
 * @param  p_ethi            地球参数
 * @param  p_Kalman12d_State 滤波器状态
 * @param  p_sys_infor       系统信息
 * @retval void
 * @note   依次调用ins_update_RT、kalman12d_sta_sta、kalman12d_time_sta，
 *         用于旋转调制条件下无外部量测时的状态递推
 */
void PreciseAlignKalman12d_RotTime(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman12d_State *p_Kalman12d_State, struct Sys_Infor *p_sys_infor)
{

	ins_update_RT(p_imu_infor, p_ethi, p_sys_infor);
	kalman12d_sta_sta(p_imu_infor, p_Kalman12d_State, p_ethi);
	kalman12d_time_sta(p_Kalman12d_State, p_imu_infor, p_sys_infor);
}

/**
 * @brief  12维滤波器速度相关状态重置：vn清零、xk速度项清零、Pk速度行列重置
 * @param  p_imu_infor       IMU信息（出参）：vn[3]清零（m/s）
 * @param  p_Kalman12d_State 滤波器状态（出参）：xk[3..5]清零，Pk第3~5行/列清零且对角元置1
 * @retval void
 * @note   量测反馈后切断速度通道的旧相关性，使速度误差状态重新从单位方差开始递推
 */
void Kalman12d_PxvReset(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State)
{
	int i = 0;
	int j = 0;
	vect3d_assign(0.0, 0.0, 0.0, (*p_imu_infor).vn);
	(*p_Kalman12d_State).xk[3] = 0.0;
	(*p_Kalman12d_State).xk[4] = 0.0;
	(*p_Kalman12d_State).xk[5] = 0.0;

	for (i = 3; i < 6; i++)
	{
		for (j = 0; j < 12; j++)
			(*p_Kalman12d_State).Pk[i][j] = 0.0;
	}

	for (i = 0; i < 12; i++)
	{
		for (j = 3; j < 6; j++)
			(*p_Kalman12d_State).Pk[i][j] = 0.0;
	}

	(*p_Kalman12d_State).Pk[3][3] = 1.0;
	(*p_Kalman12d_State).Pk[4][4] = 1.0;
	(*p_Kalman12d_State).Pk[5][5] = 1.0;
}

/**
 * @brief  速度误差状态重置：Pk速度子块置单位阵、vn清零、xk速度项清零
 * @param  Pk   12维状态方差阵（出参）：[3..5][3..5]子块重置为单位阵
 * @param  vn   速度矢量（出参）：清零（m/s）
 * @param  xk   12维状态矢量（出参）：xk[3..5]清零
 * @retval void
 * @note   以零矢量mat_assign方式清零vn，速度误差状态重置为无信息（单位方差）状态
 */
void kalman12d_dvnreset(double Pk[12][12], double vn[3], double xk[12])
{
	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			if (i == j)
				Pk[i][j] = 1;
			else
				Pk[i][j] = 0;
		}
	}
	// vect3d_assign(0, 0, 0, vn);
	double v_temp[3] = {0};
	mat_assign(v_temp, vn, 3, 1);

	xk[3] = 0;
	xk[4] = 0;
	xk[5] = 0;
}

/**
 * @brief  对准结果反馈：用xk失准角构造旋转阵修正cnb并重算姿态角att
 * @param  p_imu_infor       IMU信息（出参）：cnb被修正，att由m2att重算（rad）
 * @param  p_Kalman12d_State 滤波器状态：取xk[0..2]失准角（rad）
 * @retval void
 * @note   cnb_align=rv2mat(xk[0..2])（旋转矢量转旋转阵），cnb=cnb_align*cnb；
 *         与7d的小角阵a2mat不同，本函数采用旋转矢量形式，适用于较大失准角
 */
void AlignResultUpdate(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State)
{
	double cnb_align[3][3] = {0.0};
	double vect_temp[3] = {0.0};
	double mat_temp[3][3] = {0.0};

	vect3d_assign((*p_Kalman12d_State).xk[0], (*p_Kalman12d_State).xk[1], (*p_Kalman12d_State).xk[2], vect_temp);

	rv2mat(vect_temp, cnb_align);

	mat_assign((*p_imu_infor).cnb[0], mat_temp[0], 3, 3);
	mat_multi(cnb_align[0], mat_temp[0], (*p_imu_infor).cnb[0], 3, 3, 3);
	m2att((*p_imu_infor).cnb, (*p_imu_infor).att);
}
