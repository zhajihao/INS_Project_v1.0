/**
 * @file Kalman15d_Init.c
 * @brief 15维卡尔曼滤波器初始化
 *
 * 装订滤波器初始协方差Pk与系统噪声阵qk，状态量xk清零。
 */

#include <math.h>
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"
#include "Kalman15d_SSMat.h"

/**
 * @brief  15维卡尔曼滤波器初始化：装订Pk/qk初值，xk清零
 * @param  p_Kalman15d_SSMat  15维滤波器状态矩阵(出参)：写入xk、Pk、qk
 * @param  nts                离散时间间隔(s)
 * @retval void
 * @note   15维状态xk=[失准角3(rad)、速度误差3(m/s)、位置误差3(rad,rad,m)、陀螺零偏3(rad/s)、加表零偏3(m/s^2)]；
 *         Pk按PHIH/PHIU/V/POSH/POSV/EB/DB_INIT_STD平方装订对角阵；qk=[ARW^2*nts(3轴陀螺)、VRW^2*nts(3轴加表)]
 */
void Kalman15d_Init(struct Kalman15d_SSMat *p_Kalman15d_SSMat, float nts)
{

	// xk初始化为零
	for (int i = 0; i < 15; i++)
	{

		(*p_Kalman15d_SSMat).xk[i] = 0;
	}

	// Pk初始化为零
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 15; j++)
			(*p_Kalman15d_SSMat).Pk[i][j] = 0;
	}

	// qk初始化为零
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
			(*p_Kalman15d_SSMat).qk[i][j] = 0;
	}

	// 设置系统状态方差量矩阵初值Pk

	(*p_Kalman15d_SSMat).Pk[0][0] = pow(PHIH_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[1][1] = pow(PHIH_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[2][2] = pow(PHIU_INIT_STD, 2);

	(*p_Kalman15d_SSMat).Pk[3][3] = pow(V_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[4][4] = pow(V_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[5][5] = pow(V_INIT_STD, 2);

	(*p_Kalman15d_SSMat).Pk[6][6] = pow(POSH_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[7][7] = pow(POSH_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[8][8] = pow(POSV_INIT_STD, 2);

	(*p_Kalman15d_SSMat).Pk[9][9] = pow(EB_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[10][10] = pow(EB_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[11][11] = pow(EB_INIT_STD, 2);

	(*p_Kalman15d_SSMat).Pk[12][12] = pow(DB_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[13][13] = pow(DB_INIT_STD, 2);
	(*p_Kalman15d_SSMat).Pk[14][14] = pow(DB_INIT_STD, 2);

	// 设置噪声矩阵Q

	(*p_Kalman15d_SSMat).qk[0][0] = pow(ARW, 2) * nts;
	(*p_Kalman15d_SSMat).qk[1][1] = pow(ARW, 2) * nts;
	(*p_Kalman15d_SSMat).qk[2][2] = pow(ARW, 2) * nts;

	(*p_Kalman15d_SSMat).qk[3][3] = pow(VRW, 2) * nts;
	(*p_Kalman15d_SSMat).qk[4][4] = pow(VRW, 2) * nts;
	(*p_Kalman15d_SSMat).qk[5][5] = pow(VRW, 2) * nts;
}
