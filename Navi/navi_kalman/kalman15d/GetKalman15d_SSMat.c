/**
 * @file GetKalman15d_SSMat.c
 * @brief 15维卡尔曼滤波器系统矩阵构造
 *
 * 由惯导误差方程生成连续系统阵Ft/Gt，一阶离散化得到Phikk_1与Tauk。
 */

#include "navi_config/navi_types.h"
#include <math.h>
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"
#include "navi_math/math_matrix.h"
#include "Kalman15d_SSMat.h"

/**
 * @brief  构造15维滤波器系统矩阵：由惯导误差方程生成状态转移阵Phikk_1与噪声分配阵Tauk
 * @param  p_imu_infor        IMU信息：输入vn(m/s)、pos(rad,rad,m)、fnsf、cnb、nts(s)
 * @param  p_Kalman15d_SSMat  15维滤波器状态矩阵(出参)：写入Phikk_1、Tauk
 * @param  p_ethi             地球参数(RMh/RNh/clRNh/wnie/wnen/wnin等)
 * @retval void
 * @note   Ft由Maa/Mav/Map/Mva/Mvv/Mvp/Mpv/Mpp/M_taug/M_taua子块拼装(内部double计算)；
 *         Phikk_1=I+Ft*nts(一阶离散化)；Tauk=Gt=[-cnb(失准角/陀螺通道);+cnb(速度/加表通道)]分块
 */
void GetKalman15d_SSMat(struct Imu_Infor *p_imu_infor, struct Kalman15d_SSMat *p_Kalman15d_SSMat, struct Ethi *p_ethi)
{

#if 1
	double beta = 1.0 / 198.0, beta1 = 1.0 / 8.0 * (2.0 * beta * ff + ff * ff), beta2 = 3.08e-6, beta3 = 8.08e-9;
	double Maa[3][3] = {0}, Mav[3][3] = {0}, M1[3][3] = {0}, M2[3][3] = {0}, M3[3][3] = {0}, Map[3][3] = {0};
	double Mva[3][3] = {0}, Mvv[3][3] = {0}, Mvp[3][3] = {0}, Mpv[3][3] = {0}, Mpp[3][3] = {0}, M_taug[3][3] = {0}, M_taua[3][3] = {0};

	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0}, mat_temp3[3][3] = {0}, mat_temp4[3][3] = {0}, mat_temp5[3][3] = {0}, mat_temp6[3][3] = {0};
	double mat_temp7[3][3] = {0}, mat_temp8[15][15] = {0}, mat_temp9[15][15] = {0};
	double v_temp1[3] = {0}, v_temp2[3] = {0};

	double Ft[15][15] = {0}, Gt[15][6] = {0};
	int i = 0, j = 0;

	// 计算Maa
	vect3d_askew((*p_ethi).wnin, mat_temp1);
	mat_multi_num(mat_temp1[0], -1.0, Maa[0], 3, 3);

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

	// 计算M_taug
	// 光纤惯导设为全零矩阵,直接初始化,此处屏蔽即可
	// MEMS惯导需要设置
	M_taug[0][0] = -1.0 / TAU_G;
	M_taug[1][1] = -1.0 / TAU_G;
	M_taug[2][2] = -1.0 / TAU_G;

	// 计算M_taua
	// 光纤惯导设为全零矩阵,直接初始化,此处屏蔽即可
	// MEMS惯导需要设置
	M_taua[0][0] = -1.0 / TAU_A;
	M_taua[1][1] = -1.0 / TAU_A;
	M_taua[2][2] = -1.0 / TAU_A;

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

	Ft[9][9] = M_taug[0][0];
	Ft[10][10] = M_taug[1][1];
	Ft[11][11] = M_taug[2][2];

	Ft[12][12] = M_taua[0][0];
	Ft[13][13] = M_taua[1][1];
	Ft[14][14] = M_taua[2][2];

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

	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 6; j++)
		{
			(*p_Kalman15d_SSMat).Tauk[i][j] = (float)Gt[i][j];
		}
	}

	// 计算Phikk_1

	mat_unit(mat_temp8[0], 15);
	mat_multi_num(Ft[0], (*p_imu_infor).nts, mat_temp9[0], 15, 15);
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15; j++)
		{
			(*p_Kalman15d_SSMat).Phikk_1[i][j] = (float)(mat_temp8[i][j] + mat_temp9[i][j]); // Phikk_1=I+Ft*nts
		}
	}

#endif
}
