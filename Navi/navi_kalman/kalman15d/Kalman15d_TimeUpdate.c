/**
 * @file Kalman15d_TimeUpdate.c
 * @brief 15维卡尔曼滤波器时间更新
 *
 * 完成状态一步外推与协方差传播，全链路float矩阵运算。
 */

#include <math.h>
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"
#include "navi_math/math_matrix.h"
#include "navi_config/navi_types.h"
#include "Kalman15d_SSMat.h"

/**
 * @brief  15维卡尔曼滤波时间更新：状态一步外推与协方差传播
 * @param  p_Kalman15d_SSMat  15维滤波器状态矩阵(读写)：输入xk/Pk/Phikk_1/Tauk/qk，输出xkk_1/Pkk_1并回写xk/Pk
 * @retval void
 * @note   xkk_1=Phikk_1*xk；Pkk_1=Phikk_1*Pk*Phikk_1'+Tauk*qk*Tauk'；随后xk=xkk_1、Pk=Pkk_1
 */
void Kalman15d_TimeUpdate(struct Kalman15d_SSMat *p_Kalman15d_SSMat)
{

	float mat_temp1[15][15] = {0}, mat_temp2[15][15] = {0}, mat_temp3[15][15] = {0}, mat_temp4[15][6] = {0};
	float mat_temp5[6][15] = {0}, mat_temp6[15][15] = {0};

	// 计算xkk_1
	mat_multi_f((*p_Kalman15d_SSMat).Phikk_1[0], (*p_Kalman15d_SSMat).xk, (*p_Kalman15d_SSMat).xkk_1, 15, 15, 1); // xkk_1=Phikk_1*xk

	// 计算Pkk_1
	mat_multi_f((*p_Kalman15d_SSMat).Phikk_1[0], (*p_Kalman15d_SSMat).Pk[0], mat_temp1[0], 15, 15, 15); // mat_temp1=Phikk_1*Pk
	mat_transpose_f((*p_Kalman15d_SSMat).Phikk_1[0], mat_temp2[0], 15, 15);								// mat_temp2=Phikk_1'
	mat_multi_f(mat_temp1[0], mat_temp2[0], mat_temp3[0], 15, 15, 15);									// mat_temp3=Phikk_1*Pk*Phikk_1'

	mat_multi_f((*p_Kalman15d_SSMat).Tauk[0], (*p_Kalman15d_SSMat).qk[0], mat_temp4[0], 15, 6, 6); // mat_temp4=Tauk*qk
	mat_transpose_f((*p_Kalman15d_SSMat).Tauk[0], mat_temp5[0], 15, 6);							   // mat_temp5=Tauk'
	mat_multi_f(mat_temp4[0], mat_temp5[0], mat_temp6[0], 15, 6, 15);							   // mat_temp6=Tauk*qk*Tauk'

	mat_plus_f(mat_temp3[0], mat_temp6[0], (*p_Kalman15d_SSMat).Pkk_1[0], 15, 15); // Pkk_1=Phikk_1*Pk*Phikk_1'+Tauk*qk*Tauk'

	// 计算xk
	mat_assign_f((*p_Kalman15d_SSMat).xkk_1, (*p_Kalman15d_SSMat).xk, 15, 1);

	// 计算Pk

	mat_assign_f((*p_Kalman15d_SSMat).Pkk_1[0], (*p_Kalman15d_SSMat).Pk[0], 15, 15);
}
