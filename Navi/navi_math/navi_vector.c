/**
 * @file navi_vector.c
 * @brief 三维向量运算库
 *
 * 提供向量构造、反对称矩阵、点乘、叉乘、赋值等基础运算。
 */

#include "navi_math/navi_vector.h"
#include "navi_math/navi_matrix.h"
#include <math.h>

/**
 * @brief  三维向量构造反对称矩阵（叉乘矩阵）mat = (v×)
 * @param  v 输入三维向量
 * @param  mat 输出3×3反对称矩阵（结果写入）
 * @retval void
 * @note   对任意向量u满足 v×u = (v×)·u；对角线为0，且mat^T = -mat
 */
void vect3d_askew(double v[3], double mat[3][3])
{
	mat[0][0] = 0;
	mat[0][1] = -v[2];
	mat[0][2] = v[1];

	mat[1][0] = v[2];
	mat[1][1] = 0;
	mat[1][2] = -v[0];

	mat[2][0] = -v[1];
	mat[2][1] = v[0];
	mat[2][2] = 0;
}

/**
 * @brief  三维向量叉乘 v3 = v1 × v2
 * @param  v1 输入向量v1
 * @param  v2 输入向量v2
 * @param  v3 输出叉乘结果向量（结果写入）
 * @retval void
 * @note   借助v1的反对称矩阵与v2（按3×1列向量）相乘实现，结果方向服从右手定则
 */
void vect3d_cross_vect3d(double v1[3], double v2[3], double v3[3])
{
	double mat1[3][3];
	vect3d_askew(v1, mat1);

	mat_multi(mat1[0], v2, v3, 3, 3, 1);
}

/**
 * @brief  由三个标量分量赋值三维向量 v = [a0, a1, a2]
 * @param  a0 第1分量
 * @param  a1 第2分量
 * @param  a2 第3分量
 * @param  v 输出三维向量（结果写入）
 * @retval void
 */
void vect3d_assign(double a0, double a1, double a2, double v[3])
{
	v[0] = a0;
	v[1] = a1;
	v[2] = a2;
}

/**
 * @brief  计算三维向量的2-范数（模长）||v||
 * @param  v 输入三维向量
 * @retval 向量模长 sqrt(v[0]^2 + v[1]^2 + v[2]^2)，非负标量
 */
double norm_3d(double v[3])
{

	double sum = 0, a = 0;
	sum = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	a = sqrt(sum);
	return a;
}
