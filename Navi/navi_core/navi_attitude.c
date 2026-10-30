/**
 * @file navi_attitude.c
 * @brief 姿态角与方向余弦矩阵相互转换
 *
 * 提供 a2mat(姿态角→方向余弦矩阵)、m2att(方向余弦矩阵→姿态角)、
 * rv2mat(旋转矢量→方向余弦矩阵)。
 * 姿态角顺序：[俯仰角, 横滚角, 航向角]，单位 rad。
 */

#include "navi_attitude.h"
#include "navi_math/navi_vector.h"
#include "navi_math/navi_matrix.h"
#include <math.h>

/**
 * @brief  姿态角转方向余弦矩阵 cnb = C_b^n
 * @param  att 输入姿态角 [俯仰角rad, 横滚角rad, 航向角rad]
 * @param  cnb 输出3×3方向余弦矩阵（载体系b到导航系n，结果写入）
 * @retval void
 * @note   按航向-俯仰-横滚顺序构造，等价于 C=Rz(ψ)Rx(θ)Ry(γ)
 */
void a2mat(double att[3], double cnb[3][3])
{
	double si, sj, sk, ci, cj, ck;

	si = sin(att[0]);
	sj = sin(att[1]);
	sk = sin(att[2]);
	ci = cos(att[0]);
	cj = cos(att[1]);
	ck = cos(att[2]);

	cnb[0][0] = cj * ck - si * sj * sk;
	cnb[0][1] = -sk * ci;
	cnb[0][2] = sj * ck + si * cj * sk;

	cnb[1][0] = cj * sk + si * sj * ck;
	cnb[1][1] = ci * ck;
	cnb[1][2] = sj * sk - si * cj * ck;

	cnb[2][0] = -ci * sj;
	cnb[2][1] = si;
	cnb[2][2] = ci * cj;
}

/**
 * @brief  方向余弦矩阵转姿态角 att = [俯仰, 横滚, 航向]
 * @param  cnb 输入3×3方向余弦矩阵 C_b^n
 * @param  att 输出姿态角 [俯仰角rad, 横滚角rad, 航向角rad]（结果写入）
 * @retval void
 * @note   俯仰角=asin(C[2][1])；当|C[2][1]|≥0.999999（俯仰近±90°，万向节锁）时航向角置0并采用退化公式
 */
void m2att(double cnb[3][3], double att[3])
{
	if (fabs(cnb[2][1]) < 0.999999)
	{
		att[0] = asin(cnb[2][1]);
		att[1] = -atan2(cnb[2][0], cnb[2][2]);
		att[2] = -atan2(cnb[0][1], cnb[1][1]);
	}
	else
	{
		att[0] = asin(cnb[2][1]);
		att[1] = atan2(cnb[0][2], cnb[0][0]);
		att[2] = 0;
	}
}

/**
 * @brief  旋转矢量转方向余弦矩阵（罗德里格斯公式）mat = I + a·(v×) + b·(v×)^2
 * @param  v 输入旋转矢量（方向为瞬时转轴，模值为转动角rad）
 * @param  mat 输出3×3方向余弦矩阵（结果写入）
 * @retval void
 * @note   a=sinφ/φ、b=(1-cosφ)/φ^2，φ=|v|；当φ^2<1e-8时改用泰勒级数求a、b，避免小角度除零
 */
void rv2mat(double v[3], double mat[3][3])
{
	double modu, modu2, a, b;
	double mat_temp1[3][3], mat_temp2[3][3], mat_temp3[3][3], mat_temp4[3][3], mat_temp5[3][3], mat_temp6[3][3];
	modu2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	modu = sqrt(modu2);
	if (modu2 < 1e-8)
	{
		a = 1 - modu2 * (1.0 / 6.0 - modu2 / 120);
		b = 0.5 - modu2 * (1.0 / 24.0 - modu2 / 720);
	}
	else
	{
		a = sin(modu) / modu;
		b = (1 - cos(modu)) / modu2;
	}

	vect3d_askew(v, mat_temp1);

	mat_multi(mat_temp1[0], mat_temp1[0], mat_temp2[0], 3, 3, 3);

	mat_multi_num(mat_temp2[0], b, mat_temp3[0], 3, 3);

	mat_multi_num(mat_temp1[0], a, mat_temp4[0], 3, 3);

	mat_plus(mat_temp3[0], mat_temp4[0], mat_temp5[0], 3, 3);

	mat_unit(mat_temp6[0], 3);

	mat_plus(mat_temp6[0], mat_temp5[0], mat[0], 3, 3);
}
