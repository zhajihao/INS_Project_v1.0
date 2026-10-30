/**
 * @file navi_matrix.c
 * @brief 通用矩阵运算库
 *
 * 提供矩阵乘法、转置、加减、求逆等基础运算，
 * 供卡尔曼滤波和导航解算调用。
 */

#include "navi_math/navi_matrix.h"
#include <math.h>

/*                    c[m][p]=a[m][n]*b[n][p]                             */

/**
 * @brief  矩阵乘法 c = a*b（a为m×n，b为n×p，c为m×p）
 * @param  a 输入矩阵a，m行n列
 * @param  b 输入矩阵b，n行p列
 * @param  c 输出矩阵c，m行p列（结果写入）
 * @param  m 矩阵a、c的行数
 * @param  n 矩阵a的列数（矩阵b的行数）
 * @param  p 矩阵b、c的列数
 * @retval void
 * @note   矩阵按行优先一维线性索引；输出c不可与输入a、b指向同一内存
 */
void mat_multi(double *a, double *b, double *c, int m, int n, int p)
{
	int i, j, k;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < p; j++)
		{
			*(c + i * p + j) = 0.0; // 数组清零
			for (k = 0; k < n; k++)
			{
				*(c + i * p + j) += *(a + i * n + k) * (*(b + k * p + j));
			}
		}
	}
}

/*                    c[m][p]=a[m][n]*b[n][p]（float32运算）        */

/**
 * @brief  单精度矩阵乘法 c = a*b（a为m×n，b为n×p，c为m×p）
 * @param  a 输入矩阵a（float），m行n列
 * @param  b 输入矩阵b（float），n行p列
 * @param  c 输出矩阵c（float），m行p列（结果写入）
 * @param  m 矩阵a、c的行数
 * @param  n 矩阵a的列数（矩阵b的行数）
 * @param  p 矩阵b、c的列数
 * @retval void
 * @note   行优先一维线性索引；Cortex-M4F 具备单精度硬件浮点，内层编译为 VMLA.F32，
 *         故 15 维 Kalman 的稠密乘法全部走此函数；输出c不可与输入a、b指向同一内存
 */
void mat_multi_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n, int p)
{
	int i, j, k;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < p; j++)
		{
			*(c + i * p + j) = 0.0f; // 数组清零
			for (k = 0; k < n; k++)
			{
				*(c + i * p + j) += *(a + i * n + k) * (*(b + k * p + j));
			}
		}
	}
}

/**
 * @brief  单精度矩阵加法 c = a+b（a、b、c均为m×n同型矩阵）
 * @retval void
 * @note   行优先一维线性索引
 */
void mat_plus_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(c + i * n + j) = (*(a + i * n + j)) + (*(b + i * n + j));
		}
	}
}

/**
 * @brief  单精度矩阵减法 c = a-b（a、b、c均为m×n同型矩阵）
 * @retval void
 * @note   行优先一维线性索引
 */
void mat_minus_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(c + i * n + j) = (*(a + i * n + j)) - (*(b + i * n + j));
		}
	}
}

/**
 * @brief  单精度矩阵复制 b = a（a、b均为m×n矩阵）
 * @retval void
 * @note   行优先一维线性索引，逐元素拷贝
 */
void mat_assign_f(const float *__restrict a, float *__restrict b, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(b + i * n + j) = *(a + i * n + j);
		}
	}
}

/**
 * @brief  单精度矩阵转置 b = a^T（a为m×n，b为n×m）
 * @retval void
 * @note   行优先一维线性索引
 */
void mat_transpose_f(const float *__restrict a, float *__restrict b, int m, int n)
{
	int i, j;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < m; j++)
		{
			*(b + i * m + j) = *(a + j * n + i);
		}
	}
}

/**
 * @brief  单精度单位矩阵 a = I(m)（对角线为1，其余为0）
 * @retval void
 * @note   行优先一维线性索引
 */
void mat_unit_f(float *__restrict a, int m)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			if (i == j)
			{
				*(a + i * m + j) = 1.0f;
			}
			else
			{
				*(a + i * m + j) = 0.0f;
			}
		}
	}
}

/*                    c[m][n]=a[m][n]*b                             */

/**
 * @brief  矩阵数乘 c = a*b（a为m×n矩阵，b为标量）
 * @param  a 输入矩阵a，m行n列
 * @param  b 标量乘数b
 * @param  c 输出矩阵c，m行n列（结果写入）
 * @param  m 矩阵行数
 * @param  n 矩阵列数
 * @retval void
 * @note   矩阵按行优先一维线性索引
 */
void mat_multi_num(double *a, double b, double *c, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(c + i * n + j) = (*(a + i * n + j)) * b;
		}
	}
}

/*                    c[m][n]=a[m][n]+b[m][n]                             */

/**
 * @brief  矩阵加法 c = a+b（a、b、c均为m×n同型矩阵）
 * @param  a 输入矩阵a，m行n列
 * @param  b 输入矩阵b，m行n列
 * @param  c 输出矩阵c，m行n列（结果写入）
 * @param  m 矩阵行数
 * @param  n 矩阵列数
 * @retval void
 * @note   矩阵按行优先一维线性索引
 */
void mat_plus(double *a, double *b, double *c, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(c + i * n + j) = (*(a + i * n + j)) + (*(b + i * n + j));
		}
	}
}

/*                    c[m][n]=a[m][n]-b[m][n]                             */

/**
 * @brief  矩阵减法 c = a-b（a、b、c均为m×n同型矩阵）
 * @param  a 输入被减矩阵a，m行n列
 * @param  b 输入减数矩阵b，m行n列
 * @param  c 输出矩阵c，m行n列（结果写入）
 * @param  m 矩阵行数
 * @param  n 矩阵列数
 * @retval void
 * @note   矩阵按行优先一维线性索引
 */
void mat_minus(double *a, double *b, double *c, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(c + i * n + j) = (*(a + i * n + j)) - (*(b + i * n + j));
		}
	}
}

/*                    b[m][n]=a[m][n]                             */

/**
 * @brief  矩阵复制 b = a（a、b均为m×n矩阵）
 * @param  a 输入源矩阵a，m行n列
 * @param  b 输出目标矩阵b，m行n列（结果写入）
 * @param  m 矩阵行数
 * @param  n 矩阵列数
 * @retval void
 * @note   矩阵按行优先一维线性索引，逐元素拷贝
 */
void mat_assign(double *a, double *b, int m, int n)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			*(b + i * n + j) = *(a + i * n + j);
		}
	}
}

/*                    b[n][m]=a[m][n]                             */

/**
 * @brief  矩阵转置 b = a^T（a为m×n，b为n×m）
 * @param  a 输入矩阵a，m行n列
 * @param  b 输出矩阵b，n行m列（结果写入）
 * @param  m 矩阵a的行数（矩阵b的列数）
 * @param  n 矩阵a的列数（矩阵b的行数）
 * @retval void
 * @note   矩阵按行优先一维线性索引
 */
void mat_transpose(double *a, double *b, int m, int n)
{
	int i, j;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < m; j++)
		{
			*(b + i * m + j) = *(a + j * n + i);
		}
	}
}

/*                    a[m][m]=eye(m)                            */

/**
 * @brief  生成m阶单位矩阵 a = I(m)（对角线元素为1，其余为0）
 * @param  a 输出矩阵a，m行m列（结果写入）
 * @param  m 方阵阶数
 * @retval void
 * @note   矩阵按行优先一维线性索引
 */
void mat_unit(double *a, int m)
{
	int i, j;

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			if (i == j)
			{
				*(a + i * m + j) = 1;
			}
			else
			{
				*(a + i * m + j) = 0;
			}
		}
	}
}

/**
 * @brief  3阶方阵求逆 mat2 = mat1^{-1}（伴随矩阵法）
 * @param  mat1 输入3×3矩阵
 * @param  mat2 输出3×3逆矩阵（结果写入）
 * @retval void
 * @note   先求代数余子式矩阵，转置得伴随矩阵，再除以行列式；要求mat1行列式非零
 */
void mat3d_inverse(double mat1[3][3], double mat2[3][3])
{
	double mat_temp1[3][3], mat_temp2[3][3];
	double det = {0};

	mat_temp1[0][0] = (mat1[1][1] * mat1[2][2] - mat1[2][1] * mat1[1][2]);
	mat_temp1[0][1] = -(mat1[1][0] * mat1[2][2] - mat1[2][0] * mat1[1][2]);
	mat_temp1[0][2] = (mat1[1][0] * mat1[2][1] - mat1[2][0] * mat1[1][1]);

	mat_temp1[1][0] = -(mat1[0][1] * mat1[2][2] - mat1[2][1] * mat1[0][2]);
	mat_temp1[1][1] = (mat1[0][0] * mat1[2][2] - mat1[2][0] * mat1[0][2]);
	mat_temp1[1][2] = -(mat1[0][0] * mat1[2][1] - mat1[2][0] * mat1[0][1]);

	mat_temp1[2][0] = (mat1[0][1] * mat1[1][2] - mat1[1][1] * mat1[0][2]);
	mat_temp1[2][1] = -(mat1[0][0] * mat1[1][2] - mat1[1][0] * mat1[0][2]);
	mat_temp1[2][2] = (mat1[0][0] * mat1[1][1] - mat1[1][0] * mat1[0][1]);

	mat_transpose(mat_temp1[0], mat_temp2[0], 3, 3); // 计算伴随矩阵

	for (int i = 0; i < 3; i++)
	{
		det = det + mat1[i][0] * mat_temp1[i][0];
	} // 计算行列式

	mat_multi_num(mat_temp2[0], 1.0 / det, mat2[0], 3, 3);
}

/**
 * @brief  2阶方阵求逆 mat2 = mat1^{-1}（伴随矩阵法）
 * @param  mat1 输入2×2矩阵
 * @param  mat2 输出2×2逆矩阵（结果写入）
 * @retval void
 * @note   伴随矩阵除以行列式 det=mat1[0][0]*mat1[1][1]-mat1[0][1]*mat1[1][0]，要求行列式非零
 */
void mat2d_inverse(double mat1[2][2], double mat2[2][2])
{
	double mat_temp1[2][2], mat_temp2[2][2];
	double det = {0};

	mat_temp1[0][0] = mat1[1][1];
	mat_temp1[0][1] = -mat1[1][0];
	mat_temp1[1][0] = -mat1[0][1];
	mat_temp1[1][1] = mat1[0][0];

	mat_transpose(mat_temp1[0], mat_temp2[0], 2, 2); // 计算伴随矩阵

	det = mat1[0][0] * mat1[1][1] - mat1[0][1] * mat1[1][0]; // 计算行列式

	mat_multi_num(mat_temp2[0], 1.0 / det, mat2[0], 2, 2);
}
