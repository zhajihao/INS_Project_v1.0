/**
 * @file mat_inverse.c
 * @brief 方阵求逆（高斯-约旦消元法，float 版）
 *
 * 增广矩阵改为 static BSS 数组, 最大支持 MAX_MAT_INV_N 阶, 消除 malloc/free 依赖.
 * 全 float 运算, 复用 Cortex-M4F 单精度 FPU, 避免软件浮点.
 * 奇异矩阵返回错误码.
 */

#include <math.h>

#define EPS 1e-10f
#define MAX_MAT_INV_N  19   /* 最大支持阶数 (实际 navi 用到 n=7, 留裕量) */

/* 静态 BSS 增广矩阵: 最大 19×38 = 722 个 float ≈ 2.9 KB (原 double 版 5.8 KB) */
static float aug[MAX_MAT_INV_N * 2 * MAX_MAT_INV_N];

/**
 * @brief 高斯约旦消元求方阵逆矩阵 (float 版, Cortex-M4F FPU 加速)
 * @param n      矩阵阶数 (1 ~ MAX_MAT_INV_N)
 * @param A      输入原矩阵, 大小 n*n
 * @param A_inv  输出逆矩阵, 大小 n*n
 * @return 0成功, -1奇异矩阵, -2阶数非法或超出 MAX_MAT_INV_N
 */
int mat_inverse(int n, float *A, float *A_inv)
{
	if (n <= 0 || n > MAX_MAT_INV_N) return -2;
	int i, j, k;

	/* 初始化增广矩阵 [A | I], 放在 static aug 中 */
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
			aug[i * 2 * n + j] = A[i * n + j];
		for (j = n; j < 2 * n; j++)
			aug[i * 2 * n + j] = (j - n == i) ? 1.0f : 0.0f;
	}

	/* 高斯约旦主元消去 */
	for (k = 0; k < n; k++)
	{
		/* 1. 选主元 */
		int pivot = k;
		for (i = k; i < n; i++)
		{
			if (fabsf(aug[i * 2 * n + k]) > fabsf(aug[pivot * 2 * n + k]))
				pivot = i;
		}
		/* 主元接近 0, 奇异矩阵无逆 */
		if (fabsf(aug[pivot * 2 * n + k]) < EPS)
		{
			return -1;
		}
		/* 交换当前行 k 和主元行 pivot */
		if (pivot != k)
		{
			for (j = k; j < 2 * n; j++)
			{
				float tmp = aug[k * 2 * n + j];
				aug[k * 2 * n + j] = aug[pivot * 2 * n + j];
				aug[pivot * 2 * n + j] = tmp;
			}
		}
		/* 2. 主元行归一化, 使 aug[k][k]=1 */
		float div = aug[k * 2 * n + k];
		for (j = k; j < 2 * n; j++)
			aug[k * 2 * n + j] /= div;

		/* 3. 消去其余所有行的第 k 列 */
		for (i = 0; i < n; i++)
		{
			if (i != k && fabsf(aug[i * 2 * n + k]) > EPS)
			{
				float factor = aug[i * 2 * n + k];
				for (j = k; j < 2 * n; j++)
				{
					aug[i * 2 * n + j] -= factor * aug[k * 2 * n + j];
				}
			}
		}
	}
	/* 提取右侧部分作为逆矩阵 */
	for (i = 0; i < n; i++)
		for (j = 0; j < n; j++)
			A_inv[i * n + j] = aug[i * 2 * n + n + j];

	return 0;
}
