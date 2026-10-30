/**
 * @file navi_error_comp.c
 * @brief IMU 误差补偿（标定/温度补偿）
 *
 * 按装订参数补偿陀螺和加表的标度因数、常值零偏、安装误差。
 * 温补级与标定级共用同一接口，仅传入参数不同。
 */

#include "navi_error_comp.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include <math.h>

/**
 * @brief  误差补偿 ErrorComp: 按装订参数补偿陀螺/加表的标度因数(误差+标度)、
 *         常值零偏(漂移)和安装误差。温补级与标定级共用本函数, 仅传入的 errpara 不同。
 * @param  errpara     输入: 30 元素误差参数(打包布局见下)
 * @param  gyrodata_BC 输入: 补偿前陀螺角增量 [x,y,z](rad/帧)
 * @param  accdata_BC  输入: 补偿前加表速度增量 [x,y,z](m/s·帧)
 * @param  gyrodata_AC 输出: 补偿后陀螺角增量 [x,y,z](rad/帧)
 * @param  accdata_AC  输出: 补偿后加表速度增量 [x,y,z](m/s·帧)
 * @param  ts          输入: 单帧采样时间 (s), 用于把零偏速率换算为增量
 * @retval void
 *
 * @note   errpara[30] 打包布局(与 GetErrorPara_TC 写入顺序一致):
 *   陀螺组 [0..14]:
 *     [0,4,8]  标度因数误差 -> dEg 对角
 *     [1,2,3,5,6,7] 安装误差 -> dEg 非对角 [01,02,10,12,20,21]
 *     [9,10,11] 标度因数 -> E1g 对角
 *     [12,13,14] 常值漂移 -> eb(rad/s)
 *   加表组 [15..29]:
 *     [15,19,23] 标度因数误差 -> dKa 对角
 *     [16,17,18,20,21,22] 安装误差 -> dKa 非对角
 *     [24,25,26] 标度因数 -> K1a 对角
 *     [27,28,29] 零偏 -> db(m/s²)
 *
 *   补偿公式:
 *     data_gyro = (I - dEg) · E1g · gyro_BC - eb·ts
 *     data_acc  = (I - dKa) · K1a · acc_BC  - db·ts
 *   dEg/dKa 对角承载"标度误差"、非对角承载"安装误差", (I - 误差阵) 一次完成
 *   标度误差修正与安装去耦, 再乘标度因数, 最后减零偏增量。
 */
void ErrorComp(double errpara[30], double gyrodata_BC[3], double accdata_BC[3], double gyrodata_AC[3], double accdata_AC[3], double ts)
{
	/* 误差/标度矩阵与零偏向量(全部初始化为0) */
	double E1g[3][3] = {0}, K1a[3][3] = {0}, dEg[3][3] = {0}, dKa[3][3] = {0};
	double eb[3] = {0}, db[3] = {0};

	/* 矩阵/向量运算临时量 */
	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0}, mat_temp3[3][3] = {0}, mat_temp4[3][3] = {0}, mat_temp5[3][3] = {0};
	double v_temp1[3] = {0}, v_temp2[3] = {0}, v_temp3[3] = {0}, v_temp4[3] = {0};

	/* ===== 陀螺参数解包 ===== */
	// 陀螺标度因数误差(对角)
	dEg[0][0] = errpara[0];
	dEg[1][1] = errpara[4];
	dEg[2][2] = errpara[8];

	// 陀螺安装误差(非对角6元素)
	dEg[0][1] = errpara[1];
	dEg[0][2] = errpara[2];
	dEg[1][0] = errpara[3];
	dEg[1][2] = errpara[5];
	dEg[2][0] = errpara[6];
	dEg[2][1] = errpara[7];

	// 陀螺标度因数(对角)
	E1g[0][0] = errpara[9];
	E1g[1][1] = errpara[10];
	E1g[2][2] = errpara[11];

	// 陀螺常值漂移(零偏, 速率 rad/s)
	eb[0] = errpara[12];
	eb[1] = errpara[13];
	eb[2] = errpara[14];

	/* ===== 加表参数解包 ===== */
	// 加表标度因数误差(对角)
	dKa[0][0] = errpara[15];
	dKa[1][1] = errpara[19];
	dKa[2][2] = errpara[23];

	// 加表安装误差(非对角6元素)
	dKa[0][1] = errpara[16];
	dKa[0][2] = errpara[17];
	dKa[1][0] = errpara[18];
	dKa[1][2] = errpara[20];
	dKa[2][0] = errpara[21];
	dKa[2][1] = errpara[22];

	// 加表标度因数(对角)
	K1a[0][0] = errpara[24];
	K1a[1][1] = errpara[25];
	K1a[2][2] = errpara[26];

	// 加表零偏(比力 m/s²)
	db[0] = errpara[27];
	db[1] = errpara[28];
	db[2] = errpara[29];

	/* ===== 陀螺补偿: (I-dEg)·E1g·gyro_BC - eb·ts ===== */

	mat_unit(mat_temp1[0], 3); // mat_temp1=eye(3) 单位阵

	mat_minus(mat_temp1[0], dEg[0], mat_temp2[0], 3, 3);	// mat_temp2 = I - dEg(标度误差修正+安装去耦)
	mat_multi(mat_temp2[0], E1g[0], mat_temp3[0], 3, 3, 3); // mat_temp3 = (I-dEg)·E1g
	mat_multi(mat_temp3[0], gyrodata_BC, v_temp1, 3, 3, 1); // v_temp1 = (I-dEg)·E1g·gyro_BC
	mat_multi_num(eb, ts, v_temp2, 3, 1);					// v_temp2 = eb·ts(零偏速率转增量)
	mat_minus(v_temp1, v_temp2, gyrodata_AC, 3, 1);			// gyrodata_AC = v_temp1 - eb·ts

	/* ===== 加表补偿: (I-dKa)·K1a·acc_BC - db·ts ===== */

	mat_minus(mat_temp1[0], dKa[0], mat_temp4[0], 3, 3);	// mat_temp4 = I - dKa(标度误差修正+安装去耦)
	mat_multi(mat_temp4[0], K1a[0], mat_temp5[0], 3, 3, 3); // mat_temp5 = (I-dKa)·K1a
	mat_multi(mat_temp5[0], accdata_BC, v_temp3, 3, 3, 1);	// v_temp3 = (I-dKa)·K1a·acc_BC
	mat_multi_num(db, ts, v_temp4, 3, 1);					// v_temp4 = db·ts(零偏比力转增量)
	mat_minus(v_temp3, v_temp4, accdata_AC, 3, 1);			// accdata_AC = v_temp3 - db·ts
}

/**
 * @brief  按温度 4 次多项式解算 IMU 标度因数与零偏(常值漂移)的温补结果
 * @param  p_Imu_TempCompPara 输入: 温补多项式系数(aT_E1x/aT_K1x 等标度系数、
 *                            aT_ebx/aT_dbx 等零偏系数, 各 5 阶 [a0..a4])
 * @param  p_Imu_Infor        输入: T_gyro/T_acc 三轴温度(℃);
 *                            输出: errpara_TC[30] 温补结果(打包布局与 ErrorComp 一致)
 * @retval void
 *
 * @note   算法: 先构造温度回归向量 v=[1,T,T²,T³,T⁴], 再以
 *         p(T) = v · aT_xxx 求各误差在当前温度下的取值(行向量×列向量=标量)。
 *
 *         errpara_TC 写入布局(与 ErrorComp 解包逐项对齐):
 *           陀螺: 标度因数 E1g -> [9,10,11]; 常值漂移 eb -> [12,13,14]
 *           加表: 标度因数 K1a -> [24,25,26]; 零偏 db -> [27,28,29]
 *
 *         当前局限: 标度因数误差([0,4,8]/[15,19,23])与安装误差(非对角共12项)
 *         尚未在本函数解算, p_Imu_TempCompPara 中对应的 dE 类与 dK 类系数未被引用;
 *         这些下标依赖上层 memset 保持 0(等价不修正), 后续扩展时在此补写。
 */
void GetErrorPara_TC(struct Imu_TempCompPara *p_Imu_TempCompPara, struct Imu_Infor *p_Imu_Infor)
{

	/* 6 个温度回归向量: v_temp1..3 对应三轴陀螺温度, v_temp4..6 对应三轴加表温度 */
	double v_temp1[5] = {0}, v_temp2[5] = {0}, v_temp3[5] = {0}, v_temp4[5] = {0}, v_temp5[5] = {0}, v_temp6[5] = {0};

	/* ===== 构造陀螺温度回归向量 [1, T, T², T³, T⁴] ===== */
	/* X 陀螺 */
	v_temp1[0] = 1;
	v_temp1[1] = (*p_Imu_Infor).T_gyro[0];
	v_temp1[2] = v_temp1[1] * v_temp1[1]; // T^2 = T*T
	v_temp1[3] = v_temp1[2] * v_temp1[1]; // T^3 = T^2*T
	v_temp1[4] = v_temp1[2] * v_temp1[2]; // T^4 = T^2*T^2

	/* Y 陀螺 */
	v_temp2[0] = 1;
	v_temp2[1] = (*p_Imu_Infor).T_gyro[1];
	v_temp2[2] = v_temp2[1] * v_temp2[1]; // T^2 = T*T
	v_temp2[3] = v_temp2[2] * v_temp2[1]; // T^3 = T^2*T
	v_temp2[4] = v_temp2[2] * v_temp2[2]; // T^4 = T^2*T^2

	/* Z 陀螺 */
	v_temp3[0] = 1;
	v_temp3[1] = (*p_Imu_Infor).T_gyro[2];
	v_temp3[2] = v_temp3[1] * v_temp3[1]; // T^2 = T*T
	v_temp3[3] = v_temp3[2] * v_temp3[1]; // T^3 = T^2*T
	v_temp3[4] = v_temp3[2] * v_temp3[2]; // T^4 = T^2*T^2

	/* ===== 构造加表温度回归向量 [1, T, T², T³, T⁴] ===== */
	/* X 加表 */
	v_temp4[0] = 1;
	v_temp4[1] = (*p_Imu_Infor).T_acc[0];
	v_temp4[2] = v_temp4[1] * v_temp4[1]; // T^2 = T*T
	v_temp4[3] = v_temp4[2] * v_temp4[1]; // T^3 = T^2*T
	v_temp4[4] = v_temp4[2] * v_temp4[2]; // T^4 = T^2*T^2

	/* Y 加表 */
	v_temp5[0] = 1;
	v_temp5[1] = (*p_Imu_Infor).T_acc[1];
	v_temp5[2] = v_temp5[1] * v_temp5[1]; // T^2 = T*T
	v_temp5[3] = v_temp5[2] * v_temp5[1]; // T^3 = T^2*T
	v_temp5[4] = v_temp5[2] * v_temp5[2]; // T^4 = T^2*T^2

	/* Z 加表 */
	v_temp6[0] = 1;
	v_temp6[1] = (*p_Imu_Infor).T_acc[2];
	v_temp6[2] = v_temp6[1] * v_temp6[1]; // T^2 = T*T
	v_temp6[3] = v_temp6[2] * v_temp6[1]; // T^3 = T^2*T
	v_temp6[4] = v_temp6[2] * v_temp6[2]; // T^4 = T^2*T^2

	/* ===== 多项式求值并写入 errpara_TC (v 行向量 x 系数列向量 = 标量) ===== */

	/* 陀螺标度因数 E1g -> [9,10,11] */
	mat_multi(v_temp1, (*p_Imu_TempCompPara).aT_E1x, &((*p_Imu_Infor).errpara_TC[9]), 1, 5, 1);	 // 计算x陀螺标度因数温补后结果
	mat_multi(v_temp2, (*p_Imu_TempCompPara).aT_E1y, &((*p_Imu_Infor).errpara_TC[10]), 1, 5, 1); // 计算y陀螺标度因数温补后结果
	mat_multi(v_temp3, (*p_Imu_TempCompPara).aT_E1z, &((*p_Imu_Infor).errpara_TC[11]), 1, 5, 1); // 计算z陀螺标度因数温补后结果

	/* 加表标度因数 K1a -> [24,25,26] */
	mat_multi(v_temp4, (*p_Imu_TempCompPara).aT_K1x, &((*p_Imu_Infor).errpara_TC[24]), 1, 5, 1); // 计算x加表标度因数温补后结果
	mat_multi(v_temp5, (*p_Imu_TempCompPara).aT_K1y, &((*p_Imu_Infor).errpara_TC[25]), 1, 5, 1); // 计算y加表标度因数温补后结果
	mat_multi(v_temp6, (*p_Imu_TempCompPara).aT_K1z, &((*p_Imu_Infor).errpara_TC[26]), 1, 5, 1); // 计算z加表标度因数温补后结果

	/* 陀螺常值漂移 eb -> [12,13,14] */
	mat_multi(v_temp1, (*p_Imu_TempCompPara).aT_ebx, &((*p_Imu_Infor).errpara_TC[12]), 1, 5, 1); // 计算x陀螺常值漂移温补后结果
	mat_multi(v_temp2, (*p_Imu_TempCompPara).aT_eby, &((*p_Imu_Infor).errpara_TC[13]), 1, 5, 1); // 计算y陀螺常值漂移温补后结果
	mat_multi(v_temp3, (*p_Imu_TempCompPara).aT_ebz, &((*p_Imu_Infor).errpara_TC[14]), 1, 5, 1); // 计算z陀螺常值漂移温补后结果

	/* 加表零偏 db -> [27,28,29] */
	mat_multi(v_temp4, (*p_Imu_TempCompPara).aT_dbx, &((*p_Imu_Infor).errpara_TC[27]), 1, 5, 1); // 计算x加表零偏温补后结果
	mat_multi(v_temp5, (*p_Imu_TempCompPara).aT_dby, &((*p_Imu_Infor).errpara_TC[28]), 1, 5, 1); // 计算y加表零偏温补后结果
	mat_multi(v_temp6, (*p_Imu_TempCompPara).aT_dbz, &((*p_Imu_Infor).errpara_TC[29]), 1, 5, 1); // 计算z加表零偏温补后结果
}

/**
 * @brief  MEMS器件参数温度补偿(占位空函数,未实现)
 * @param  p_ 预留参数指针
 * @retval void
 */
void Para_TemCompMEMS(double *p_)
{
}
