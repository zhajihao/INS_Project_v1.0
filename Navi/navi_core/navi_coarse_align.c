/**
 * @file navi_coarse_align.c
 * @brief 粗对准:双矢量解析对准、惯性基座对准、水平对准
 *
 * 由 coarsealign_2vect.c / coarsealign_2vect__RT.c / coarsealign_inertial.c /
 * coarsealign_inertialRT.c / CoarseAlign_Horizon.c 合并,仅改写 #include 路径,
 * 算法逻辑逐字节保留。cnscl 等跨模块依赖通过 navi_ins.h 引入。
 */
#include "navi_coarse_align.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include "navi_attitude.h"
#include "navi_ins.h"
#include <math.h>

/**
 * @brief  双矢量解析粗对准:利用平均重力矢量与地球自转角速度矢量正交化构建机体到导航系方向余弦矩阵cnb
 * @param  average_accdata  对准时间内平均比力(m/s^2)
 * @param  average_gyrodata 对准时间内平均角速度(rad/s)
 * @param  cnb              出参,机体系(b)到导航系(n)方向余弦矩阵
 * @retval void
 * @note   align1=average_accdata×average_gyrodata,align2=align1×average_accdata,align3=average_accdata;三者归一化后依次作为cnb第1行(取负)、第2行、第3行
 */
void coarsealign_2vect(double average_accdata[3], double average_gyrodata[3], double cnb[3][3])

{

	double align1[3], align2[3], align3[3];
	double align1_mod, align2_mod, align3_mod;

	vect3d_cross_vect3d(average_accdata, average_gyrodata, align1);
	align1_mod = sqrt(pow(align1[0], 2) + pow(align1[1], 2) + pow(align1[2], 2));

	vect3d_cross_vect3d(align1, average_accdata, align2);
	align2_mod = sqrt(pow(align2[0], 2) + pow(align2[1], 2) + pow(align2[2], 2));

	vect3d_assign(average_accdata[0], average_accdata[1], average_accdata[2], align3);
	align3_mod = sqrt(pow(align3[0], 2) + pow(align3[1], 2) + pow(align3[2], 2));

	cnb[0][0] = -align1[0] / align1_mod;
	cnb[0][1] = -align1[1] / align1_mod;
	cnb[0][2] = -align1[2] / align1_mod;

	cnb[1][0] = align2[0] / align2_mod;
	cnb[1][1] = align2[1] / align2_mod;
	cnb[1][2] = align2[2] / align2_mod;

	cnb[2][0] = align3[0] / align3_mod;
	cnb[2][1] = align3[1] / align3_mod;
	cnb[2][2] = align3[2] / align3_mod;
}
/**
 * @brief  双矢量解析粗对准实时版:在t_align/4时间内累加IMU数据求平均,再按双矢量解析法确定初始姿态
 * @param  p_imu_infor  IMU信息:输入data_acc(m/s)/data_gyro(rad);输出cnb、att(姿态角rad)
 * @param  p_sys_infor  系统信息:t_align对准时长(s),完成后置flag_Finish_calign=1
 * @retval void
 * @note   平均量按累加值/(kk*ts)换算为比力(m/s^2)与角速度(rad/s);对准完成后清flag_ValidIMU=0,防止后续复用对准期数据
 */
void coarsealign_2vect_RT(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)

{
	static double sum_accdata[3] = {0}, sum_gyrodata[3] = {0}, average_accdata[3] = {0}, average_gyrodata[3] = {0};

	static double kk = 0;
	double align1[3] = {0}, align2[3] = {0}, align3[3] = {0};
	double align1_mod = 0, align2_mod = 0, align3_mod = 0;

	if (kk < 1.0 / 4.0 * (*p_sys_infor).t_align * (*p_imu_infor).fs)

	{
		mat_plus(sum_accdata, (*p_imu_infor).data_acc, sum_accdata, 3, 1);
		mat_plus(sum_gyrodata, (*p_imu_infor).data_gyro, sum_gyrodata, 3, 1);
		kk++;
	}

	if (fabs(kk - 1.0 / 4.0 * (*p_sys_infor).t_align * (*p_imu_infor).fs) < 1e-10)
	{
		mat_multi_num(sum_accdata, 1 / (kk * (*p_imu_infor).ts), average_accdata, 3, 1);
		mat_multi_num(sum_gyrodata, 1 / (kk * (*p_imu_infor).ts), average_gyrodata, 3, 1);

		vect3d_cross_vect3d(average_accdata, average_gyrodata, align1);
		align1_mod = sqrt(pow(align1[0], 2) + pow(align1[1], 2) + pow(align1[2], 2));

		vect3d_cross_vect3d(align1, average_accdata, align2);
		align2_mod = sqrt(pow(align2[0], 2) + pow(align2[1], 2) + pow(align2[2], 2));

		vect3d_assign(average_accdata[0], average_accdata[1], average_accdata[2], align3);
		align3_mod = sqrt(pow(align3[0], 2) + pow(align3[1], 2) + pow(align3[2], 2));

		(*p_imu_infor).cnb[0][0] = -align1[0] / align1_mod;
		(*p_imu_infor).cnb[0][1] = -align1[1] / align1_mod;
		(*p_imu_infor).cnb[0][2] = -align1[2] / align1_mod;

		(*p_imu_infor).cnb[1][0] = align2[0] / align2_mod;
		(*p_imu_infor).cnb[1][1] = align2[1] / align2_mod;
		(*p_imu_infor).cnb[1][2] = align2[2] / align2_mod;

		(*p_imu_infor).cnb[2][0] = align3[0] / align3_mod;
		(*p_imu_infor).cnb[2][1] = align3[1] / align3_mod;
		(*p_imu_infor).cnb[2][2] = align3[2] / align3_mod;

		m2att((*p_imu_infor).cnb, (*p_imu_infor).att);
		kk = 0;

		(*p_sys_infor).flag_Finish_calign = 1;

		// 为了避免接下来的函数重复使用粗对准的IMU数据，粗对准完成后将IMU数据有效位置0，重新读取或产生IMU数据
		(*p_imu_infor).flag_ValidIMU = 0;
	}
}
/**
 * @brief  惯性基座粗对准单步更新(离线版):双子样圆锥划桨补偿后,递推机体相对初始机体的矩阵cb0b,并在初始机体系/初始导航系积分比力Fb与重力Gn
 * @param  data_acc1  第1子样速度增量(m/s)
 * @param  data_acc2  第2子样速度增量(m/s)
 * @param  data_gyro1 第1子样角增量(rad)
 * @param  data_gyro2 第2子样角增量(rad)
 * @param  pos        当前位置[纬度rad,经度rad,高度m]
 * @param  p_ethi     地球参数(gn重力矢量等)
 * @param  cb0b       出参,当前机体系相对初始机体系b0的方向余弦矩阵
 * @param  cn0n       出参,当前导航系相对初始导航系n0的方向余弦矩阵(仅含地球自转)
 * @param  Fb         出参,b0系比力积分(m/s)
 * @param  Gn         出参,n0系重力积分,随时间扣除(m/s)
 * @param  pointer_t  累计对准时间指针(输入当前值,输出加nts后的值,s)
 * @param  nts        双子样补偿时间间隔(s)
 * @retval void
 * @note   cn0n由纬度与地球自转角wie*t解析构造,静止基座下成立
 */
void coarsealign_inertial(double data_acc1[3], double data_acc2[3], double data_gyro1[3], double data_gyro2[3], double pos[3], struct Ethi *p_ethi, double cb0b[3][3], double cn0n[3][3], double Fb[3], double Gn[3], double *pointer_t, double nts)
{

	double phim[3] = {0}, dvbm[3] = {0};

	double mat_temp1[3][3], mat_temp2[3][3];
	double v_temp1[3], v_temp2[3], v_temp3[3], v_temp4[3], v_temp5[3];

	*pointer_t = (*pointer_t) + nts;
	cnscl(data_gyro1, data_gyro2, data_acc1, data_acc2, phim, dvbm);

	rv2mat(phim, mat_temp1);
	mat_multi(cb0b[0], mat_temp1[0], mat_temp2[0], 3, 3, 3);
	mat_assign(mat_temp2[0], cb0b[0], 3, 3);

	mat_multi(cb0b[0], dvbm, v_temp1, 3, 3, 1);
	mat_plus(Fb, v_temp1, v_temp2, 3, 1);
	mat_assign(v_temp2, Fb, 3, 1);

	cn0n[0][0] = cos(wie * (*pointer_t));
	cn0n[0][1] = -sin(wie * (*pointer_t)) * sin(pos[0]);
	cn0n[0][2] = sin(wie * (*pointer_t)) * cos(pos[0]);

	cn0n[1][0] = sin(wie * (*pointer_t)) * sin(pos[0]);
	cn0n[1][1] = 1 - (1 - cos(wie * (*pointer_t))) * pow(sin(pos[0]), 2);
	cn0n[1][2] = (1 - cos(wie * (*pointer_t))) * sin(pos[0]) * cos(pos[0]);

	cn0n[2][0] = -sin(wie * (*pointer_t)) * cos(pos[0]);
	cn0n[2][1] = (1 - cos(wie * (*pointer_t))) * sin(pos[0]) * cos(pos[0]);
	cn0n[2][2] = 1 - (1 - cos(wie * (*pointer_t))) * pow(cos(pos[0]), 2);

	mat_multi_num((*p_ethi).gn, nts, v_temp3, 3, 1);
	mat_multi(cn0n[0], v_temp3, v_temp4, 3, 3, 1);
	mat_minus(Gn, v_temp4, v_temp5, 3, 1);
	mat_assign(v_temp5, Gn, 3, 1);
}
/**
 * @brief  惯性基座粗对准实时版:t_align时间内实时积分Fb/Gn,在半程、全程两次取样,以双矢量法定姿并解算初始cnb
 * @param  p_imu_infor  IMU信息:输入data_acc/data_gyro;输出cnb、att(姿态角rad)
 * @param  p_ethi       地球参数(pos、gn)
 * @param  p_sys_infor  系统信息:t_align对准时长(s),对准完成置flag_Finish_calign=1
 * @retval void
 * @note   两次调用完成一个双子样更新;半程(cnt=t_align/2/nts)记录Fb01/Gn01,全程记录Fb02/Gn02,经G系、F系双矢量正交基求解cn0b0后,cnb=cn0n'*cn0b0*cb0b
 */
void coarsealign_inertialRT(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor)
{
	static unsigned short cnt = 0;
	static unsigned char kk = 0;
	static double fb1[3] = {0};
	static double fb2[3] = {0};
	static double wb1[3] = {0};
	static double wb2[3] = {0};

	static double Fb[3] = {0};
	static double Fb01[3] = {0};
	static double Fb02[3] = {0};

	static double Gn[3] = {0};
	static double Gn01[3] = {0};
	static double Gn02[3] = {0};

	static double cb0b[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	static double cn0n[3][3] = {0};

	static double t = 0.0;

	double phim[3] = {0};
	double dvbm[3] = {0};

	double mat_temp1[3][3], mat_temp2[3][3], mat_temp3[3][3], mat_temp4[3][3], mat_temp5[3][3], mat_temp6[3][3], mat_temp7[3][3];
	double v_temp1[3], v_temp2[3], v_temp3[3], v_temp4[3], v_temp5[3], v_temp6[3], v_temp7[3], v_temp8[3], v_temp9[3], v_temp10[3], v_temp11[3];

	double G1[3], G2[3], G3[3], F1[3], F2[3], F3[3];
	double cn0b0[3][3] = {0};

	if (kk == 0)
	{
		vect3d_assign((*p_imu_infor).data_acc[0], (*p_imu_infor).data_acc[1], (*p_imu_infor).data_acc[2], fb1);
		vect3d_assign((*p_imu_infor).data_gyro[0], (*p_imu_infor).data_gyro[1], (*p_imu_infor).data_gyro[2], wb1);
		kk = kk + 1;
		return;
	}
	else
	{
		kk = 0;

		cnt = cnt + 1;
		t = cnt * (*p_imu_infor).nts;

		vect3d_assign((*p_imu_infor).data_acc[0], (*p_imu_infor).data_acc[1], (*p_imu_infor).data_acc[2], fb2);
		vect3d_assign((*p_imu_infor).data_gyro[0], (*p_imu_infor).data_gyro[1], (*p_imu_infor).data_gyro[2], wb2);

		cnscl(wb1, wb2, fb1, fb2, phim, dvbm);

		rv2mat(phim, mat_temp1);
		mat_multi(cb0b[0], mat_temp1[0], mat_temp2[0], 3, 3, 3);
		mat_assign(mat_temp2[0], cb0b[0], 3, 3);

		mat_multi(cb0b[0], dvbm, v_temp1, 3, 3, 1);
		mat_plus(Fb, v_temp1, v_temp2, 3, 1);
		mat_assign(v_temp2, Fb, 3, 1);

		cn0n[0][0] = cos(wie * t);
		cn0n[0][1] = -sin(wie * t) * sin((*p_ethi).pos[0]);
		cn0n[0][2] = sin(wie * t) * cos((*p_ethi).pos[0]);

		cn0n[1][0] = sin(wie * t) * sin((*p_ethi).pos[0]);
		cn0n[1][1] = 1 - (1 - cos(wie * t)) * pow(sin((*p_ethi).pos[0]), 2);
		cn0n[1][2] = (1 - cos(wie * t)) * sin((*p_ethi).pos[0]) * cos((*p_ethi).pos[0]);

		cn0n[2][0] = -sin(wie * t) * cos((*p_ethi).pos[0]);
		cn0n[2][1] = (1 - cos(wie * t)) * sin((*p_ethi).pos[0]) * cos((*p_ethi).pos[0]);
		cn0n[2][2] = 1 - (1 - cos(wie * t)) * pow(cos((*p_ethi).pos[0]), 2);

		mat_multi_num((*p_ethi).gn, (*p_imu_infor).nts, v_temp3, 3, 1);
		mat_multi(cn0n[0], v_temp3, v_temp4, 3, 3, 1);
		mat_minus(Gn, v_temp4, v_temp5, 3, 1);
		mat_assign(v_temp5, Gn, 3, 1);
	}

	if (cnt == (*p_sys_infor).t_align / 2.0 / (*p_imu_infor).nts)
	{

		vect3d_assign(Fb[0], Fb[1], Fb[2], Fb01);
		vect3d_assign(Gn[0], Gn[1], Gn[2], Gn01);
	}

	if (cnt == (*p_sys_infor).t_align / (*p_imu_infor).nts)
	{

		vect3d_assign(Fb[0], Fb[1], Fb[2], Fb02);
		vect3d_assign(Gn[0], Gn[1], Gn[2], Gn02);

		vect3d_assign(Gn01[0], Gn01[1], Gn01[2], G1);
		vect3d_cross_vect3d(G1, Gn02, G2);
		vect3d_cross_vect3d(G2, G1, G3);

		vect3d_assign(Fb01[0], Fb01[1], Fb01[2], F1);
		vect3d_cross_vect3d(F1, Fb02, F2);
		vect3d_cross_vect3d(F2, F1, F3);

		mat_multi_num(G1, 1.0 / norm_3d(G1), v_temp6, 3, 1);
		mat_multi_num(G2, 1.0 / norm_3d(G2), v_temp7, 3, 1);
		mat_multi_num(G3, 1.0 / norm_3d(G3), v_temp8, 3, 1);

		for (int j = 0; j < 3; j++)
		{
			mat_temp3[j][0] = v_temp6[j];
		}

		for (int j = 0; j < 3; j++)
		{
			mat_temp3[j][1] = v_temp7[j];
		}

		for (int j = 0; j < 3; j++)
		{
			mat_temp3[j][2] = v_temp8[j];
		}

		mat_multi_num(F1, 1.0 / norm_3d(F1), v_temp9, 3, 1);
		mat_multi_num(F2, 1.0 / norm_3d(F2), v_temp10, 3, 1);
		mat_multi_num(F3, 1.0 / norm_3d(F3), v_temp11, 3, 1);

		for (int j = 0; j < 3; j++)
		{
			mat_temp4[j][0] = v_temp9[j];
		}

		for (int j = 0; j < 3; j++)
		{
			mat_temp4[j][1] = v_temp10[j];
		}

		for (int j = 0; j < 3; j++)
		{
			mat_temp4[j][2] = v_temp11[j];
		}

		mat_transpose(mat_temp4[0], mat_temp5[0], 3, 3);

		mat_multi(mat_temp3[0], mat_temp5[0], cn0b0[0], 3, 3, 3);

		mat_transpose(cn0n[0], mat_temp6[0], 3, 3);

		mat_multi(mat_temp6[0], cn0b0[0], mat_temp7[0], 3, 3, 3);

		mat_multi(mat_temp7[0], cb0b[0], (*p_imu_infor).cnb[0], 3, 3, 3);

		m2att((*p_imu_infor).cnb, (*p_imu_infor).att);

		(*p_sys_infor).flag_Finish_calign = 1;
	}
}
/**
 * @brief  AHRS水平粗对准实时版:t_HorAlign时间内平均加表/陀螺数据,以重力矢量为基准构造水平姿态矩阵
 * @param  p_imu_infor   IMU信息:输入data_acc/data_gyro、fs采样率
 * @param  p_AHRS_Infor  AHRS信息:t_HorAlign水平对准时长(s);输出average_accdata/average_gyrodata及其模值、cnb_align、att_align(rad)、Flag_HorAlignFinish
 * @retval void
 * @note   以归一化比力作为矩阵第3行(天向),选近正交的水平分量构造第2、1行;
 *         仅水平姿态(pitch/roll)可观测,航向不可观(平均陀螺仅存入average_gyrodata,不参与姿态解算);
 *         内部无完成判定,累加器sum_*与kk为static且无复位接口(重复对准会续算,重启需扩展复位接口);
 *         完成判定/结果拷贝由外部状态机处理 (navi_pipeline.c 的 imu_frame_process_all 步骤5);
 *         末行强制写全局g_ahrs.flag_EnvFactor=1属调试占位,使 navi_pipeline.c 步骤3环境因子门控恒通过
 */
void CoarseAlign_Horizon_RT(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor)
{
	/* 静态累加器: 跨调用保持累计(无复位接口, 重复对准会续算) */
	static double sum_accdata[3] = {0}, sum_gyrodata[3] = {0};

	static long int kk = 0; /* 已累计帧数(滚动平均分母) */

	double mat_temp[3][3] = {0}; /* 对准姿态阵中间结果 */
	double v_temp[3] = {0};		 /* 正交化辅助向量 */

	/* 帧计数+1, 本帧加表/陀螺增量累加进总和 */
	kk++;
	mat_plus(sum_accdata, (*p_imu_infor).data_acc, sum_accdata, 3, 1);
	mat_plus(sum_gyrodata, (*p_imu_infor).data_gyro, sum_gyrodata, 3, 1);

	/* 滚动平均: average = sum / kk (静止段多帧平均抑制噪声, 等效低通) */
	mat_multi_num(sum_accdata, 1.0 / kk, (*p_AHRS_Infor).average_accdata, 3, 1);
	mat_multi_num(sum_gyrodata, 1.0 / kk, (*p_AHRS_Infor).average_gyrodata, 3, 1);

	mat_temp[2][0] = (*p_AHRS_Infor).average_accdata[0] / norm_3d((*p_AHRS_Infor).average_accdata);
	mat_temp[2][1] = (*p_AHRS_Infor).average_accdata[1] / norm_3d((*p_AHRS_Infor).average_accdata);
	mat_temp[2][2] = (*p_AHRS_Infor).average_accdata[2] / norm_3d((*p_AHRS_Infor).average_accdata);

	if (fabs(mat_temp[2][0]) > sqrt(3) / 3)
		vect3d_assign(mat_temp[2][1], -mat_temp[2][0], 0.0, v_temp);
	else
		vect3d_assign(0.0, mat_temp[2][2], -mat_temp[2][1], v_temp);

	vect3d_assign(v_temp[0] / norm_3d(v_temp), v_temp[1] / norm_3d(v_temp), v_temp[2] / norm_3d(v_temp), mat_temp[1]);
	vect3d_cross_vect3d(mat_temp[1], mat_temp[2], mat_temp[0]);
	mat_assign(mat_temp[0], (*p_AHRS_Infor).cnb_align[0], 3, 3);

	m2att(mat_temp, (*p_AHRS_Infor).att_align);

	/* 环境因子门控: 静止/低振动场景恒满足对准条件. */
	(*p_AHRS_Infor).flag_EnvFactor = 1;
}
