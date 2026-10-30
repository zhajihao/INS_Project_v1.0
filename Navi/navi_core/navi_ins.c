/**
 * @file navi_ins.c
 * @brief INS 速度/位置/姿态更新、圆锥划桨补偿(cnscl)、坐标转换(datatrans)
 *
 * 由 ins_update_RT.c / cnscl.c / datatrans.c 合并,仅改写 #include 路径,
 * 算法逻辑逐字节保留。LeverArm_Inner / earth 等跨模块依赖通过头文件引入。
 */
#include "navi_ins.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_math/navi_vector.h"
#include "navi_attitude.h"
#include "navi_earth.h"
#include "navi_compensation/navi_leverarm.h"
#include <math.h>

/**
 * @brief  INS机械编排实时更新:双子样采样,圆锥+划桨补偿后依次进行速度更新、位置更新、姿态更新,并解算滤波辅助向量
 * @param  p_imu_infor  IMU信息:输入data_acc/data_gyro(速度增量m/s、角增量rad);输出cnb、vn(m/s)、pos([纬度rad,经度rad,高度m])、phim、dvbm、fnsf、wbeb、an等
 * @param  p_ethi       地球参数(wnie/wnin/gcc/gn/RMh/clRNh等)
 * @param  p_sys_infor  系统信息
 * @retval void
 * @note   每两次调用构成一个更新周期:kk交替缓存两子样(vm1/wm1、vm2/wm2),周期内依次调用cnscl、LeverArm_Inner、earth;速度更新采用前后半周期平均;本周期编排成功置flag_navsuc=1
 */
void ins_update_RT(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor)
{
	static unsigned char kk = 0;
	static double vm1[3] = {0}, vm2[3] = {0}, wm1[3] = {0}, wm2[3] = {0};
	double vn1[3] = {0};

	double v_temp1[3] = {0}, v_temp2[3] = {0}, v_temp3[3] = {0}, v_temp4[3] = {0};
	double v_temp5[3] = {0}, v_temp6[3] = {0}, v_temp7[3] = {0}, v_temp8[3] = {0}, v_temp9[3] = {0};

	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0}, mat_temp3[3][3] = {0}, mat_temp4[3][3] = {0};
	double mat_temp5[3][3] = {0}, mat_temp6[3][3] = {0};

	double v_temp10[3] = {0}, v_temp11[3] = {0}, v_temp12[3] = {0}, v_temp13[3] = {0}, v_temp14[3] = {0};
	double mat_temp7[3][3] = {0};
	/* mat_temp8 — 未使用, 已移除声明 */

	if (kk == 0)
	{

		mat_assign((*p_imu_infor).data_acc, vm1, 3, 1);	 // vm1=data_acc(k)
		mat_assign((*p_imu_infor).data_gyro, wm1, 3, 1); // wm1=data_gyro(k)

		kk = 1;

		// 导航成功标志位置0
		(*p_imu_infor).flag_navsuc = 0;
	}
	else if (kk == 1)
	{
		mat_assign((*p_imu_infor).data_acc, vm2, 3, 1);	 // vm2=data_acc(k+1)
		mat_assign((*p_imu_infor).data_gyro, wm2, 3, 1); // wm2=data_gyro(k+1)
		kk = 0;

		// 经过圆锥和划桨补偿后得到phim,dvbm
		cnscl(wm1, wm2, vm1, vm2, (*p_imu_infor).phim, (*p_imu_infor).dvbm);

		// 加表杆臂补偿
		LeverArm_Inner(p_imu_infor);

		// 计算地球相关参数
		earth(p_imu_infor, p_ethi);

		// 计算vn
		mat_multi_num((*p_ethi).wnin, -(*p_imu_infor).nts / 2.0, v_temp1, 3, 1); // v_temp1=-wnin*nts/2

		rv2mat(v_temp1, mat_temp1); // mat_temp1=rv2mat(-wnin*nts/2)

		mat_multi(mat_temp1[0], (*p_imu_infor).cnb[0], mat_temp2[0], 3, 3, 3); // mat_temp2=rv2mat(-wnin*nts/2)*cnb

		mat_multi(mat_temp2[0], (*p_imu_infor).dvbm, v_temp2, 3, 3, 1); // v_temp2=rv2mat(-wnin*nts/2)*cnb*dvbm

		mat_plus((*p_imu_infor).vn, v_temp2, v_temp3, 3, 1); // v_temp3=vn+rv2mat(-wnin*nts/2)*cnb*dvbm

		mat_multi_num((*p_ethi).gcc, (*p_imu_infor).nts, v_temp4, 3, 1); // v_temp4=gcc*nts

		mat_plus(v_temp3, v_temp4, vn1, 3, 1); // vn1=vn+rv2mat(-wnin*nts/2)*cnb*dvbm+gcc*nts

		mat_plus((*p_imu_infor).vn, vn1, v_temp5, 3, 1); // v_temp5=vn+vn1

		mat_multi_num(v_temp5, 1.0 / 2.0, (*p_imu_infor).vn, 3, 1); // vn=(vn+vn1)/2

		// 计算pos
		vect3d_assign((*p_imu_infor).vn[1] / (*p_ethi).RMh, (*p_imu_infor).vn[0] / (*p_ethi).clRNh, (*p_imu_infor).vn[2], v_temp9); // v_temp9=vn

		mat_assign(v_temp9, v_temp6, 3, 1); // v_temp6=vn

		mat_multi_num(v_temp6, (*p_imu_infor).nts, v_temp7, 3, 1); // v_temp7=vn*nts

		mat_plus((*p_imu_infor).pos, v_temp7, (*p_imu_infor).pos, 3, 1); // pos=pos+vn*nts

		mat_assign(vn1, (*p_imu_infor).vn, 3, 1); // vn=vn1

		// 计算cnb
		mat_multi_num((*p_ethi).wnin, (*p_imu_infor).nts, v_temp8, 3, 1); // v_temp8=wnin*nts
		rv2mat(v_temp8, mat_temp3);										  // mat_temp3=rv2m(wnin*nts)

		mat_transpose(mat_temp3[0], mat_temp4[0], 3, 3); // mat_temp4=(rv2m(wnin*nts))'
		rv2mat((*p_imu_infor).phim, mat_temp5);			 // mat_temp5=rv2m(phim)

		mat_multi(mat_temp4[0], (*p_imu_infor).cnb[0], mat_temp6[0], 3, 3, 3); // mat_temp6=(rv2m(wnin*nts))'*cnb

		mat_multi(mat_temp6[0], mat_temp5[0], (*p_imu_infor).cnb[0], 3, 3, 3); // cnb=(rv2m(wnin*nts))'*cnb*rv2m(phim)

		// 导航成功标志位置1
		(*p_imu_infor).flag_navsuc = 1;

		// 计算滤波需要用到辅助向量

		// 计算fnsf
		mat_plus(vm1, vm2, v_temp10, 3, 1);										  // v_temp10=vm1+vm2
		mat_multi_num(v_temp10, 1.0 / (*p_imu_infor).nts, v_temp11, 3, 1);		  // v_temp11=(vm1+vm2)/nts
		mat_multi((*p_imu_infor).cnb[0], v_temp11, (*p_imu_infor).fnsf, 3, 3, 1); // fnsf=cnb*(vm1+vm2)/nts

		// 计算wbeb
		mat_plus(wm1, wm2, v_temp12, 3, 1);								   // v_temp12=wm1+wm2
		mat_multi_num(v_temp12, 1.0 / (*p_imu_infor).nts, v_temp13, 3, 1); // v_temp13=(wm1+wm2)/nts

		mat_transpose((*p_imu_infor).cnb[0], mat_temp7[0], 3, 3);	// mat_temp7=cnb'
		mat_multi(mat_temp7[0], (*p_ethi).wnie, v_temp14, 3, 3, 1); // v_temp14=cnb'*wnie
		mat_minus(v_temp13, v_temp14, (*p_imu_infor).wbeb, 3, 1);

		// 计算an
		mat_plus((*p_imu_infor).fnsf, (*p_ethi).gn, (*p_imu_infor).an, 3, 1);

		// 计算phimk_1
		mat_assign((*p_imu_infor).phim, (*p_imu_infor).phimk_1, 3, 1);
	}
}

/**
 * @brief  圆锥+划桨双子样补偿:由相邻两个子样的角增量、速度增量解算等效旋转矢量phim与等效速度增量dvbm
 * @param  wm1  第1子样角增量(rad)
 * @param  wm2  第2子样角增量(rad)
 * @param  vm1  第1子样速度增量(m/s)
 * @param  vm2  第2子样速度增量(m/s)
 * @param  phim  出参,等效旋转矢量(rad)
 * @param  dvbm 出参,含划桨补偿的等效速度增量(m/s)
 * @retval void
 * @note   phim=(wm1+wm2)+2/3*(wm1×wm2);dvbm=(vm1+vm2)+1/2*(wm1+wm2)×(vm1+vm2)+2/3*(wm1×vm2+vm1×wm2)
 */
void cnscl(double wm1[3], double wm2[3], double vm1[3], double vm2[3], double phim[3], double dvbm[3])

{
	double wmm[3], vmm[3], csw[3], csv[3], vm31[3], vm32[3], vm33[3], vm34[3], vm3[3], scullm[3], dphim[3];

	mat_plus(wm1, wm2, wmm, 3, 1);
	mat_plus(vm1, vm2, vmm, 3, 1);

	mat_multi_num(wm1, 2.0 / 3.0, csw, 3, 1);
	mat_multi_num(vm1, 2.0 / 3.0, csv, 3, 1);

	vect3d_cross_vect3d(csw, wm2, dphim);
	vect3d_cross_vect3d(csw, vm2, vm31);
	vect3d_cross_vect3d(csv, wm2, vm32);
	mat_plus(vm31, vm32, scullm, 3, 1);

	mat_plus(wmm, dphim, phim, 3, 1);
	vect3d_cross_vect3d(wmm, vmm, vm33);

	mat_multi_num(vm33, 0.5, vm34, 3, 1);
	mat_plus(vmm, vm34, vm3, 3, 1);
	mat_plus(vm3, scullm, dvbm, 3, 1);
}
/**
 * @brief  IMU安装轴向坐标转换:按upflag选择轴置换/变号矩阵,将原始角增量、速度增量重排到标准机体系轴向
 * @param  wm1    输入原始角增量(rad)
 * @param  vm1    输入原始速度增量(m/s)
 * @param  upflag 轴向配置标志:1/2/3为三种正序置换,-1/-2/-3为对应含符号翻转的置换
 * @param  wm2    出参,转换后角增量(rad)
 * @param  vm2    出参,转换后速度增量(m/s)
 * @retval void
 * @note   upflag取其他值时转换矩阵未定义,结果不可用
 */
void datatrans(double wm1[3], double vm1[3], int upflag, double wm2[3], double vm2[3])
{
	double c_trans[3][3];

	if (upflag == 1)
	{
		c_trans[0][0] = 0;
		c_trans[0][1] = 1;
		c_trans[0][2] = 0;
		c_trans[1][0] = 0;
		c_trans[1][1] = 0;
		c_trans[1][2] = 1;
		c_trans[2][0] = 1;
		c_trans[2][1] = 0;
		c_trans[2][2] = 0;
	}
	if (upflag == 2)
	{
		c_trans[0][0] = 0;
		c_trans[0][1] = 0;
		c_trans[0][2] = 1;
		c_trans[1][0] = 1;
		c_trans[1][1] = 0;
		c_trans[1][2] = 0;
		c_trans[2][0] = 0;
		c_trans[2][1] = 1;
		c_trans[2][2] = 0;
	}
	if (upflag == 3)
	{
		c_trans[0][0] = 1;
		c_trans[0][1] = 0;
		c_trans[0][2] = 0;
		c_trans[1][0] = 0;
		c_trans[1][1] = 1;
		c_trans[1][2] = 0;
		c_trans[2][0] = 0;
		c_trans[2][1] = 0;
		c_trans[2][2] = 1;
	}

	if (upflag == -1)
	{
		c_trans[0][0] = 0;
		c_trans[0][1] = 0;
		c_trans[0][2] = 1;
		c_trans[1][0] = 0;
		c_trans[1][1] = 1;
		c_trans[1][2] = 0;
		c_trans[2][0] = -1;
		c_trans[2][1] = 0;
		c_trans[2][2] = 0;
	}

	if (upflag == -2)
	{
		c_trans[0][0] = 1;
		c_trans[0][1] = 0;
		c_trans[0][2] = 0;
		c_trans[1][0] = 0;
		c_trans[1][1] = 0;
		c_trans[1][2] = 1;
		c_trans[2][0] = 0;
		c_trans[2][1] = -1;
		c_trans[2][2] = 0;
	}

	if (upflag == -3)
	{
		c_trans[0][0] = 0;
		c_trans[0][1] = 1;
		c_trans[0][2] = 0;
		c_trans[1][0] = 1;
		c_trans[1][1] = 0;
		c_trans[1][2] = 0;
		c_trans[2][0] = 0;
		c_trans[2][1] = 0;
		c_trans[2][2] = -1;
	}

	mat_multi(c_trans[0], wm1, wm2, 3, 3, 1);
	mat_multi(c_trans[0], vm1, vm2, 3, 3, 1);
}
