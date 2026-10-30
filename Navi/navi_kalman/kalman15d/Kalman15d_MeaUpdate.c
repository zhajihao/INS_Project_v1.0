/**
 * @file Kalman15d_MeaUpdate.c
 * @brief 15维卡尔曼滤波器量测更新
 *
 * 以GNSS速度/位置(双天线航向可用时附加航向)为量测，融合校正惯导误差状态。
 */

#include <math.h>
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"
#include "navi_math/math_matrix.h"
#include "Kalman15d_SSMat.h"
#include "navi_config/GNSS_Infor.h"
#include "navi_config/navi_types.h"

/**
 * @brief  15维卡尔曼滤波量测更新：INS-GNSS速度/位置量测(双天线航向可用时附加航向量测)融合校正
 * @param  p_Kalman15d_SSMat  15维滤波器状态矩阵(读写)：输入xkk_1/Pkk_1，输出xk/Pk
 * @param  p_GNSS_infor       GNSS信息：输入vn(m/s)、pos(rad,rad,m)、azimuth(rad)、GNSSmode(0单点/1RTK)
 * @param  p_imu_infor        IMU信息：输入vn、pos、att
 * @param  p_sys_infor        系统信息：输入flag_GNSSvalid/flag_azimuthvalid；输出flag_KalmanSuc
 * @retval void
 * @note   量测zk=INS-GNSS速度/位置差(双天线时附加航向差)；rk按SPP/RTK标准差平方装订；
 *         Kk=Pkk_1*Hk'*(Hk*Pkk_1*Hk'+rk)^-1；xk=xkk_1+Kk*(zk-Hk*xkk_1)；Pk=(I-Kk*Hk)*Pkk_1
 */
void Kalman15d_MesUpdate(struct Kalman15d_SSMat *p_Kalman15d_SSMat, struct GNSS_Infor *p_GNSS_infor, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor)
{

	// 声明INS-GNSS组合所需的临时矩阵
	float Kk_GNSS[15][6] = {0}, Hk_GNSS[6][15] = {0}, rk_GNSS[6][6] = {0}, zk_GNSS[6] = {0};
	float mat_temp1[6][15] = {0}, mat_temp2[15][6] = {0}, mat_temp3[6][6] = {0}, mat_temp4[6][6] = {0}, mat_temp5[6][6] = {0},
		  mat_temp6[15][6] = {0}, mat_temp7[15][15] = {0}, mat_temp8[15][15] = {0}, mat_temp9[15][15] = {0};
	float v_temp1[6] = {0}, v_temp2[6] = {0}, v_temp3[15] = {0};

	// 声明INS-GNSS-双天线组合所需的临时矩阵
	float Kk_GA[15][7] = {0}, Hk_GA[7][15] = {0}, rk_GA[7][7] = {0}, zk_GA[7] = {0};
	float mat_temp11[7][15] = {0}, mat_temp12[15][7] = {0}, mat_temp13[7][7] = {0}, mat_temp14[7][7] = {0}, mat_temp15[7][7] = {0},
		  mat_temp16[15][7] = {0}, mat_temp17[15][15] = {0}, mat_temp18[15][15] = {0}, mat_temp19[15][15] = {0};
	float v_temp11[7] = {0}, v_temp12[7] = {0}, v_temp13[15] = {0};

	//
	if (1 == (*p_sys_infor).flag_GNSSvalid) // 如果卫导数据可用
	{
		/* 手动清除系统GNSS标志 */
		(*p_sys_infor).flag_GNSSvalid = 0;
		if (0 == (*p_sys_infor).flag_azimuthvalid) // 如果双天线航向不可用
		{

			if (0 == (*p_GNSS_infor).GNSSmode) // 如果是单点定位
			{

				rk_GNSS[0][0] = V_STD_SPP * V_STD_SPP;
				rk_GNSS[1][1] = V_STD_SPP * V_STD_SPP;
				rk_GNSS[2][2] = V_STD_SPP * V_STD_SPP;

				rk_GNSS[3][3] = PH_STD_SPP * PH_STD_SPP;
				rk_GNSS[4][4] = PH_STD_SPP * PH_STD_SPP;
				rk_GNSS[5][5] = PV_STD_SPP * PV_STD_SPP;
			}

			if (1 == (*p_GNSS_infor).GNSSmode) // 如果是RTK
			{

				rk_GNSS[0][0] = V_STD_RTK * V_STD_RTK;
				rk_GNSS[1][1] = V_STD_RTK * V_STD_RTK;
				rk_GNSS[2][2] = V_STD_RTK * V_STD_RTK;

				rk_GNSS[3][3] = PH_STD_RTK * PH_STD_RTK;
				rk_GNSS[4][4] = PH_STD_RTK * PH_STD_RTK;
				rk_GNSS[5][5] = PV_STD_RTK * PV_STD_RTK;
			}

			Hk_GNSS[0][3] = 1.0;
			Hk_GNSS[1][4] = 1.0;
			Hk_GNSS[2][5] = 1.0;
			Hk_GNSS[3][6] = 1.0;
			Hk_GNSS[4][7] = 1.0;
			Hk_GNSS[5][8] = 1.0;

			zk_GNSS[0] = p_imu_infor->vn[0] - p_GNSS_infor->vn[0];
			zk_GNSS[1] = p_imu_infor->vn[1] - p_GNSS_infor->vn[1];
			zk_GNSS[2] = p_imu_infor->vn[2] - p_GNSS_infor->vn[2];

			zk_GNSS[3] = p_imu_infor->pos[0] - p_GNSS_infor->pos[0];
			zk_GNSS[4] = p_imu_infor->pos[1] - p_GNSS_infor->pos[1];
			zk_GNSS[5] = p_imu_infor->pos[2] - p_GNSS_infor->pos[2];

			// 计算Kk_GNSS
			mat_multi_f(Hk_GNSS[0], (*p_Kalman15d_SSMat).Pkk_1[0], mat_temp1[0], 6, 15, 15);   // mat_temp1=Hk*Pkk_1
			mat_transpose_f(Hk_GNSS[0], mat_temp2[0], 6, 15);								   // mat_temp2=Hk_GNSS';
			mat_multi_f(mat_temp1[0], mat_temp2[0], mat_temp3[0], 6, 15, 6);				   // mat_temp3=Hk_GNSS*Pkk_1*Hk_GNSS';
			mat_plus_f(mat_temp3[0], rk_GNSS[0], mat_temp4[0], 6, 6);						   // mat_temp4=Hk_GNSS*Pkk_1*Hk_GNSS'+rk_GNSS;
			mat_inverse(6, mat_temp4[0], mat_temp5[0]);										   // mat_temp5=(Hk_GNSS*Pkk_1*Hk_GNSS'+rk_GNSS)^-1;
			mat_multi_f((*p_Kalman15d_SSMat).Pkk_1[0], mat_temp2[0], mat_temp6[0], 15, 15, 6); // mat_temp6=Pkk_1*Hk_GNSS';
			mat_multi_f(mat_temp6[0], mat_temp5[0], Kk_GNSS[0], 15, 6, 6);					   // Kk_GNSS=Pkk_1*Hk_GNSS'*(Hk_GNSS*Pkk_1*Hk_GNSS'+rk_GNSS)^-1;

			// 计算xk
			mat_multi_f(Hk_GNSS[0], (*p_Kalman15d_SSMat).xkk_1, v_temp1, 6, 15, 1); // v_temp1=Hk_GNSS*xkk_1
			mat_minus_f(zk_GNSS, v_temp1, v_temp2, 6, 1);							// v_temp2=zk_GNSS-Hk_GNSS*xkk_1
			mat_multi_f(Kk_GNSS[0], v_temp2, v_temp3, 15, 6, 1);					// v_temp3=Kk_GNSS*(zk_GNSS-Hk_GNSS*xkk_1)
			mat_plus_f((*p_Kalman15d_SSMat).xkk_1, v_temp3, (*p_Kalman15d_SSMat).xk, 15, 1);

			// 计算Pk
			mat_multi_f(Kk_GNSS[0], Hk_GNSS[0], mat_temp7[0], 15, 6, 15);									  // mat_temp7=Kk_GNSS*Hk_GNSS
			mat_unit_f(mat_temp8[0], 15);																	  // mat_temp8=eye(15)
			mat_minus_f(mat_temp8[0], mat_temp7[0], mat_temp9[0], 15, 15);									  // mat_temp9=eye(15)-Kk_GNSS*Hk_GNSS
			mat_multi_f(mat_temp9[0], (*p_Kalman15d_SSMat).Pkk_1[0], (*p_Kalman15d_SSMat).Pk[0], 15, 15, 15); // Pk=(eye(15)-Kk_GNSS*Hk_GNSS)*Pkk_1;
		}

		if (1 == (*p_sys_infor).flag_azimuthvalid) // 如果双天线航向可用
		{
			if (0 == (*p_GNSS_infor).GNSSmode) // 如果是单点定位
			{

				rk_GA[0][0] = V_STD_SPP * V_STD_SPP;
				rk_GA[1][1] = V_STD_SPP * V_STD_SPP;
				rk_GA[2][2] = V_STD_SPP * V_STD_SPP;

				rk_GA[3][3] = PH_STD_SPP * PH_STD_SPP;
				rk_GA[4][4] = PH_STD_SPP * PH_STD_SPP;
				rk_GA[5][5] = PV_STD_SPP * PV_STD_SPP;

				rk_GA[6][6] = AZI_STD_DUALGNSS * AZI_STD_DUALGNSS;
			}

			if (1 == (*p_GNSS_infor).GNSSmode) // 如果是RTK
			{

				rk_GA[0][0] = V_STD_RTK * V_STD_RTK;
				rk_GA[1][1] = V_STD_RTK * V_STD_RTK;
				rk_GA[2][2] = V_STD_RTK * V_STD_RTK;

				rk_GA[3][3] = PH_STD_RTK * PH_STD_RTK;
				rk_GA[4][4] = PH_STD_RTK * PH_STD_RTK;
				rk_GA[5][5] = PV_STD_RTK * PV_STD_RTK;

				rk_GA[6][6] = AZI_STD_DUALGNSS * AZI_STD_DUALGNSS;
			}

			Hk_GA[0][3] = 1.0;
			Hk_GA[1][4] = 1.0;
			Hk_GA[2][5] = 1.0;
			Hk_GA[3][6] = 1.0;
			Hk_GA[4][7] = 1.0;
			Hk_GA[5][8] = 1.0;
			Hk_GA[6][2] = 1.0;

			zk_GA[0] = p_imu_infor->vn[0] - p_GNSS_infor->vn[0];
			zk_GA[1] = p_imu_infor->vn[1] - p_GNSS_infor->vn[1];
			zk_GA[2] = p_imu_infor->vn[2] - p_GNSS_infor->vn[2];

			zk_GA[3] = p_imu_infor->pos[0] - p_GNSS_infor->pos[0];
			zk_GA[4] = p_imu_infor->pos[1] - p_GNSS_infor->pos[1];
			zk_GA[5] = p_imu_infor->pos[2] - p_GNSS_infor->pos[2];

			zk_GA[6] = p_imu_infor->att[2] - p_GNSS_infor->azimuth;

			// 计算Kk_GA
			mat_multi_f(Hk_GA[0], (*p_Kalman15d_SSMat).Pkk_1[0], mat_temp11[0], 7, 15, 15);		 // mat_temp11=Hk*Pkk_1
			mat_transpose_f(Hk_GA[0], mat_temp12[0], 7, 15);									 // mat_temp12=Hk_GA';
			mat_multi_f(mat_temp11[0], mat_temp12[0], mat_temp13[0], 7, 15, 7);					 // mat_temp13=Hk_GA*Pkk_1*Hk_GA';
			mat_plus_f(mat_temp13[0], rk_GA[0], mat_temp14[0], 7, 7);							 // mat_temp14=Hk_GA*Pkk_1*Hk_GA'+rk_GA;
			mat_inverse(7, mat_temp14[0], mat_temp15[0]);										 // mat_temp15=(Hk_GA*Pkk_1*Hk_GA'+rk_GA)^-1;
			mat_multi_f((*p_Kalman15d_SSMat).Pkk_1[0], mat_temp12[0], mat_temp16[0], 15, 15, 7); // mat_temp16=Pkk_1*Hk_GA';
			mat_multi_f(mat_temp16[0], mat_temp15[0], Kk_GA[0], 15, 7, 7);						 // Kk_GA=Pkk_1*Hk_GA'*(Hk_GA*Pkk_1*Hk_GA'+rk_GA)^-1;

			// 计算xk
			mat_multi_f(Hk_GA[0], (*p_Kalman15d_SSMat).xkk_1, v_temp11, 7, 15, 1); // v_temp11=Hk_GA*xkk_1
			mat_minus_f(zk_GA, v_temp11, v_temp12, 7, 1);						   // v_temp12=zk_GA-Hk_GA*xkk_1
			mat_multi_f(Kk_GA[0], v_temp12, v_temp13, 15, 7, 1);				   // v_temp13=Kk_GA*(zk_GA-Hk_GA*xkk_1)
			mat_plus_f((*p_Kalman15d_SSMat).xkk_1, v_temp13, (*p_Kalman15d_SSMat).xk, 15, 1);

			// 计算Pk
			mat_multi_f(Kk_GA[0], Hk_GA[0], mat_temp17[0], 15, 7, 15);										   // mat_temp17=Kk_GA*Hk_GA
			mat_unit_f(mat_temp18[0], 15);																	   // mat_temp18=eye(15)
			mat_minus_f(mat_temp18[0], mat_temp17[0], mat_temp19[0], 15, 15);								   // mat_temp19=eye(15)-Kk_GA*Hk_GA
			mat_multi_f(mat_temp19[0], (*p_Kalman15d_SSMat).Pkk_1[0], (*p_Kalman15d_SSMat).Pk[0], 15, 15, 15); // Pk=(eye(15)-Kk_GNSS*Hk_GNSS)*Pkk_1;
		}
		/* 置位量测更新有效*/
		(*p_sys_infor).flag_KalmanSuc = 1;
	}
}
