/**
 * @file navi_earth.c
 * @brief 地球参数解算
 *
 * 根据当前速度和位置计算地球曲率半径、自转角速度投影、
 * 重力矢量及有害加速度等导航必需参数。
 */

#include "navi_earth.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_vector.h"
#include "navi_math/navi_matrix.h"
#include <math.h>

/**
 * @brief  依据当前速度、位置解算地球参数（曲率半径、角速度投影、重力及有害加速度等）
 * @param  p_imu_infor 输入IMU导航信息：vn速度[m/s,m/s,m/s]、pos位置[纬度rad,经度rad,高度m]
 * @param  p_ethi 输出地球参数结构体（RMh/RNh、wnie、wnen、wnin、wnien、gn、gcc等，结果写入）
 * @retval void
 * @note   RMh、RNh为子午圈/卯酉圈曲率半径加高度(m)；wnie=ω_ie^n，wnen=ω_en^n，
 *         wnin=ω_in^n，wnien=ω_ie^n+ω_in^n（rad/s）；gn为当地重力向量(m/s^2)，
 *         gcc=gn-ω_ie^n×vn 为含哥氏项的有害加速度(m/s^2)
 */
void earth(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi)
{

	double v_temp[3] = {0};
	
	
	//静态导航
	//vect3d_assign(0, 0, 0, (*p_ethi).vn); 
	//vect3d_assign(34.1974 * arcdeg, 108.8542 * arcdeg, 400.1, (*p_ethi).pos);

	//动态导航
vect3d_assign((*p_imu_infor).vn[0], (*p_imu_infor).vn[1], (*p_imu_infor).vn[2], (*p_ethi).vn);
vect3d_assign((*p_imu_infor).pos[0], (*p_imu_infor).pos[1], (*p_imu_infor).pos[2], (*p_ethi).pos);
	
	
	
	(*p_ethi).e2 = pow(ee,2);
	(*p_ethi).sl = sin((*p_ethi).pos[0]);
	(*p_ethi).cl = cos((*p_ethi).pos[0]);
	(*p_ethi).tl = (*p_ethi).sl / (*p_ethi).cl;
	(*p_ethi).sl2 = (*p_ethi).sl * (*p_ethi).sl;
	(*p_ethi).sl4 = (*p_ethi).sl2 * (*p_ethi).sl2;
	(*p_ethi).sq = 1 - (*p_ethi).e2 * (*p_ethi).sl2;
	(*p_ethi).sq2 = sqrt((*p_ethi).sq);
	(*p_ethi).RMh = Re * (1 - (*p_ethi).e2) / (*p_ethi).sq / (*p_ethi).sq2 + (*p_ethi).pos[2];
	(*p_ethi).RNh = Re / (*p_ethi).sq2 + (*p_ethi).pos[2];
	(*p_ethi).clRNh = (*p_ethi).cl * (*p_ethi).RNh;

	
	vect3d_assign(0.0, (*p_ethi).cl * wie, (*p_ethi).sl * wie, (*p_ethi).wnie);
	


	


	


	vect3d_assign(-(*p_ethi).vn[1] / (*p_ethi).RMh, (*p_ethi).vn[0] / (*p_ethi).RNh, (*p_ethi).vn[0] / (*p_ethi).RNh * (*p_ethi).tl,(*p_ethi).wnen);
	mat_plus((*p_ethi).wnie, (*p_ethi).wnen, (*p_ethi).wnin,3,1);
	mat_plus((*p_ethi).wnie, (*p_ethi).wnin, (*p_ethi).wnien,3,1);
	

	(*p_ethi).gLh = g0 * (1 + 5.27094e-3 * (*p_ethi).sl2 + 2.32718e-5 * (*p_ethi).sl4) - 3.086e-6 * (*p_ethi).pos[2];
	//(*p_ethi).gLh= [0; 0; -9.794107];//测控
	//(*p_ethi).gLh = [0; 0; -9.7955];//咸阳
	//(*p_ethi).gLh = [0; 0; -9.787276];//贵阳
	vect3d_assign(0.0,0.0,-(*p_ethi).gLh,(*p_ethi).gn);
	
	vect3d_cross_vect3d((*p_ethi).wnie, (*p_ethi).vn, v_temp);
	mat_minus((*p_ethi).gn, v_temp,(*p_ethi).gcc,3,1);
	//(*p_ethi).g = 9.796469015495772;
	
}
