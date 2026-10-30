/**
 * @file navi_ahrs.c
 * @brief AHRS 水平对准有效性判断与水平姿态更新
 *
 * 由 AHRS.c / Valid_HorizonAlign.c / HoriAtt_Update.c 合并,仅改写 #include 路径,
 * 算法逻辑逐字节保留。CoarseAlign_Horizon_RT 跨模块依赖通过 navi_coarse_align.h 引入。
 */
#include "navi_ahrs.h"
#include "navi_config/navi_types.h"
#include "navi_config/navi_constants.h"
#include "navi_math/navi_matrix.h"
#include "navi_attitude.h"
#include "navi_coarse_align.h"
#include <math.h>

/**
 * @brief  AHRS主流程:依次执行水平粗对准、水平对准有效性判断、水平姿态更新
 * @param  p_imu_infor   IMU信息(输入陀螺/加表增量)
 * @param  p_AHRS_Infor  AHRS状态与水平对准结果
 * @retval void
 */
void AHRS(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor)
{

	CoarseAlign_Horizon_RT(p_imu_infor, p_AHRS_Infor);
	Valid_HorizonAlign(p_imu_infor, p_AHRS_Infor);
	HoriAtt_Update(p_imu_infor, p_AHRS_Infor);
}
/**
 * @brief  水平对准有效性判断:依据比力模值、比力水平分量及角运动强度,判定水平粗对准结果是否可用
 * @param  p_imu_infor   IMU信息
 * @param  p_AHRS_Infor  AHRS信息:输入mod_accdata/mod_gyrodata、cnb_align、average_accdata,有效时输出Flag_ValidHorAlign=1
 * @retval void
 * @note   判据:平均陀螺模值<0.04°(静止flag_rot=0)、比力模值接近9.8(|差|<0.5 m/s^2)、水平分量mod_fH<0.2 m/s^2;Flag_HorAlignFinish=0时直接返回
 */
void Valid_HorizonAlign(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor)

{
	unsigned char flag_rot = 0;
	double mod_fU = 0, mod_fH = 0;

	if ((*p_AHRS_Infor).Flag_HorAlignFinish == 0)
	{
		return;
	}
	else
	{
		if ((*p_AHRS_Infor).mod_gyrodata < 0.04 * arcdeg)
			flag_rot = 0;
		else
			flag_rot = 1;

		mat_multi((*p_AHRS_Infor).cnb_align[2], (*p_AHRS_Infor).average_accdata, &mod_fU, 1, 3, 1);

		mod_fH = sqrt(fabs(pow((*p_AHRS_Infor).mod_accdata, 2) - pow(mod_fU, 2)));

		if (fabs(((*p_AHRS_Infor).mod_accdata - 9.8) < 0.5) && (mod_fH < 0.2) && (flag_rot == 0))

		{
			(*p_AHRS_Infor).Flag_ValidHorAlign = 1;
		}
	}
}

/**
 * @brief  水平姿态更新:用陀螺角增量对cnb做姿态跟踪,水平对准有效时对俯仰/横滚做互补修正
 * @param  p_imu_infor   IMU信息:输入data_gyro(rad)、ts(s)
 * @param  p_AHRS_Infor  AHRS信息:输入att_align、Flag_ValidHorAlign;输入输出cnb
 * @retval void
 * @note   修正系数alpha=0.5:跟踪时长t_track<10s时att=(1-alpha)*att+alpha*att_align;超过10s直接采用对准姿态并复位t_track;航向不修正;处理后清Flag_ValidHorAlign
 */
void HoriAtt_Update(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor)

{
	static double alpha = 0.5, t_track = 0;

	double mat_temp1[3][3] = {0}, mat_temp2[3][3] = {0};
	/* mat_temp3 / v_temp1 / v_temp2 — 未使用, 已移除声明 */
	double att_temp[3] = {0};

	// 姿态跟踪
	rv2mat((*p_imu_infor).data_gyro, mat_temp1);
	mat_multi((*p_AHRS_Infor).cnb[0], mat_temp1[0], mat_temp2[0], 3, 3, 3);
	mat_assign(mat_temp2[0], (*p_AHRS_Infor).cnb[0], 3, 3);
	t_track = t_track + (*p_imu_infor).ts;

	// 姿态修正
	if ((*p_AHRS_Infor).Flag_ValidHorAlign == 1)
	{

		m2att((*p_AHRS_Infor).cnb, att_temp);

		if (t_track < 10.0)
		{

			att_temp[0] = (1 - alpha) * att_temp[0] + alpha * (*p_AHRS_Infor).att_align[0];
			att_temp[1] = (1 - alpha) * att_temp[1] + alpha * (*p_AHRS_Infor).att_align[1];
		}
		else
		{
			att_temp[0] = (*p_AHRS_Infor).att_align[0];
			att_temp[1] = (*p_AHRS_Infor).att_align[1];
			t_track = 0.0;
		}

		a2mat(att_temp, (*p_AHRS_Infor).cnb);

		(*p_AHRS_Infor).Flag_ValidHorAlign = 0;
	}
}
