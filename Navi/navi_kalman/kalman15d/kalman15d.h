#ifndef KALMAN15D_H
#define KALMAN15D_H

/**
 * @file kalman15d.h
 * @brief 15 维 INS-GNSS 松组合卡尔曼滤波器对外接口
 *
 * 提供导航成功后进入组合滤波阶段所需的 5 个函数声明:
 *   - Kalman15d_Init        初始化状态/协方差矩阵
 *   - GetKalman15d_SSMat    根据 IMU/Sys/Ethi 构建量测矩阵
 *   - Kalman15d_TimeUpdate  时间更新 (预测)
 *   - Kalman15d_MesUpdate   量测更新 (校正)
 *   - Kalman_Feedback       闭环反馈:用滤波误差修正 Sys/Imu 惯导量
 */

#include "navi_config/navi_types.h" /* struct Imu_Infor / Sys_Infor / Ethi */
#include "navi_config/GNSS_Infor.h" /* struct GNSS_Infor (内部已 include navi_types.h) */
#include "Kalman15d_SSMat.h"        /* struct Kalman15d_SSMat (纯结构, 无外部依赖) */

#ifdef __cplusplus
extern "C"
{
#endif

	void Kalman15d_Init(struct Kalman15d_SSMat *p_Kalman15d_SSMat, float nts);

	void GetKalman15d_SSMat(struct Imu_Infor *p_imu_infor,
							struct Kalman15d_SSMat *p_Kalman15d_SSMat,
							struct Ethi *p_ethi);

	void Kalman15d_TimeUpdate(struct Kalman15d_SSMat *p_Kalman15d_SSMat);

	void Kalman15d_MesUpdate(struct Kalman15d_SSMat *p_Kalman15d_SSMat,
							 struct GNSS_Infor *p_GNSS_infor,
							 struct Imu_Infor *p_imu_infor,
							 struct Sys_Infor *p_sys_infor);

	void Kalman_Feedback(struct Kalman15d_SSMat *p_Kalman15d_SSMat,
						 struct Sys_Infor *p_sys_infor,
						 struct Imu_Infor *p_imu_infor);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN15D_H */
