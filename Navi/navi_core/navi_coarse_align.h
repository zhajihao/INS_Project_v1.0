/**
 * @file navi_coarse_align.h
 * @brief 粗对准:双矢量解析对准、惯性基座对准、水平对准
 */
#pragma once
#ifndef NAVI_COARSE_ALIGN_H
#define NAVI_COARSE_ALIGN_H

#include "navi_config/navi_types.h"

void coarsealign_2vect(double average_accdata[3], double average_gyrodata[3], double cnb[3][3]);
void coarsealign_2vect_RT(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void coarsealign_inertial(double data_acc1[3], double data_acc2[3], double data_gyro1[3], double data_gyro2[3], double pos[3], struct Ethi *p_ethi, double cb0b[3][3], double cn0n[3][3], double Fb[3], double Gn[3], double *pointer_t, double nts);
void coarsealign_inertialRT(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor);
void CoarseAlign_Horizon_RT(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor);

#endif /* NAVI_COARSE_ALIGN_H */
