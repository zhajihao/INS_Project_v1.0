#pragma once
#ifndef NAVI_KALMAN12D_H
#define NAVI_KALMAN12D_H

#include "navi_config/navi_types.h"

void Kalman12d_Init(struct Kalman12d_Starter *p_Kalman12d_Starter, struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void kalman12d_sta_sta(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State, struct Ethi *p_ethi);
void kalman12d_time_sta(struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void kalman12d_mes_sta(struct Kalman12d_State *p_Kalman12d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void PreciseAlignKalman12d_SinglePos(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman12d_Starter *p_Kalman12d_Starter, struct Kalman12d_State *p_Kalman12d_State, struct Sys_Infor *p_sys_infor);
void PreciseAlignKalman12d_RotTime(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman12d_State *p_Kalman12d_State, struct Sys_Infor *p_sys_infor);
void Kalman12d_PxvReset(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State);
void kalman12d_dvnreset(double Pk[12][12], double vn[3], double xk[12]);
void AlignResultUpdate(struct Imu_Infor *p_imu_infor, struct Kalman12d_State *p_Kalman12d_State);

#endif /* NAVI_KALMAN12D_H */
