#pragma once
#ifndef NAVI_KALMAN7D_H
#define NAVI_KALMAN7D_H

#include "navi_config/navi_types.h"

void Kalman7d_Init(struct Kalman7d_Starter *p_Kalman7d_Starter, struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void Kalman7d_sta_sta(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Kalman7d_State *p_Kalman7d_State, struct Ethi *p_ethi);
void Kalman7d_time_sta(struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void Kalman7d_mes_sta(struct Kalman7d_State *p_Kalman7d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void PreciseAlign_Kalman7d(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Kalman7d_Starter *p_Kalman7d_Starter, struct Kalman7d_State *p_Kalman7d_State, struct Sys_Infor *p_sys_infor);

#endif /* NAVI_KALMAN7D_H */
