#pragma once
#ifndef NAVI_KALMAN19D_H
#define NAVI_KALMAN19D_H

#include "navi_config/navi_types.h"

void Kalman19d_Init(struct Kalman19d_Starter *p_Kalman19d_Starter, struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor);
void Kalman19d_staRT(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Kalman19d_State *p_Kalman19d_State, struct Ethi *p_ethi);
void Kalman19d_timeRT(struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor);
void Kalman19d_mes_seq6dRT(struct Kalman19d_State *p_Kalman19d_State, struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi);
void Kalman19d_Feedback(struct Kalman19d_State *p_Kalman19d_State, struct Sys_Infor *p_sys_infor, struct Imu_Infor *p_imu_infor);
void nav_integ(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor, struct Kalman19d_State *p_Kalman19d_State, struct Kalman19d_Starter *p_Kalman19d_Starter);

#endif /* NAVI_KALMAN19D_H */
