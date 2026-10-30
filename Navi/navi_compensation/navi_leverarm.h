/**
 * @file navi_leverarm.h
 * @brief 杆臂补偿:内杆臂 + 外杆臂
 */
#pragma once
#ifndef NAVI_LEVERARM_H
#define NAVI_LEVERARM_H

#include "navi_config/navi_types.h"

void LeverArm_Inner(struct Imu_Infor *p_imu_infor);

void LeverArm_Spatial(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi);

#endif /* NAVI_LEVERARM_H */
