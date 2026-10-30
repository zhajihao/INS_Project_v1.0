/**
 * @file navi_time_sync.h
 * @brief 时间同步补偿:IMU/GNSS 时间不同步引起的速度与位置误差
 */
#pragma once
#ifndef NAVI_TIME_SYNC_H
#define NAVI_TIME_SYNC_H

#include "navi_config/navi_types.h"

void TimeAsync_ImuGNSS(struct Imu_Infor *p_imu_infor, struct Sys_Infor *p_sys_infor, struct Ethi *p_ethi);

#endif /* NAVI_TIME_SYNC_H */
