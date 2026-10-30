/**
 * @file navi_ahrs.h
 * @brief AHRS 水平对准有效性判断与水平姿态更新
 */
#pragma once
#ifndef NAVI_AHRS_H
#define NAVI_AHRS_H

#include "navi_config/navi_types.h"

void AHRS(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor);
void Valid_HorizonAlign(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor);
void HoriAtt_Update(struct Imu_Infor *p_imu_infor, struct AHRS_Infor *p_AHRS_Infor);

#endif /* NAVI_AHRS_H */
