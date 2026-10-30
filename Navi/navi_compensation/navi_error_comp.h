/**
 * @file navi_error_comp.h
 * @brief IMU 误差补偿:标定误差补偿 + 温度补偿参数获取 + MEMS 温补
 */
#pragma once
#ifndef NAVI_ERROR_COMP_H
#define NAVI_ERROR_COMP_H

#include "navi_config/navi_types.h"

void GetErrorPara_TC(struct Imu_TempCompPara *p_Imu_TempCompPara, struct Imu_Infor *p_Imu_Infor);
void ErrorComp(double errpara[30], double gyrodata_BC[3], double accdata_BC[3], double gyrodata_AC[3], double accdata_AC[3], double ts);
void Para_TemCompMEMS(double *p_);

#endif /* NAVI_ERROR_COMP_H */
