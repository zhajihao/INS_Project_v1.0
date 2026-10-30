/**
 * @file navi_ins.h
 * @brief INS 速度/位置/姿态更新、圆锥划桨补偿(cnscl)、坐标转换(datatrans)
 */
#pragma once
#ifndef NAVI_INS_H
#define NAVI_INS_H

#include "navi_config/navi_types.h"

void ins_update_RT(struct Imu_Infor *p_imu_infor, struct Ethi *p_ethi, struct Sys_Infor *p_sys_infor);
void cnscl(double wm1[3], double wm2[3], double vm1[3], double vm2[3], double phim[3], double dvbm[3]);
void datatrans(double wm1[3], double vm1[3], int upflag, double wm2[3], double vm2[3]);

#endif /* NAVI_INS_H */
