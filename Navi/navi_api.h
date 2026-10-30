/**
 * @file navi_api.h
 * @brief 组合导航对外统一接口
 *
 * 上层(Lwip 应用 main.c / proto 命令处理)只需 include 本文件,
 * 在每帧 IMU 数据就绪后填充 imu->gyro_ori/acc_ori 并调用 navi_run_one_frame。
 */
#ifndef NAVI_API_H
#define NAVI_API_H

#include "navi_config/navi_types.h"
#include "navi_kalman/kalman15d/Kalman15d_SSMat.h" /* struct Kalman15d_SSMat (navi_init 入参) */

void navi_init(struct Imu_Infor *imu,
               struct Ethi *ethi,
               struct Sys_Infor *sys,
               struct Kalman19d_State *k19,
               struct Kalman19d_Starter *k19s,
               struct Kalman15d_SSMat *k15);

void navi_run_one_frame(struct Imu_Infor *imu,
                        struct Ethi *ethi,
                        struct Sys_Infor *sys,
                        struct Kalman19d_State *k19,
                        struct Kalman19d_Starter *k19s,
                        struct Kalman7d_State *k7,
                        struct Kalman7d_Starter *k7s);

#endif /* NAVI_API_H */
