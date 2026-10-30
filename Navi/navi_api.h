/**
 * @file navi_api.h
 * @brief 组合导航核心接口 (初始化 + 旧链路单帧入口)
 *
 * 本固件 (IMU_Project) 的调用方是 User/navi_pipeline.c:
 *   - navi_init() 由 navi_pipeline_init() 调用完成惯导/地球参数/15d 滤波器初始化;
 *   - navi_run_one_frame() 是 7d 精对准 / 19d 组合导航旧链路的入口,
 *     当前 15 维 Kalman 方案未使用它 (保留以备回退)。
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
