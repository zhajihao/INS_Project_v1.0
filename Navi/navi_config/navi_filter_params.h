#ifndef NAVI_FILTER_PARAMS_H
#define NAVI_FILTER_PARAMS_H

/**
 * @file navi_filter_params.h
 * @brief 滤波器参数 (Kalman 初始标准差 / PDOP 阈值 / 杆臂等)
 *
 * 本文件包含各滤波器共享的参数宏, 与 navi_constants.h (物理/数学常量) 互补.
 * 源自旧框架 constants.h, 为避免与 navi_constants.h 的 20 个重复宏定义冲突,
 * 已剥离到 navi_config/ 下单独管理.
 *
 * 各滤波器源码统一 include 本文件而非旧 constants.h:
 *   #include "navi_config/navi_filter_params.h"
 *   #include "navi_config/navi_constants.h"   (物理常量)
 */

#include "navi_constants.h"   /* PI / arcdeg / dph / ug / ugpsHz 等 (navi_filter_params 依赖 arcdeg 等) */

/* ============ 系统参数 / GNSS 有效性阈值 ============ */
#define  PDOP_MAX  2.1           /* PDOP 上限值 */
#define  PDOP_MIN  0             /* PDOP 下限值 */
#define  GNSSINVALIDTIME 2000   /* GNSS 信号失效超时(s), 阈值 = GNSSINVALIDTIME * fs 次无效计数 */

/* ============ Kalman 滤波器标准差 ============ */

/* Pk 初值 */
#define  PHIH_INIT_STD  0.05*arcdeg      /* 水平失准角初始标准差 */
#define  PHIU_INIT_STD  2.1*arcdeg       /* 方位失准角初始标准差 */
#define  V_INIT_STD    0.05              /* 速度初值标准差 */
#define  POSH_INIT_STD  5                /* 水平位置初值标准差(m) */
#define  POSV_INIT_STD  10               /* 高度初值标准差(m) */
#define  EB_INIT_STD     5*dph           /* 陀螺零偏初值标准差 */
#define  DB_INIT_STD    200*ug           /* 加计零偏初值标准差 */

/* Q 过程噪声 */
#define  ARW    0.5*dpsh           /* 角度随机游走系数 */
#define  VRW    20*ugpsHz          /* 速度随机游走系数 */

/* Rk 量测噪声 — SPP 单点解 */
#define  V_STD_SPP     0.05        /* SPP 速度标准差(m/s) */
#define  PH_STD_SPP     3          /* SPP 水平位置标准差(m) */
#define  PV_STD_SPP     5          /* SPP 高度标准差(m) */

/* Rk 量测噪声 — RTK */
#define  V_STD_RTK     0.05        /* RTK 速度标准差(m/s) */
#define  PH_STD_RTK    0.01        /* RTK 水平位置标准差(m) */
#define  PV_STD_RTK     0.02       /* RTK 高度标准差(m) */

/* Rk 量测噪声 — 双天线航向 */
#define  AZI_STD_DUALGNSS   0.2*arcdeg   /* 双天线航向标准差(rad) */

/* ============ 安装参数 / 杆臂 ============ */
#define OUTER_ARM_X      1.0       /* 外杆臂 X(m) */
#define OUTER_ARM_Y      2.0       /* 外杆臂 Y(m) */
#define OUTER_ARM_Z      3.0       /* 外杆臂 Z(m) */



#define TAU_G      1000
#define TAU_A      1000
#endif /* NAVI_FILTER_PARAMS_H */
