/**
 * @file navi_constants.h
 * @brief 组合导航地球物理常数与单位换算因子
 */
#ifndef NAVI_CONSTANTS_H
#define NAVI_CONSTANTS_H

#define PI       3.141592653589              /* 圆周率 */
#define GM       3.986004418e14               /* 地球引力常数 GM */
#define Re       6378137                      /* 椭圆长半轴 (m) */
#define wie      7.2921151467e-5              /* 地球自转角速度 (rad/s) */
#define ff       3.352810664747480e-3         /* 椭圆扁率 f = (Re - Rp) / Re */
#define ee       8.181919084262149e-2         /* 椭圆偏心率 e = sqrt(2*f - f^2) */
#define Rp       6.356752314245179e6          /* 椭圆短半轴 (m) */
#define ge       9.780325333434361            /* 赤道重力加速度 (m/s^2) */
#define gp       9.832184935381024            /* 极点重力加速度 (m/s^2) */
#define g0       9.780325333434361            /* 标准重力加速度 g0 = ge */
#define ug       9.780325333434361e-6         /* 微 g */
#define arcdeg   1.745329251993889e-02        /* 度 -> 弧度 */
#define arcmin   2.908882086656482e-04        /* 分 -> 弧度 */
#define arcsec   4.848136811094137e-06        /* 秒 -> 弧度 */
#define hur      3600                         /* 小时 -> 秒 */
#define dph      4.848136811094136e-06        /* 度/小时 -> 弧度/秒 */
#define dpsh     2.908882086656482e-04        /* 度/sqrt(小时) -> 弧度/sqrt(s) */
#define ugpsHz   9.780325333434361e-6         /* 微g/sqrt(Hz) -> m/s^2/sqrt(Hz) */
#define ppm      1e-6                          /* 百万分之一 */
#define ugpg2    1.0225e-7                    /* mGal/(m/s^2)^2 */

#endif /* NAVI_CONSTANTS_H */
