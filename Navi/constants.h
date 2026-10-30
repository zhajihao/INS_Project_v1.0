
#ifndef CONSTANTS_H
#define CONSTANTS_H

/* ============================================================================
 * 注意: 本文件已废弃 (DEPRECATED).
 *
 * 原 constants.h 与 navi_config/navi_constants.h 有 21 个重复宏定义
 * (PI / GM / Re / wie / ff / ee / Rp / ge / gp / g0 / ug / arcdeg / arcmin /
 *  arcsec / hur / dph / dpsh / ugpsHz / ppm / ugpg2 等), 已在 P0 优化中:
 *   - 重复的物理/数学常量 → 已统一由 navi_config/navi_constants.h 提供
 *   - 滤波器参数独有部分 → 已移入 navi_config/navi_filter_params.h
 *
 * 所有消费者请改用:
 *   #include "navi_config/navi_constants.h"       (物理/数学常量)
 *   #include "navi_config/navi_filter_params.h"   (滤波器参数 / PDOP / 杆臂)
 *
 * 本文件暂时保留仅为兼容, 不再建议直接 include. 待 kalman15d 旧代码
 * 升级完毕后可删除.
 * ============================================================================ */

/* 以下宏已在 navi_config/navi_constants.h 定义, 这里全部重定向 */
#include "navi_config/navi_constants.h"

/* 以下独有宏已在 navi_config/navi_filter_params.h 定义, 这里全部重定向 */
#include "navi_config/navi_filter_params.h"

#endif /* CONSTANTS_H */
