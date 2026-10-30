/**
 * @file GetFlag_GNSS_type.c
 * @brief GNSS 类型判别与有效性筛选
 *
 * 根据 GNSS 类型标志位（RTK/单点/差分等）选择有效数据源，
 * 设置 flag_GNSSvalid 和 GNSSmode 供后续滤波使用。
 */

#include "navi_config/navi_types.h"
#include "navi_math/math_matrix.h"
#include "navi_config/GNSS_Infor.h"
#include <string.h>
#include "navi_config/navi_constants.h"
#include "navi_config/navi_filter_params.h"

void GetGNSSType(struct GNSS_Infor *p_GNSS_Infor, struct Sys_Infor *p_Sys_Infor)
{
	(*p_GNSS_Infor).PDOP = 20; // 将位置精度因子设一大值，防止无效量测误入
	(*p_GNSS_Infor).sat_num = 0;

	if (0x08 == (((*p_GNSS_Infor).GNSStype) & 0x08)) // 内部RTK
	{
		(*p_GNSS_Infor).flag_GNSSvalid = (*p_GNSS_Infor).flag_GNSS1valid; // 赋值GNSS有效标志
		memcpy((*p_GNSS_Infor).vn, (*p_GNSS_Infor).vn1, sizeof((*p_GNSS_Infor).vn));
		memcpy((*p_GNSS_Infor).pos, (*p_GNSS_Infor).pos1, sizeof((*p_GNSS_Infor).pos));
		memset((*p_GNSS_Infor).r_outlearm, 0, sizeof((*p_GNSS_Infor).r_outlearm));
		(*p_GNSS_Infor).PDOP = (*p_GNSS_Infor).PDOP_1;
		(*p_GNSS_Infor).sat_num = (*p_GNSS_Infor).sat_num1;
		(*p_GNSS_Infor).GNSSmode = 1; // 卫导工作模式为RTK模式

		vect3d_assign(0.0, 0.0, 0.0, (*p_Sys_Infor).r_outlearm); // 将外杆臂设置为0
	}
	else if (0x04 == (((*p_GNSS_Infor).GNSStype) & 0x04))
	{
		(*p_GNSS_Infor).flag_GNSSvalid = (*p_GNSS_Infor).flag_GNSS2valid; // 赋值GNSS有效标志
		memcpy((*p_GNSS_Infor).vn, (*p_GNSS_Infor).vn2, sizeof((*p_GNSS_Infor).vn));
		memcpy((*p_GNSS_Infor).pos, (*p_GNSS_Infor).pos2, sizeof((*p_GNSS_Infor).pos));
		(*p_GNSS_Infor).PDOP = (*p_GNSS_Infor).PDOP_2;
		(*p_GNSS_Infor).sat_num = (*p_GNSS_Infor).sat_num2;
		(*p_GNSS_Infor).GNSSmode = 1; // 卫导工作模式为RTK模式
		// 杆臂赋值
		(*p_GNSS_Infor).r_outlearm[0] = OUTER_ARM_X;
		(*p_GNSS_Infor).r_outlearm[1] = OUTER_ARM_Y;
		(*p_GNSS_Infor).r_outlearm[2] = OUTER_ARM_Z;
		vect3d_assign((*p_GNSS_Infor).r_outlearm[0], (*p_GNSS_Infor).r_outlearm[1], (*p_GNSS_Infor).r_outlearm[2], (*p_Sys_Infor).r_outlearm); // 写入外杆臂值
	}
	else if (0x02 == (((*p_GNSS_Infor).GNSStype) & 0x02))
	{
		(*p_GNSS_Infor).flag_GNSSvalid = (*p_GNSS_Infor).flag_GNSS1valid; // 赋值GNSS有效标志
		memcpy((*p_GNSS_Infor).vn, (*p_GNSS_Infor).vn1, sizeof((*p_GNSS_Infor).vn));
		memcpy((*p_GNSS_Infor).pos, (*p_GNSS_Infor).pos1, sizeof((*p_GNSS_Infor).pos));
		memset((*p_GNSS_Infor).r_outlearm, 0, sizeof((*p_GNSS_Infor).r_outlearm));
		(*p_GNSS_Infor).PDOP = (*p_GNSS_Infor).PDOP_1;
		(*p_GNSS_Infor).sat_num = (*p_GNSS_Infor).sat_num1;
		// 若传入 outerDGNSS 的类型号，传递相应数据
		(*p_GNSS_Infor).GNSSmode = 0;                            // 卫导工作模式为单点定位模式
		vect3d_assign(0.0, 0.0, 0.0, (*p_Sys_Infor).r_outlearm); // 将外杆臂设置为0
	}
	else
	{
		(*p_GNSS_Infor).flag_GNSSvalid = (*p_GNSS_Infor).flag_GNSS2valid; // 赋值GNSS有效标志
		memcpy((*p_GNSS_Infor).vn, (*p_GNSS_Infor).vn2, sizeof((*p_GNSS_Infor).vn));
		memcpy((*p_GNSS_Infor).pos, (*p_GNSS_Infor).pos2, sizeof((*p_GNSS_Infor).pos));
		(*p_GNSS_Infor).PDOP = (*p_GNSS_Infor).PDOP_2;
		(*p_GNSS_Infor).sat_num = (*p_GNSS_Infor).sat_num2;
		// 杆臂赋值
		(*p_GNSS_Infor).r_outlearm[0] = OUTER_ARM_X;
		(*p_GNSS_Infor).r_outlearm[1] = OUTER_ARM_Y;
		(*p_GNSS_Infor).r_outlearm[2] = OUTER_ARM_Z;
		// 若传入 outerGNSS 类型号，传递相应数据
		if (0x01 == (((*p_GNSS_Infor).GNSStype) & 0x01))
		{
			(*p_GNSS_Infor).GNSSmode = 0;                                                                                                          // 卫导工作模式为单点定位模式
			vect3d_assign((*p_GNSS_Infor).r_outlearm[0], (*p_GNSS_Infor).r_outlearm[1], (*p_GNSS_Infor).r_outlearm[2], (*p_Sys_Infor).r_outlearm); // 写入外杆臂值
		}
	}
}

void IsGNSSDataValid(struct GNSS_Infor *p_GNSS_Infor, struct Imu_Infor *p_Imu_Infor, struct Sys_Infor *p_Sys_Infor)
{
	static unsigned int cnt_NoGNSS = 0;

	// PDOP 为 float, PDOP_MAX/PDOP_MIN 为 double 常量: 显式提升以消除隐式转换警告 (语义与原隐式提升完全等价)
	if (((double)((*p_GNSS_Infor).PDOP) < PDOP_MAX) && ((double)((*p_GNSS_Infor).PDOP) > PDOP_MIN) && (1 == (*p_GNSS_Infor).flag_GNSSvalid))
	{
		(*p_Sys_Infor).flag_GNSSvalid = 1; // GNSS 数据标志位有效且 PDOP有效，则 GNSS 一包数据有效标志位置 1
		(*p_GNSS_Infor).flag_GNSSvalid = 0;
		cnt_NoGNSS = 0;
	}
	else
	{
		(*p_Sys_Infor).flag_GNSSvalid = 0; // 反之，GNSS 数据无效，无效计数值累加
		(*p_GNSS_Infor).flag_GNSSvalid = 0;
		cnt_NoGNSS++;
	}

	if (cnt_NoGNSS > GNSSINVALIDTIME * (*p_Imu_Infor).fs) // 如果累计卫导无效时间大于上限，则置卫导故障标志

	{
		(*p_Sys_Infor).flag_GNSSfault = 1;
	}
	else
	{
		(*p_Sys_Infor).flag_GNSSfault = 0;
	}
}
