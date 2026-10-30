#ifndef GNSS_INFOR_H
#define GNSS_INFOR_H

#include "navi_types.h" /* struct Sys_Infor (GetGNSSType 入参) */

struct GNSS_Infor
{
        int GNSStype;           // 卫导类型：8为内RTK，4为外RTK，2为内单点，1为外单点
        char flag_GNSS1valid;   // 卫导1数据有效标志位
        char flag_GNSS2valid;   // 卫导2数据有效标志位
        char flag_GNSSvalid;    // 最终使用卫导数据有效标志位
        double vn1[3];          // 卫导1速度
        double vn2[3];          // 卫导2速度
        double vn[3];           // 最终使用卫导速度
        double pos1[3];         // 卫导1位置
        double pos2[3];         // 卫导2位置
        double pos[3];          // 最终使用卫导位置
        char GNSSmode;          // 卫导模式：0为单点，1为RTK
        float PDOP_1;           // 卫导1 PDOP_1、卫导三维位置精度衰减因子，一般1-2精度极好，2-3精度良好，3-5精度一般，大于5不建议使用
        float PDOP_2;           // 卫导2 PDOP_2
        float PDOP;             // 最终使用PDOP
        unsigned char sat_num1; // 卫导1参与定位卫星数
        unsigned char sat_num2; // 卫导2参与定位卫星数
        unsigned char sat_num;  // 最终使用卫星数
        double r_outlearm[3];   // 空间杆臂;
        double azimuth;         // 航向角
};

void GetGNSSType(struct GNSS_Infor *p_GNSS_Infor, struct Sys_Infor *p_Sys_Infor);

void IsGNSSDataValid(struct GNSS_Infor *p_GNSS_Infor, struct Imu_Infor *p_Imu_Infor, struct Sys_Infor *p_Sys_Infor);

#endif
