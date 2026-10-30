/**
 * @file navi_types.h
 * @brief 组合导航全部数据结构定义(由原 ethi.h/Imu_Infor.h/Sys_Infor.h/AHRS_Infor.h/
 *        Kalman*_State.h/Kalman*_Starter.h/imu_tempcomppara.h 合并)
 *
 * 坐标/分量约定:
 *   导航系 n 为 ENU(东-北-天); 速度/角速率向量分量顺序均为 [E, N, U];
 *   位置 pos 分量顺序为 [纬度 lat(rad), 经度 lon(rad), 高度 h(m)];
 *   姿态阵 cnb 为 机体系 b -> 导航系 n 的方向余弦阵;
 *   姿态角 att 由 cnb 经 m2att 解算, 顺序 [pitch, roll, yaw](rad)。
 */
#ifndef NAVI_TYPES_H
#define NAVI_TYPES_H

/* 地球相关参数: 由 earth() 依据当前 vn/pos 实时解算 */
struct Ethi
{
	double e2;       /* 参考椭球第一偏心率平方 e²(常数, ee²) */
	double sl;       /* sinL, 当地纬度正弦 */
	double cl;       /* cosL, 当地纬度余弦 */
	double tl;       /* tanL = sinL/cosL */
	double sl2;      /* sin²L */
	double sl4;      /* sin⁴L */
	double sq;       /* 卯酉圈分母项 1 - e²·sin²L(未开方) */
	double sq2;      /* sqrt(sq), 卯酉圈曲率半径分母 */
	double RMh;      /* 子午圈曲率半径加高度 RM + h (m) */
	double RNh;      /* 卯酉圈曲率半径加高度 RN + h (m) */
	double clRNh;    /* cosL·RNh, 位置微分常用组合项 (m) */
	double gLh;      /* 纬度L、高度h处重力加速度标量 (m/s²) */
	double wnie[3];  /* 地球自转角速率在导航系投影 ω_ie^n = (0, cosL·wie, sinL·wie) (rad/s) */
	double vn[3];    /* 载体相对地球速度在导航系投影 [E, N, U] (m/s), 由 imu.vn 拷入 */
	double wnen[3];  /* 位置速率 ω_en^n(导航系相对地球系), (-vn_N/RMh, vn_E/RNh, vn_E·tanL/RNh) (rad/s) */
	double wnin[3];  /* 导航系相对惯性系角速率 ω_in^n = wnie + wnen (rad/s) */
	double wnien[3]; /* = wnie + wnin (rad/s), 特定算法用组合角速率 */
	double gn[3];    /* 当地重力向量导航系 (0, 0, -gLh) (m/s²) */
	double gcc[3];   /* 含哥氏项的有害加速度 gn - wnie×vn (m/s²) */
	double pos[3];   /* 位置 [纬度 lat(rad), 经度 lon(rad), 高度 h(m)], 由 imu.pos 拷入 */
};

/* IMU 信息(原始数据/标定参数/姿态速度位置/杆臂/采样等) */
struct Imu_Infor
{
	double gyro_ori[3]; /* 原始陀螺角增量 [x,y,z](rad/帧), 已由速率×ts 转换 */
	double acc_ori[3];  /* 原始加表速度增量 [x,y,z](m/s·帧), 已由比力×ts 转换 */
	float T_gyro[3];    /* 三轴陀螺温度 (℃), 温补多项式自变量 */
	float T_acc[3];     /* 三轴加表温度 (℃), 温补多项式自变量 */

	double errpara_TC[30]; /* 温补级误差参数: GetErrorPara_TC 写入, 供第一级 ErrorComp 解包 */
	double gyro_TC[3];     /* 温补级输出陀螺角增量 (rad/帧), 温补与标定之间的中间结果 */
	double acc_TC[3];      /* 温补级输出加表速度增量 (m/s·帧), 温补与标定之间的中间结果 */

	double para_calerr[30]; /* 标定级误差参数: 标度/零偏/安装误差装订值(原 errpara_cal) */

	double data_gyro[3]; /* 标定级补偿后陀螺角增量 (rad/帧), 供对准/惯导递推消费 */
	double data_acc[3];  /* 标定级补偿后加表速度增量 (m/s·帧), 供对准/惯导递推消费 */

	double tau_g[3];//MEMS陀螺漂移的相关系数
	double tau_a[3];//MEMS加表零偏的相关系数
	
	double cnb[3][3]; /* 姿态方向余弦阵(机体系->导航系), 惯导递推核心状态 */
	double att[3];    /* 姿态角 [pitch, roll, yaw](rad), 由 m2att(cnb) 刷新 */
	double vn[3];     /* 载体速度 [E, N, U] (m/s) */
	double pos[3];    /* 位置 [纬度 lat(rad), 经度 lon(rad), 高度 h(m)] */

	double phim[3];    /* 当前姿态失准角/旋转矢量 φ (rad) */
	double phimk_1[3]; /* 上一时刻旋转矢量 φ(k-1) (rad), 双子样用 */
	double dvbm[3];    /* 机体系速度增量(比力积分) Δv_b (m/s) */
	double fnsf[3];    /* 导航系比力(去除有害加速度后) f^n (m/s²) */
	double wbeb[3];    /* 机体系陀螺角增量(补偿后) ω_b·Δt (rad) */
	double an[3];      /* 载体加速度导航系 a^n (m/s²) */

	double r_inlearm[9]; /* 内杆臂参数(IMU内敏感轴不一致), 3x3 row-major */

	double ts;        /* 单帧采样时间 (s), 200Hz 时为 0.005 */
	unsigned char fs; /* 采样率 (Hz) */
	unsigned char nn; /* 子样数(一个 nts 周期内采样数), 双子样为 2 */
	double nts;       /* 导航更新周期 (s), 双子样 0.01 */

	unsigned char flag_ValidIMU; /* IMU 数据有效标志: 1=本帧有有效测量 */
	unsigned char flag_navsuc;   /* 导航解算成功标志 */
};

/* 系统信息(GNSS/对准时间/杆臂与时间不同步补偿/各阶段标志位) */
struct Sys_Infor
{
	double cnb[3][3]; /* 系统级姿态阵备份(机体系->导航系) */
	double att[3];    /* 系统级姿态角 [pitch, roll, yaw](rad) */
	double vn[3];     /* 系统级速度 [E, N, U] (m/s) */
	double pos[3];    /* 系统级位置 [lat(rad), lon(rad), h(m)] */

	double vn_GNSS[3];  /* GNSS 速度观测 [E, N, U] (m/s) */
	double pos_GNSS[3]; /* GNSS 位置观测 [lat(rad), lon(rad), h(m)] */

	double t_align; /* 对准总时长装订 (s) */
	
	char flag_GNSSvalid;
	char flag_azimuthvalid;
	char flag_GNSSfault;

	double r_outlearm[3]; /* 外杆臂(IMU中心到GNSS天线)空间向量 [E,N,U] (m) */
	double dvnL[3];       /* 外杆臂引起的速度补偿量 (m/s) */
	double dpGL[3];       /* 外杆臂引起的位置补偿量 (m 或 rad) */

	double deltat;   /* GNSS/IMU 时间不同步误差 δt (s) */
	double dvndt[3]; /* 时间不同步引起的速度补偿 (m/s) */
	double dpdt[3];  /* 时间不同步引起的位置补偿 (m 或 rad) */

	unsigned char flag_Finish_calign;    /* 粗对准完成标志: 1=已完成 */
	unsigned char flag_Finish_palign;    /* 精对准完成标志: 1=已完成 */
	unsigned char flag_PreciseAlignInit; /* 精对准 Kalman 已初始化标志 */
	unsigned char flag_NavIntegInit;     /* 组合导航 19d Kalman 已初始化标志 */
	unsigned char flag_ValidGNSS;        /* GNSS 观测有效标志: 1=本帧可量测(量测后清零) */
	unsigned char flag_KalmanSuc;        /* Kalman 解算成功标志 */
};

/* 水平对准/姿态参考 */
struct AHRS_Infor
{
	double average_accdata[3];  /* 静止段加表多帧平均(速度增量/比力) */
	double average_gyrodata[3]; /* 静止段陀螺多帧平均(角增量/角速率) */
	double mod_accdata;         /* 加表平均向量模值 */
	double mod_gyrodata;        /* 陀螺平均向量模值 */
	double cnb_align[3][3];     /* 对准时解算的姿态阵 */
	double att_align[3];        /* 对准时姿态角 [pitch, roll, yaw](rad) */
	unsigned char t_HorAlign;   /* 水平对准计时 (s) */

	unsigned char Flag_HorAlignFinish; /* 水平对准完成标志 */
	unsigned char Flag_ValidHorAlign;  /* 水平对准结果有效标志 */
	unsigned char flag_EnvFactor;      /* 环境因子标志: 1=环境满足对准条件(如静止/低振动) */

	double cnb[3][3]; /* 当前姿态阵备份(机体系->导航系) */
};

/* 7 维卡尔曼滤波器状态(水平失准角 + 速度,用于精对准)
 * 状态向量 x 顺序: [φ_E, φ_N, φ_U(失准角), dv_E, dv_N, dv_U(速度误差), eb(陀螺漂移)] */
struct Kalman7d_State
{
	double Phikk_1[7][7]; /* 一步状态转移阵 Φ(k,k-1) */
	double Phik[7][7];    /* 当前状态转移阵 Φ(k) */
	double Tauk[7][6];    /* 噪声驱动阵 τ(7x6) */
	double Pkk_1[7][7];   /* 一步预测协方差 P(k,k-1) */
	double Pk[7][7];      /* 估计协方差 P(k) */
	double xkk_1[7];      /* 一步预测状态 x(k,k-1) */
	double xk[7];         /* 后验状态估计 x(k) */
	double qk[6][6];      /* 系统噪声协方差 Q(6x6) */
	double rk[2][2];      /* 量测噪声协方差 R(2x2, 水平两速度观测) */
	double Hk[2][7];      /* 量测阵 H(2x7) */
};

/* 7d Kalman 启动参数(初始方差/噪声系数装订) */
struct Kalman7d_Starter
{
	double pk_phipr; /* 水平失准角(北)初始标准差 */
	double pk_phiy;  /* 水平失准角(东)初始标准差 */
	double pk_dvn;   /* 速度误差初始标准差 */
	double pk_eb;    /* 陀螺常值漂移初始标准差 */

	double qt_gyro; /* 陀螺噪声相关系数(过程噪声) */
	double qt_acc;  /* 加表噪声相关系数(过程噪声) */
	double rk0;     /* 量测噪声标准差 */
};

/* 12 维卡尔曼滤波器状态(位置 + 速度 + 姿态 + 加表偏置) */
struct Kalman12d_State
{
	double Phikk_1[12][12]; /* 一步状态转移阵 Φ(k,k-1) */
	double Phik[12][12];    /* 当前状态转移阵 Φ(k) */
	double Tauk[12][6];     /* 噪声驱动阵 τ(12x6) */
	double Pkk_1[12][12];   /* 一步预测协方差 P(k,k-1) */
	double Pk[12][12];      /* 估计协方差 P(k) */
	double xkk_1[12];       /* 一步预测状态 x(k,k-1) */
	double xk[12];          /* 后验状态估计 x(k) */
	double qk[6][6];        /* 系统噪声协方差 Q(6x6) */
	double rk[3][3];        /* 量测噪声协方差 R(3x3) */
	double Hk[3][12];       /* 量测阵 H(3x12) */
};

/* 12d Kalman 启动参数 */
struct Kalman12d_Starter
{
	double pk_phipr;  /* 水平失准角(北)初始标准差 */
	double pk_phiy;   /* 水平失准角(东)初始标准差 */
	double pk_dvn;    /* 速度误差初始标准差 */
	double pk_dposLL; /* 水平位置(经纬)误差初始标准差 */
	double pk_dposH;  /* 高度误差初始标准差 */
	double pk_eb;     /* 陀螺常值漂移初始标准差 */
	double pk_db;     /* 加表零偏初始标准差 */
	double pk_dl;     /* 杆臂误差初始标准差 */
	double pk_dt;     /* 时间同步误差初始标准差 */
	double qt_gyro;   /* 陀螺噪声相关系数 */
	double qt_acc;    /* 加表噪声相关系数 */
	double rk_dvn;    /* 速度量测噪声标准差 */
	double rk_dposLL; /* 水平位置量测噪声标准差 */
	double rk_dposH;  /* 高度量测噪声标准差 */
	double rk0;       /* 量测噪声(与7d一致,统一初值) */
};

/* 19 维卡尔曼滤波器状态(组合导航全状态)
 * 状态向量 x[19]: 失准角φ(3) + 速度误差dv(3) + 位置误差dp(3)
 *                 + 陀螺漂移eb(3) + 加表零偏db(3) + 杆臂dl(3)
 *                 + 时间同步误差 x[18](其余1个为相关扩展项) */
struct Kalman19d_State
{
	double Phikk_1[19][19]; /* 一步状态转移阵 Φ(k,k-1) */
	double Phik[19][19];    /* 当前状态转移阵 Φ(k) */
	double Tauk[19][6];     /* 噪声驱动阵 τ(19x6) */
	double Pkk_1[19][19];   /* 一步预测协方差 P(k,k-1) */
	double Pk[19][19];      /* 估计协方差 P(k) */
	double xkk_1[19];       /* 一步预测状态 x(k,k-1) */
	double xk[19];          /* 后验状态估计 x(k)(含 x[18] 时间同步误差) */
	double qk[6][6];        /* 系统噪声协方差 Q(6x6) */
	double rk[6][6];        /* 量测噪声协方差 R(6x6, 速度3+位置3观测) */
	double Hk[6][19];       /* 量测阵 H(6x19) */
};

/* 19d Kalman 启动参数 */
struct Kalman19d_Starter
{
	double pk_phipr;  /* 水平失准角(北)初始标准差 */
	double pk_phiy;   /* 水平失准角(东)初始标准差 */
	double pk_dvn;    /* 速度误差初始标准差 */
	double pk_dposLL; /* 水平位置(经纬)误差初始标准差 */
	double pk_dposH;  /* 高度误差初始标准差 */
	double pk_eb;     /* 陀螺常值漂移初始标准差 */
	double pk_db;     /* 加表零偏初始标准差 */
	double pk_dl;     /* 杆臂误差初始标准差 */
	double pk_dt;     /* 时间同步误差初始标准差 */
	double qt_gyro;   /* 陀螺噪声相关系数 */
	double qt_acc;    /* 加表噪声相关系数 */
	double rk_dvn;    /* 速度量测噪声标准差 */
	double rk_dposLL; /* 水平位置量测噪声标准差 */
	double rk_dposH;  /* 高度量测噪声标准差 */
};

/* IMU 温度补偿参数: 各类误差关于温度的 5 阶多项式系数 [a0..a4],
 * 求值形式 p(T) = a0 + a1·T + a2·T² + a3·T³ + a4·T⁴ */
struct Imu_TempCompPara
{
	/* 加表标度因数温度系数 */
	double aT_K1x[5]; /* X 加表标度 K1x 多项式 */
	double aT_K1y[5]; /* Y 加表标度 K1y 多项式 */
	double aT_K1z[5]; /* Z 加表标度 K1z 多项式 */

	/* 加表安装误差(非正交)温度系数 */
	double aT_dKyx[5]; /* dK_yx 安装误差多项式 */
	double aT_dKzx[5]; /* dK_zx 安装误差多项式 */
	double aT_dKxy[5]; /* dK_xy 安装误差多项式 */
	double aT_dKzy[5]; /* dK_zy 安装误差多项式 */
	double aT_dKxz[5]; /* dK_xz 安装误差多项式 */
	double aT_dKyz[5]; /* dK_yz 安装误差多项式 */

	/* 加表零偏温度系数 */
	double aT_dbx[5]; /* X 加表零偏 dbx 多项式 */
	double aT_dby[5]; /* Y 加表零偏 dby 多项式 */
	double aT_dbz[5]; /* Z 加表零偏 dbz 多项式 */

	/* 陀螺标度因数温度系数 */
	double aT_E1x[5]; /* X 陀螺标度 E1x 多项式 */
	double aT_E1y[5]; /* Y 陀螺标度 E1y 多项式 */
	double aT_E1z[5]; /* Z 陀螺标度 E1z 多项式 */

	/* 陀螺安装误差(非正交)温度系数 */
	double aT_dEyx[5]; /* dE_yx 安装误差多项式 */
	double aT_dEzx[5]; /* dE_zx 安装误差多项式 */
	double aT_dExy[5]; /* dE_xy 安装误差多项式 */
	double aT_dEzy[5]; /* dE_zy 安装误差多项式 */
	double aT_dExz[5]; /* dE_xz 安装误差多项式 */
	double aT_dEyz[5]; /* dE_yz 安装误差多项式 */

	/* 陀螺常值漂移(零偏)温度系数 */
	double aT_ebx[5]; /* X 陀螺漂移 ebx 多项式 */
	double aT_eby[5]; /* Y 陀螺漂移 eby 多项式 */
	double aT_ebz[5]; /* Z 陀螺漂移 ebz 多项式 */
};

#endif /* NAVI_TYPES_H */
