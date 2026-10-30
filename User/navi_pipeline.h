/**
 * @file navi_pipeline.h
 * @brief 组合导航流水线编排层
 *
 * 职责: 串联 Navi/ 算法包各模块, 把 MCU 主循环 (main.c) 喂入的原始 IMU + 温度数据
 *        转成完整 INS + 15 维 Kalman 融合输出. 内部持有 navi 所有状态
 *        (g_imu/g_ethi/g_sys/g_ahrs/gnss_info/g_k15s/imu_temp_para), 外部不可见.
 *
 * 主循环调用链 (main.c while(1)):
 *   for(;;) {
 *     gnss1_rx_pump();                         // GNSS1 DMA 环缓冲帧解析
 *     if(imu_sample_due) {                     // 5ms TIMER3 ISR 置位
 *       SPI 采样 + 坐标映射 + navi_pipeline_feed_imu(accel, gyro, gyro_temp, accel_temp);
 *     }
 *     navi_pipeline_run_kalman();              // flag_navsuc 触发 Kalman 全链路
 *     if(navi_push_div >= 10) {                // NAVI_PUSH_DIV=10 → 20Hz 上报
 *       navi_pipeline_get_state(...); nav_send_frame(...); navi_push_div = 0;
 *     }
 *   }
 *
 * feed_imu 内部 7 步流水线 (每 5ms):
 *   0. GNSS 数据填充 (GNSS1=真机 gnss1_rx, GNSS2=stub)
 *   1. IMU 原始量解包 (accel/gyro → gyro_ori/acc_ori)
 *   2. 温补参数 GetErrorPara_TC (T_gyro/T_acc → errpara_TC)
 *   3. ErrorComp ×2 (温补级 → 标定级 → data_gyro/data_acc)
 *   4. GetGNSSType + IsGNSSDataValid (判定 GNSS 类型 + 有效性)
 *   5. 粗对准 3 步状态机 (CoarseAlign_Horizon_RT + 固定航向装订 GNSS1_DEFAULT_HEADING_DEG)
 *   6. ins_update_RT 双子样 (对准完成后跑, flag_navsuc 每 10ms 置 1)
 *
 * run_kalman 内部 3 步流水线 (每 10ms):
 *   kalman15d_update(GetKalman15d_SSMat → TimeUpdate)
 *   → Kalman15d_MesUpdate (含 mat_inverse 6×6 / 7×7)
 *   → Kalman_Feedback
 *
 * 算法参数装订: ts=0.005, fs=200, t_align=30s, 标度恒等 para_calerr[9..11]=1, [24..26]=1
 */
#ifndef NAVI_PIPELINE_H
#define NAVI_PIPELINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void navi_pipeline_init(void);

    void navi_pipeline_feed_imu(const float accel_mps2[3], const float gyro_radps[3],
                                const float gyro_temp_c[3], float accel_temp_c);

    uint8_t navi_pipeline_run_kalman(void);

    void navi_pipeline_get_state(double att_rad[3], double vn_mps[3], double pos[3]);

#ifdef __cplusplus
}
#endif

#endif /* NAVI_PIPELINE_H */
