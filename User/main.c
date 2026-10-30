#include "gd32f4xx.h"
#include "gd32f4xx_libopt.h"
#include "systick.h"
#include "timer.h"
#include "ADXL357.h"
#include "XV7021.h"
#include "uart.h"
#include "IMU_FRAME.h"
#include "version.h"
#include "navi_pipeline.h"
#include "gnss1_rx.h"
#include "navi_config/navi_constants.h"
#include <math.h>

/* ---- 硬件参数 ---- */
#define IMU_SAMPLE_PERIOD_MS 5U
#define ALL_GYROS_VALID_MASK 0x07U
#define STANDARD_GRAVITY_M_S2 9.80665f
#define DPS_TO_RADPS ((float)arcdeg) /* 度/秒 -> rad/秒, 复用 navi_constants.h 定义 */

/* ---- 错误码 ---- */
#define MAIN_FAULT_ADXL357_INIT 1
#define MAIN_FAULT_XV7021_INIT 2
#define MAIN_FAULT_NAVI_INIT 3

/* ---- 致命故障停机 ----
 * 裸机环境下 main() 返回属未定义行为 (向量表无调用者, 会跳飞或复位),
 * 因此初始化失败时记录故障码后死循环停机:
 *   - g_main_fault_code 非 static, 便于 Keil Watch 直接查看故障原因
 *   - 关闭总中断, 避免外设未就绪时 ISR 继续运行 */
volatile uint32_t g_main_fault_code = 0U;

/**
 * @brief  初始化失败后的安全停机
 * @param  code  故障码 (MAIN_FAULT_xxx)
 * @note   本函数不会返回
 */
static void main_fault_halt(uint32_t code)
{
    g_main_fault_code = code;
    __disable_irq();
    for (;;)
    {
    }
}

/**
 * @brief  IMU + 组合导航主程序入口
 *
 * 初始化:
 *   ADXL357 (SPI1, ±10g, 1000Hz) + 3× XV7021BB (SPI0, 24bit dps)
 *   TIMER3 5ms 采样定时器 + UART0 230400 + 组合导航 navi_pipeline
 *
 * 主循环 (每 5ms):
 *   1. 读 ADXL357 三轴加速度原始值
 *   2. 读三颗 XV7021BB 角速度
 *   3. 坐标轴重映射 + 单位换算 → 机体系 accel (m/s²) + gyro (rad/s)
 *   4. navi_pipeline_feed_imu(): 逐帧喂 → 粗对准阶段 CoarseAlign_Horizon_RT /
 *      已对齐后 ins_update_RT 双子样 + 15d Kalman
 *   5. 导航帧 (ins_update_RT 双子样, ~10ms 一次)
 *   6. 原始 IMU 帧打包发送 (保留原功能)
 *   7. gnss1_rx_pump() 从 USART0 DMA 环缓冲解析 T_HJD_POSDATA 帧
 */
int main(void)
{
    int32_t accel_raw_x;
    int32_t accel_raw_y;
    int32_t accel_raw_z;
    uint16_t accel_temp_raw;
    uint8_t accel_valid;
    uint8_t gyro_index;
    xv7021bb_status_t sensor_init_status;
    xv7021bb_temp_t gyro_temp;

    /* ---- 机体系中间变量 ---- */
    float gyro_radps[3];
    float accel_mps2[3];
    float gyro_temp_c[3];
    float accel_temp_c_local;
    uint32_t navi_push_div;

    /* ---- 初始化 ----
     *
     * 顺序: 时钟 → SysTick → 外设 (UART0 DMA / 传感器 / TIMER3) → navi_pipeline → gnss1_rx
     *   - UART0_DMA_Init 独占配置 USART0 (GPIO/230400 8E1/TX DMA),
     *     gnss1_rx_init 只追加 RX DMA 通道 (DMA1_CH2) 与环缓冲状态
     *   - TIMER3 5ms ISR 只置 imu_sample_due=1, 主循环消费 (避免 ISR 内跑 SPI/DWT)
     *   - navi_pipeline_init 必须在所有传感器 init 之后 (依赖 g_imu.ts/fs 等装订) */
    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    systick_config();

    UART0_DMA_Init(230400U);

    if (adxl357_init() != 0U)
        main_fault_halt(MAIN_FAULT_ADXL357_INIT);

    XV7021BB_BoardInit();
    sensor_init_status = XV7021BB_InitAll();
    if (sensor_init_status != XV7021BB_OK)
        main_fault_halt(MAIN_FAULT_XV7021_INIT);

    TIMER3_Init(IMU_SAMPLE_PERIOD_MS);

    /* 组合导航初始化 (依赖所有传感器已就绪) */
    navi_pipeline_init();

    /* USART0 GNSS1 接收初始化: 仅追加 RX 侧 (DMA1_CH2 循环模式 + DENR + 环缓冲)
     * USART0 模式 (230400 8E1) 已在 UART0_DMA_Init 中配置完毕, 此处不重复配置 */
    gnss1_rx_init();

    /* ---- 主循环骨架 ----
     *
     * 每次迭代:
     *   gnss1_rx_pump()              — GNSS1 DMA 帧解析
     *   if(sample_due) { SPI采样 + navi_pipeline_feed_imu() }
     *   navi_pipeline_run_kalman() — flag_navsuc 触发 Kalman 链
     *   if(navi_push_div>=10) nav_send_frame()
     */
    navi_push_div = 0U;

    for (;;)
    {
        /* ① GNSS1 数据泵 (从 DMA 环缓冲取帧解析) */
        gnss1_rx_pump();

        /* ② 采样事件
         *   TIMER3 ISR 只置 imu_sample_due=1, 主循环消费 */
        if (imu_sample_due == 1U)
        {
            imu_sample_due = 0U;

            /* --- SPI 采样 + 坐标映射 --- */
            accel_valid = adxl357_read_accel(&accel_raw_x, &accel_raw_y, &accel_raw_z);
            if (accel_valid != 0U)
            {
                adxl357_convert_to_g(accel_raw_x, accel_raw_y, accel_raw_z,
                                     &adxl357_data.x_g, &adxl357_data.y_g, &adxl357_data.z_g);
            }

            (void)XV7021BB_SampleTask();

            accel_temp_raw = adxl357_read_temp();
            if (accel_temp_raw != 0U)
                adxl357_data.temperature_c = adxl357_temp_to_celsius(accel_temp_raw);

            for (gyro_index = 0U; gyro_index < 3U; ++gyro_index)
            {
                if (XV7021BB_ReadTemp((xv7021bb_id_t)gyro_index, &gyro_temp) == XV7021BB_OK &&
                    gyro_temp.valid != 0U)
                    xv7021_data.temp[gyro_index] = gyro_temp;
            }

            /* --- 机体系 → 导航系坐标映射 (与 IMU_FRAME.c 规则一致) ---
             * 陀螺: navi_x = +xv7021[1], navi_y = -xv7021[2], navi_z = -xv7021[0]
             * 加计: navi_x = -adxl357.y_g, navi_y = -adxl357.x_g, navi_z = -adxl357.z_g */
            gyro_radps[0] = (float)xv7021_data.gyro[1].dps * DPS_TO_RADPS;
            gyro_radps[1] = (float)-xv7021_data.gyro[2].dps * DPS_TO_RADPS;
            gyro_radps[2] = (float)-xv7021_data.gyro[0].dps * DPS_TO_RADPS;

            accel_mps2[0] = -adxl357_data.y_g * STANDARD_GRAVITY_M_S2;
            accel_mps2[1] = -adxl357_data.x_g * STANDARD_GRAVITY_M_S2;
            accel_mps2[2] = -adxl357_data.z_g * STANDARD_GRAVITY_M_S2;

            /* --- 温度: T_gyro[0..2] / T_acc[0..2] ---
             * xv7021_data.temp[i].degC °C (XV7021.h: 字段名 degC),
             * adxl357_data.temperature_c °C (ADXL357.h) */
            gyro_temp_c[0] = xv7021_data.temp[1].degC;
            gyro_temp_c[1] = xv7021_data.temp[2].degC;
            gyro_temp_c[2] = xv7021_data.temp[0].degC;
            accel_temp_c_local = adxl357_data.temperature_c;

            /* --- 喂 IMU ---
             * 内部: T_gyro/T_acc → gyro_ori/acc_ori → ErrorComp×2 → 粗对准状态机 → ins_update_RT */
            navi_pipeline_feed_imu(accel_mps2, gyro_radps, gyro_temp_c, accel_temp_c_local);

            /* 每帧 IMU navi_push_div +1 */
            navi_push_div++;
        }

        /* ③ Kalman 链 (flag_navsuc 触发, while(1) 外层统一跑)
         *   与 feed_imu 同层级, 不嵌在 sample_due 里 */
        navi_pipeline_run_kalman();

        /* ④ 导航上报 (navi_push_div >= NAVI_PUSH_DIV)
         *   NAVI_PUSH_DIV=10 → 200Hz/10 = 20Hz 上报; 在 Kalman 之后发, 保证融合值 */
        if (navi_push_div >= 10U)
        {
            navi_push_div = 0U;
            double att_nav[3], vn_nav[3], pos_nav[3];
            navi_pipeline_get_state(att_nav, vn_nav, pos_nav);
            nav_send_frame(att_nav, vn_nav, pos_nav);
        }
    }
}
