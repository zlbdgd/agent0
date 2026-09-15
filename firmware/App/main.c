/**
 * @file main.c
 * @brief 自研嵌入式穿越机飞控主任务架构与主循环实现
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "../include/config.h"
#include "../Middlewares/filter.h"
#include "../Middlewares/ahrs_mahony.h"
#include "../Middlewares/pid.h"
#include "../Middlewares/mixer.h"
#include "../Devices/dev_icm42688.h"
#include "../Devices/dev_crsf.h"
#include "../Drivers/drv_dshot.h"
#include "failsafe.h"
#include "scheduler.h"

/* ================= 全局系统变量 ================= */
static ICM42688_t g_imu;
static MahonyAHRS_t g_ahrs;
static CRSF_Receiver_t g_rc;
static SafetyManager_t g_safety;
static MotorOutput_t g_motors;

/* 低通滤波器 */
static LowPassFilter_t g_gyro_lpf[3];
static LowPassFilter_t g_acc_lpf[3];

/* 串级 PID 控制器 */
static PID_t g_pid_angle_roll;
static PID_t g_pid_angle_pitch;
static PID_t g_pid_rate_roll;
static PID_t g_pid_rate_pitch;
static PID_t g_pid_rate_yaw;

/* 遥测与监控数据 */
static float g_vbat_volts = 7.4f;

/* 硬件总线桩函数 (与实际 STM32 HAL 绑定) */
static uint8_t hal_spi_transfer(uint8_t data) {
    /* 实际移植时调用: return (uint8_t)HAL_SPI_TransmitReceive(...); */
    return 0x47; /* 假定测试桩 */
}
static void hal_cs_control(bool select) {
    /* 实际移植时操作 PC4 引脚拉高或拉低 */
}
static void hal_delay_ms(uint32_t ms) {
    /* 实际移植时调用 HAL_Delay(ms); */
}

static const ICM42688_Bus_t g_imu_bus = {
    .spi_transfer = hal_spi_transfer,
    .cs_control   = hal_cs_control,
    .delay_ms     = hal_delay_ms
};

/* ================= 任务定义 ================= */

/**
 * @brief 1000Hz 核心控制循环: IMU采集 -> 滤波 -> 姿态解算 -> 内环角速度PID -> 电机输出
 */
static void Task_1000Hz_AttitudeControl(uint32_t current_time_us) {
    /* 1. 读取 IMU 数据 */
    ICM42688_ReadSensors(&g_imu, &g_imu_bus);

    /* 2. 低通滤波 (滤除电机 200Hz+ 高频机械噪声) */
    float gx = LPF_Update(&g_gyro_lpf[0], g_imu.gyro_dps[0]);
    float gy = LPF_Update(&g_gyro_lpf[1], g_imu.gyro_dps[1]);
    float gz = LPF_Update(&g_gyro_lpf[2], g_imu.gyro_dps[2]);

    float ax = LPF_Update(&g_acc_lpf[0], g_imu.accel_g[0]);
    float ay = LPF_Update(&g_acc_lpf[1], g_imu.accel_g[1]);
    float az = LPF_Update(&g_acc_lpf[2], g_imu.accel_g[2]);

    /* 3. 姿态解算 (Mahony 互补滤波) */
    Mahony_Update(&g_ahrs, 
                  gx * 0.0174533f, gy * 0.0174533f, gz * 0.0174533f, 
                  ax, ay, az, 
                  LOOP_PERIOD_S);

    /* 4. 外环角度 PID 计算 (自稳模式 Angle Mode) */
    /* 摇杆映射: 1000~2000 -> -MAX_TILT_ANGLE ~ +MAX_TILT_ANGLE */
    float target_roll_angle  = ((float)g_rc.channels[0] - 1500.0f) / 500.0f * MAX_TILT_ANGLE_DEG;
    float target_pitch_angle = ((float)g_rc.channels[1] - 1500.0f) / 500.0f * MAX_TILT_ANGLE_DEG;
    float target_yaw_rate    = ((float)g_rc.channels[3] - 1500.0f) / 500.0f * MAX_YAW_RATE_DPS;

    /* 外环输出目标角速度 */
    float target_roll_rate  = PID_Update(&g_pid_angle_roll, target_roll_angle, g_ahrs.roll, LOOP_PERIOD_S);
    float target_pitch_rate = PID_Update(&g_pid_angle_pitch, target_pitch_angle, g_ahrs.pitch, LOOP_PERIOD_S);

    /* 5. 内环角速度 PID 计算 (以 1000Hz 抵御阵风与角速度偏差) */
    float roll_torque  = PID_Update(&g_pid_rate_roll, target_roll_rate, gx, LOOP_PERIOD_S);
    float pitch_torque = PID_Update(&g_pid_rate_pitch, target_pitch_rate, gy, LOOP_PERIOD_S);
    float yaw_torque   = PID_Update(&g_pid_rate_yaw, target_yaw_rate, gz, LOOP_PERIOD_S);

    /* 6. 油门归一化映射 (1000~2000 -> 0 ~ 2000) */
    float base_throttle = (float)(g_rc.channels[2] > 1000 ? g_rc.channels[2] - 1000 : 0) * 2.0f;

    /* 7. 混控器解算 4 路电机输出 */
    bool is_armed = (g_safety.state == FLIGHT_STATE_ARMED);
    Mixer_Update(&g_motors, base_throttle, roll_torque, pitch_torque, yaw_torque, is_armed);

    /* 8. 触发 DShot DMA 驱动发送指令给 4 个电调 */
    /* 实际移植时调用: DShot_SendAll(g_motors.motor); */
}

/**
 * @brief 100Hz 遥控与安全监控: CRSF 超时检测与解锁状态机更新
 */
static void Task_100Hz_SafetyAndRC(uint32_t current_time_us) {
    uint32_t now_ms = current_time_us / 1000U;
    CRSF_CheckTimeout(&g_rc, now_ms);
    Safety_Update(&g_safety, &g_rc, &g_ahrs, g_imu.is_calibrated, g_vbat_volts);

    /* 若进入未解锁或失控保护，清空 PID 历史积分 */
    if (g_safety.state != FLIGHT_STATE_ARMED) {
        PID_Reset(&g_pid_rate_roll);
        PID_Reset(&g_pid_rate_pitch);
        PID_Reset(&g_pid_rate_yaw);
        PID_Reset(&g_pid_angle_roll);
        PID_Reset(&g_pid_angle_pitch);
    }
}

/**
 * @brief 10Hz 低频状态监控: 电池 ADC 电压与 LED 心跳灯
 */
static void Task_10Hz_TelemetryAndLED(uint32_t current_time_us) {
    /* 实际移植时读取 ADC1_IN11 采样并乘以分压系数 BAT_VOLTAGE_DIVIDER */
    /* 状态指示灯逻辑: 解锁时蓝灯长亮，未解锁时 1Hz 闪烁，失控保护时急闪 */
}

/* ================= 调度表 ================= */
static Task_t g_tasks[] = {
    { Task_1000Hz_AttitudeControl, 1000U, 0U }, /* 1ms (1000Hz) */
    { Task_100Hz_SafetyAndRC,     10000U, 0U }, /* 10ms (100Hz) */
    { Task_10Hz_TelemetryAndLED, 100000U, 0U }  /* 100ms (10Hz) */
};
#define TASK_COUNT (sizeof(g_tasks) / sizeof(Task_t))

void Scheduler_Init(void) {
    for (size_t i = 0; i < TASK_COUNT; i++) {
        g_tasks[i].last_executed_us = 0;
    }
}

void Scheduler_Run(uint32_t current_time_us) {
    for (size_t i = 0; i < TASK_COUNT; i++) {
        if ((current_time_us - g_tasks[i].last_executed_us) >= g_tasks[i].period_us) {
            g_tasks[i].last_executed_us = current_time_us;
            g_tasks[i].function(current_time_us);
        }
    }
}

/* ================= 系统启动入口 ================= */
int main(void) {
    /* 1. 初始化低通滤波器 (截止频率 90Hz) */
    for (int i = 0; i < 3; i++) {
        LPF_Init(&g_gyro_lpf[i], 90.0f, (float)LOOP_RATE_HZ);
        LPF_Init(&g_acc_lpf[i],  30.0f, (float)LOOP_RATE_HZ);
    }

    /* 2. 初始化 Mahony 姿态解算器 (Kp = 1.0, Ki = 0.01) */
    Mahony_Init(&g_ahrs, 1.0f, 0.01f);

    /* 3. 初始化双环 PID 控制器参数 */
    /* 外环角度环: P = 4.5 */
    PID_Init(&g_pid_angle_roll,  4.5f, 0.0f, 0.0f, 0.0f, MAX_ROLL_RATE_DPS, 0.0f, LOOP_RATE_HZ);
    PID_Init(&g_pid_angle_pitch, 4.5f, 0.0f, 0.0f, 0.0f, MAX_PITCH_RATE_DPS, 0.0f, LOOP_RATE_HZ);

    /* 内环角速度环: P = 0.8, I = 0.15, D = 0.025 */
    PID_Init(&g_pid_rate_roll,  0.80f, 0.15f, 0.025f, 400.0f, 500.0f, 50.0f, LOOP_RATE_HZ);
    PID_Init(&g_pid_rate_pitch, 0.80f, 0.15f, 0.025f, 400.0f, 500.0f, 50.0f, LOOP_RATE_HZ);
    PID_Init(&g_pid_rate_yaw,   1.20f, 0.20f, 0.000f, 400.0f, 500.0f, 50.0f, LOOP_RATE_HZ);

    /* 4. 初始化设备与安全状态机 */
    CRSF_Init(&g_rc);
    Safety_Init(&g_safety);
    ICM42688_Init(&g_imu, &g_imu_bus);
    ICM42688_CalibrateGyro(&g_imu, &g_imu_bus, 500);

    /* 5. 启动时间片调度器 */
    Scheduler_Init();

    uint32_t mock_time_us = 0;
    while (1) {
        /* 在真实 STM32 硬件上使用 TIM2/TIM5 32位高精度计数器获取微秒数:
         * uint32_t current_time_us = __HAL_TIM_GET_COUNTER(&htim2);
         */
        mock_time_us += 100;
        Scheduler_Run(mock_time_us);
    }

    return 0;
}
