/**
 * @file failsafe.c
 * @brief 解锁安全检查与失控状态机算法实现
 */

#include "failsafe.h"
#include "../include/config.h"
#include <math.h>

void Safety_Init(SafetyManager_t *safety) {
    if (safety == 0) return;
    safety->state = FLIGHT_STATE_DISARMED;
    safety->allow_arm = false;
    safety->arm_blocked_reason = 0;
}

void Safety_Update(SafetyManager_t *safety, 
                   const CRSF_Receiver_t *rx, 
                   const MahonyAHRS_t *ahrs, 
                   bool is_imu_calibrated,
                   float battery_voltage) {
    if (safety == 0 || rx == 0 || ahrs == 0) return;

    safety->arm_blocked_reason = 0;

    /* 1. 检查遥控通信是否存活 */
    if (!rx->is_connected) {
        safety->arm_blocked_reason |= ARM_BLOCK_RC_DISCONNECTED;
    }

    /* 2. 检查油门是否拉到最底部 (防误碰射出) */
    if (rx->channels[2] > RC_THROTTLE_MIN_CHECK) {
        safety->arm_blocked_reason |= ARM_BLOCK_THROTTLE_NOT_LOW;
    }

    /* 3. 检查 IMU 是否已完成零偏校准 */
    if (!is_imu_calibrated) {
        safety->arm_blocked_reason |= ARM_BLOCK_IMU_NOT_CALIBRATED;
    }

    /* 4. 检查机身放置倾角 (若倾角 > 25度，禁止在斜坡或手中盲目解锁) */
    if (fabsf(ahrs->roll) > 25.0f || fabsf(ahrs->pitch) > 25.0f) {
        safety->arm_blocked_reason |= ARM_BLOCK_TILT_TOO_LARGE;
    }

    /* 5. 电池严重过放检查 */
    if (battery_voltage > 1.0f && battery_voltage < (BATTERY_CRIT_CELL_VOLT * BATTERY_CELLS)) {
        safety->arm_blocked_reason |= ARM_BLOCK_BATTERY_CRITICAL;
    }

    safety->allow_arm = (safety->arm_blocked_reason == 0);

    /* 6. 状态机流转控制 (AUX1 通道为常用 ARM 解锁开关，大于 1600 为拨上) */
    bool arm_switch = (rx->channels[4] > RC_ARM_THRESHOLD);

    switch (safety->state) {
        case FLIGHT_STATE_DISARMED:
            if (arm_switch) {
                if (safety->allow_arm) {
                    safety->state = FLIGHT_STATE_ARMED;
                }
            }
            break;

        case FLIGHT_STATE_ARMED:
            /* 飞行中如果断开遥控链路，立刻切换为 FAILSAFE */
            if (!rx->is_connected) {
                safety->state = FLIGHT_STATE_FAILSAFE;
            } else if (!arm_switch) {
                /* 飞行员主动拨下解锁开关 */
                safety->state = FLIGHT_STATE_DISARMED;
            }
            break;

        case FLIGHT_STATE_FAILSAFE:
            /* 失控保护状态：必须先拨下上锁开关，且遥控重新连接后才能退出故障 */
            if (!arm_switch && rx->is_connected) {
                safety->state = FLIGHT_STATE_DISARMED;
            }
            break;

        default:
            safety->state = FLIGHT_STATE_DISARMED;
            break;
    }
}
