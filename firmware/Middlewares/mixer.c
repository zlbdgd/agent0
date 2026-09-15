/**
 * @file mixer.c
 * @brief X 型四轴混控器算法实现
 */

#include "mixer.h"
#include "../include/config.h"

static inline float constrain_float(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void Mixer_Update(MotorOutput_t *output, float throttle, float roll_torque, float pitch_torque, float yaw_torque, bool is_armed) {
    if (output == 0) return;

    /* 1. 若处于上锁状态 (Disarmed)，必须绝对切断动力，输出 0 */
    if (!is_armed) {
        output->motor[0] = DSHOT_DISARMED;
        output->motor[1] = DSHOT_DISARMED;
        output->motor[2] = DSHOT_DISARMED;
        output->motor[3] = DSHOT_DISARMED;
        return;
    }

    /* 2. 若油门低于起飞门限，保持最低怠速旋转 (AirMode / 待命) */
    if (throttle < (float)DSHOT_MIN_THROTTLE) {
        throttle = (float)DSHOT_MIN_THROTTLE;
    }

    /* 
     * 3. X 型四轴电机布局映射:
     *    M4 (右前 CCW)     M2 (左前 CW)
     *          \          /
     *           [ 飞控板 ]
     *          /          \
     *    M3 (右后 CW)      M1 (左后 CCW)
     */
    float m1 = throttle - roll_torque + pitch_torque + yaw_torque;
    float m2 = throttle - roll_torque - pitch_torque - yaw_torque;
    float m3 = throttle + roll_torque + pitch_torque - yaw_torque;
    float m4 = throttle + roll_torque - pitch_torque + yaw_torque;

    /* 4. 饱和限幅在 DShot 安全范围 [48, 2047] */
    output->motor[0] = (uint16_t)constrain_float(m1, (float)DSHOT_MIN_THROTTLE, (float)DSHOT_MAX_THROTTLE);
    output->motor[1] = (uint16_t)constrain_float(m2, (float)DSHOT_MIN_THROTTLE, (float)DSHOT_MAX_THROTTLE);
    output->motor[2] = (uint16_t)constrain_float(m3, (float)DSHOT_MIN_THROTTLE, (float)DSHOT_MAX_THROTTLE);
    output->motor[3] = (uint16_t)constrain_float(m4, (float)DSHOT_MIN_THROTTLE, (float)DSHOT_MAX_THROTTLE);
}
