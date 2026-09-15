/**
 * @file mixer.h
 * @brief X 型四轴混控器 (Mixer)
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t motor[4]; /**< 4 路电机的目标输出值 (DShot: 0 或 48~2047) */
} MotorOutput_t;

/**
 * @brief 计算 4 路电机输出
 * @param output 电机输出句柄
 * @param throttle 飞行员油门基础指令 (0 ~ 2000)
 * @param roll_torque 滚转控制力矩
 * @param pitch_torque 俯仰控制力矩
 * @param yaw_torque 偏航控制力矩
 * @param is_armed 当前是否处于解锁安全状态 (true 为解锁，false 强制电机停转输出 0)
 */
void Mixer_Update(MotorOutput_t *output, float throttle, float roll_torque, float pitch_torque, float yaw_torque, bool is_armed);

#ifdef __cplusplus
}
#endif

#endif /* MIXER_H */
