/**
 * @file pid.c
 * @brief 双环串级 PID 算法核心实现
 */

#include "pid.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_max, float out_max, float d_lpf_cutoff, float sample_freq) {
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_max = fabsf(integral_max);
    pid->out_max = fabsf(out_max);
    pid->out_min = -pid->out_max;
    pid->prev_meas = 0.0f;
    pid->last_d = 0.0f;

    if (d_lpf_cutoff > 0.0f && sample_freq > 0.0f) {
        float rc = 1.0f / (2.0f * (float)M_PI * d_lpf_cutoff);
        float dt = 1.0f / sample_freq;
        pid->d_lpf_alpha = dt / (rc + dt);
    } else {
        pid->d_lpf_alpha = 1.0f; /* 不滤波 */
    }
}

void PID_Reset(PID_t *pid) {
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->last_d = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt) {
    if (pid == 0 || dt <= 0.0f) return 0.0f;

    /* 1. 误差计算 */
    float error = setpoint - measurement;

    /* 2. 比例项 (P) */
    float p_out = pid->kp * error;

    /* 3. 积分项 (I) + 积分抗饱和 (Anti-Windup) */
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    /* 4. 微分项 (D) - 采用测量值微分 (Derivative on Measurement)，消除打杆突变带来的微分冲击 */
    float raw_d = -(measurement - pid->prev_meas) / dt;
    pid->prev_meas = measurement;

    /* 对微分信号实施低通滤波，极大减轻高频机械噪声放大造成的电机发烫 */
    pid->last_d = pid->last_d + pid->d_lpf_alpha * (raw_d - pid->last_d);
    float d_out = pid->kd * pid->last_d;

    /* 5. 合成总输出并限幅 */
    float output = p_out + pid->integral + d_out;
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}
