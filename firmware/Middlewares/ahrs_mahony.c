/**
 * @file ahrs_mahony.c
 * @brief Mahony 互补滤波姿态解算算法实现
 */

#include "ahrs_mahony.h"
#include <math.h>

#define RAD_TO_DEG  57.29577951308232f

/**
 * @brief 快速反平方根算法 (Fast Inverse Square Root)
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(MahonyAHRS_t *ahrs, float kp, float ki) {
    if (ahrs == 0) return;
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->roll = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw = 0.0f;
    ahrs->kp = kp;
    ahrs->ki = ki;
    ahrs->e_int_x = 0.0f;
    ahrs->e_int_y = 0.0f;
    ahrs->e_int_z = 0.0f;
}

void Mahony_Update(MahonyAHRS_t *ahrs, 
                   float gx, float gy, float gz, 
                   float ax, float ay, float az, 
                   float dt) {
    if (ahrs == 0 || dt <= 0.0f) return;

    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    /* 仅当加速度计读数有效时进行重力校准补偿 (排除自由落体或超大加速度) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* 加速度计归一化 */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 计算由四元数推导出的理论重力向量 v (在机体坐标系下的投影) */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* 实测重力向量 a 与理论重力向量 v 的叉乘，得到姿态误差向量 e */
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        /* 误差积分，用于修正陀螺仪零偏 */
        if (ahrs->ki > 0.0f) {
            ahrs->e_int_x += ex * ahrs->ki * dt;
            ahrs->e_int_y += ey * ahrs->ki * dt;
            ahrs->e_int_z += ez * ahrs->ki * dt;
            gx += ahrs->e_int_x;
            gy += ahrs->e_int_y;
            gz += ahrs->e_int_z;
        }

        /* 比例反馈补偿陀螺仪 */
        gx += ahrs->kp * ex;
        gy += ahrs->kp * ey;
        gz += ahrs->kp * ez;
    }

    /* 一阶数值积分更新四元数微分方程 */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* 重新归一化四元数 */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    ahrs->q0 = q0 * recipNorm;
    ahrs->q1 = q1 * recipNorm;
    ahrs->q2 = q2 * recipNorm;
    ahrs->q3 = q3 * recipNorm;

    /* 四元数转换为机体欧拉角 (单位: 度) */
    ahrs->roll  = atan2f(2.0f * (ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3), 
                         1.0f - 2.0f * (ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2)) * RAD_TO_DEG;
    
    float sinp = 2.0f * (ahrs->q0 * ahrs->q2 - ahrs->q3 * ahrs->q1);
    if (fabsf(sinp) >= 1.0f) {
        ahrs->pitch = copysignf(90.0f, sinp); /* 奇异点保护 (俯仰 ±90度) */
    } else {
        ahrs->pitch = asinf(sinp) * RAD_TO_DEG;
    }

    ahrs->yaw   = atan2f(2.0f * (ahrs->q0 * ahrs->q3 + ahrs->q1 * ahrs->q2), 
                         1.0f - 2.0f * (ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3)) * RAD_TO_DEG;
}
