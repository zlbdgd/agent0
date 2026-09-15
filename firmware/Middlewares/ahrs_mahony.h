/**
 * @file ahrs_mahony.h
 * @brief Mahony 互补滤波姿态解算算法 (AHRS)
 */

#ifndef AHRS_MAHONY_H
#define AHRS_MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0, q1, q2, q3;      /**< 当前姿态四元数 */
    float roll, pitch, yaw;    /**< 解算出的机体欧拉角 (度, deg) */
    float kp;                  /**< 比例增益 (加速度计重力向量校正权重) */
    float ki;                  /**< 积分增益 (陀螺仪零偏补偿积分权重) */
    float e_int_x;             /**< 零偏误差积分项 X */
    float e_int_y;             /**< 零偏误差积分项 Y */
    float e_int_z;             /**< 零偏误差积分项 Z */
} MahonyAHRS_t;

/**
 * @brief 初始化 Mahony 姿态解算器
 * @param ahrs 句柄指针
 * @param kp 比例增益 (推荐 0.5f ~ 2.0f)
 * @param ki 积分增益 (推荐 0.0f ~ 0.05f)
 */
void Mahony_Init(MahonyAHRS_t *ahrs, float kp, float ki);

/**
 * @brief 更新姿态四元数与欧拉角
 * @param ahrs 句柄指针
 * @param gx 陀螺仪角速度 X (rad/s)
 * @param gy 陀螺仪角速度 Y (rad/s)
 * @param gz 陀螺仪角速度 Z (rad/s)
 * @param ax 加速度计 X (已归一化或任意等比例单位)
 * @param ay 加速度计 Y
 * @param az 加速度计 Z
 * @param dt 采样周期时间步长 (秒, 如 0.001f 代表 1kHz)
 */
void Mahony_Update(MahonyAHRS_t *ahrs, 
                   float gx, float gy, float gz, 
                   float ax, float ay, float az, 
                   float dt);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_MAHONY_H */
