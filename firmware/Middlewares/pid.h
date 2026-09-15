/**
 * @file pid.h
 * @brief 工业级双环串级 PID 控制器 (防积分饱和 + 测量值微分 + 微分低通滤波)
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;               /**< 比例系数 */
    float ki;               /**< 积分系数 */
    float kd;               /**< 微分系数 */

    float integral;         /**< 积分累加值 */
    float integral_max;     /**< 积分上限 (抗饱和) */

    float prev_meas;        /**< 上一次的测量值 (用于微分先行) */
    float d_lpf_alpha;      /**< 微分项一阶低通滤波系数 */
    float last_d;           /**< 滤波后的上一次微分值 */

    float out_min;          /**< 最小输出限幅 */
    float out_max;          /**< 最大输出限幅 */
} PID_t;

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_max, float out_max, float d_lpf_cutoff, float sample_freq);

/**
 * @brief 重置 PID 积分与状态 (上锁 Disarm 时调用)
 */
void PID_Reset(PID_t *pid);

/**
 * @brief 位置式/微分先行 PID 计算
 * @param pid 控制器句柄
 * @param setpoint 期望目标值
 * @param measurement 当前传感器实际反馈值
 * @param dt 运行步长 (秒)
 * @return PID 控制输出力矩/指令
 */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
