/**
 * @file filter.h
 * @brief 数字低通滤波器与双二阶陷波滤波器
 */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 一阶低通滤波器结构体 */
typedef struct {
    float alpha;
    float prev_out;
} LowPassFilter_t;

/* 双二阶陷波滤波器 (Biquad Filter) 结构体 */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} BiquadFilter_t;

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 滤波器句柄
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 */
void LPF_Init(LowPassFilter_t *filter, float cutoff_freq, float sample_freq);

/**
 * @brief 执行一阶低通滤波
 * @param filter 滤波器句柄
 * @param input 当前采样输入值
 * @return 滤波后的平滑输出
 */
float LPF_Update(LowPassFilter_t *filter, float input);

/**
 * @brief 初始化双二阶陷波滤波器 (用于精准剔除电机特定转速振动)
 * @param filter 滤波器句柄
 * @param center_freq 陷波中心频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 * @param q_factor 品质因数 Q (通常取 2.0 ~ 4.0)
 */
void BiquadNotch_Init(BiquadFilter_t *filter, float center_freq, float sample_freq, float q_factor);

/**
 * @brief 执行双二阶滤波
 */
float Biquad_Update(BiquadFilter_t *filter, float input);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
