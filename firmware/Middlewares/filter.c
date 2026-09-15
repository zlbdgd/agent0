/**
 * @file filter.c
 * @brief 数字低通滤波器与双二阶陷波滤波器算法实现
 */

#include "filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void LPF_Init(LowPassFilter_t *filter, float cutoff_freq, float sample_freq) {
    if (filter == 0 || sample_freq <= 0.0f) return;
    
    float rc = 1.0f / (2.0f * (float)M_PI * cutoff_freq);
    float dt = 1.0f / sample_freq;
    filter->alpha = dt / (rc + dt);
    filter->prev_out = 0.0f;
}

float LPF_Update(LowPassFilter_t *filter, float input) {
    if (filter == 0) return input;
    
    filter->prev_out = filter->prev_out + filter->alpha * (input - filter->prev_out);
    return filter->prev_out;
}

void BiquadNotch_Init(BiquadFilter_t *filter, float center_freq, float sample_freq, float q_factor) {
    if (filter == 0 || sample_freq <= 0.0f || q_factor <= 0.0f) return;

    float omega = 2.0f * (float)M_PI * center_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_factor);

    float a0 = 1.0f + alpha;
    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * cs) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = (-2.0f * cs) / a0;
    filter->a2 = (1.0f - alpha) / a0;

    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

float Biquad_Update(BiquadFilter_t *filter, float input) {
    if (filter == 0) return input;

    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                                      - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}
