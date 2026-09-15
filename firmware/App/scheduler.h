/**
 * @file scheduler.h
 * @brief 飞控毫秒级非抢占式时间片轮询任务调度器
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*task_func_t)(uint32_t current_time_us);

typedef struct {
    task_func_t function;       /**< 任务回调函数 */
    uint32_t period_us;         /**< 运行周期 (微秒) */
    uint32_t last_executed_us;  /**< 上一次执行的时间戳 (微秒) */
} Task_t;

/**
 * @brief 初始化调度器
 */
void Scheduler_Init(void);

/**
 * @brief 在主循环中高频调用的轮询执行器
 * @param current_time_us 当前系统高精度定时器时间 (微秒)
 */
void Scheduler_Run(uint32_t current_time_us);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
