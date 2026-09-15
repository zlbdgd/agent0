/**
 * @file config.h
 * @brief 自研穿越机飞控全局系统参数与宏配置
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================== 调度周期与循环频率 ================== */
#define LOOP_RATE_HZ            1000U        /**< 主姿态控制与IMU采集速率: 1000Hz (1ms) */
#define LOOP_PERIOD_S           0.001f      /**< 姿态控制周期: 0.001s */

#define SCHEDULER_RATE_1000HZ   1U
#define SCHEDULER_RATE_500HZ    2U
#define SCHEDULER_RATE_100HZ    10U
#define SCHEDULER_RATE_10HZ     100U

/* ================== 姿态与控制限制 ================== */
#define MAX_TILT_ANGLE_DEG      55.0f       /**< 自稳模式下最大倾角限制 (度) */
#define MAX_ROLL_RATE_DPS       720.0f      /**< 滚转最大角速度限制 (度/秒) */
#define MAX_PITCH_RATE_DPS      720.0f      /**< 俯仰最大角速度限制 (度/秒) */
#define MAX_YAW_RATE_DPS        540.0f      /**< 偏航最大角速度限制 (度/秒) */

/* ================== DShot 电调参数 ================== */
#define DSHOT_MIN_THROTTLE      48U         /**< DShot 解锁最低怠速 */
#define DSHOT_MAX_THROTTLE      2047U       /**< DShot 满油门 */
#define DSHOT_DISARMED          0U          /**< 电机完全停转 */

/* ================== 遥控器与安全限制 ================== */
#define CRSF_TIMEOUT_MS         300U        /**< 丢失遥控信号 300ms 触发失控保护 */
#define RC_CHANNEL_MIN          1000U       /**< 遥控通道下限 (μs 等效值) */
#define RC_CHANNEL_MID          1500U       /**< 摇杆回中值 */
#define RC_CHANNEL_MAX          2000U       /**< 遥控通道上限 */
#define RC_ARM_THRESHOLD        1600U       /**< 解锁通道阈值 (AUX1 > 1600 为 ARM) */
#define RC_THROTTLE_MIN_CHECK   1050U       /**< 允许解锁的最低油门门槛 (油门必须拉到最低) */

/* ================== 电池监测与保护 ================== */
#define BATTERY_CELLS           2U          /**< 标称 2S 电池 (7.4V) */
#define BATTERY_WARN_CELL_VOLT  3.50f       /**< 单芯低压报警阈值: 3.50V (总电压 7.0V) */
#define BATTERY_CRIT_CELL_VOLT  3.30f       /**< 单芯严重过放保护: 3.30V (总电压 6.6V) */
#define BAT_VOLTAGE_DIVIDER     11.0f       /**< 10k:1k 分压系数 */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
