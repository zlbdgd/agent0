/**
 * @file failsafe.h
 * @brief 飞控解锁安全检查与失控保护状态机
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>
#include <stdbool.h>
#include "../Devices/dev_crsf.h"
#include "../Middlewares/ahrs_mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLIGHT_STATE_DISARMED = 0,   /**< 上锁状态 (电机停转) */
    FLIGHT_STATE_ARMING_CHECK,   /**< 解锁前安全自检中 */
    FLIGHT_STATE_ARMED,          /**< 已解锁 (允许怠速与飞行) */
    FLIGHT_STATE_FAILSAFE        /**< 失控保护激活 (紧急迫降/切断动力) */
} FlightState_t;

typedef struct {
    FlightState_t state;
    bool allow_arm;              /**< 是否满足所有安全解锁条件 */
    uint32_t arm_blocked_reason; /**< 阻止解锁的原因掩码 (0 为无故障) */
} SafetyManager_t;

#define ARM_BLOCK_THROTTLE_NOT_LOW   (1 << 0)
#define ARM_BLOCK_RC_DISCONNECTED    (1 << 1)
#define ARM_BLOCK_IMU_NOT_CALIBRATED (1 << 2)
#define ARM_BLOCK_TILT_TOO_LARGE     (1 << 3)
#define ARM_BLOCK_BATTERY_CRITICAL   (1 << 4)

void Safety_Init(SafetyManager_t *safety);

/**
 * @brief 评估安全状态并处理解锁/上锁/失控转换
 */
void Safety_Update(SafetyManager_t *safety, 
                   const CRSF_Receiver_t *rx, 
                   const MahonyAHRS_t *ahrs, 
                   bool is_imu_calibrated,
                   float battery_voltage);

#ifdef __cplusplus
}
#endif

#endif /* FAILSAFE_H */
