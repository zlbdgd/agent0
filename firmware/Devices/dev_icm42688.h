/**
 * @file dev_icm42688.h
 * @brief ICM-42688-P 高性能 6 轴 IMU 传感器驱动
 */

#ifndef DEV_ICM42688_H
#define DEV_ICM42688_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM42688_WHOAMI_VALUE     0x47

typedef struct {
    /* 物理量单位输出 */
    float gyro_dps[3];     /**< 陀螺仪角速度 X, Y, Z (度/秒, deg/s) */
    float gyro_rad[3];     /**< 陀螺仪角速度 X, Y, Z (弧度/秒, rad/s) */
    float accel_g[3];      /**< 加速度计 X, Y, Z (重力加速度单位, g) */
    float temperature_c;   /**< 芯片内部温度 (摄氏度) */

    /* 静态校准零偏 (Bias) */
    float gyro_bias_dps[3];
    bool is_calibrated;
} ICM42688_t;

/* SPI 底层读写函数指针类型 */
typedef uint8_t (*icm_spi_transfer_fn)(uint8_t data);
typedef void (*icm_cs_fn)(bool select);
typedef void (*icm_delay_ms_fn)(uint32_t ms);

typedef struct {
    icm_spi_transfer_fn spi_transfer;
    icm_cs_fn cs_control;
    icm_delay_ms_fn delay_ms;
} ICM42688_Bus_t;

/**
 * @brief 初始化 ICM-42688-P 传感器
 * @param dev 设备数据句柄
 * @param bus 硬件接口总线函数指针
 * @return true 初始化成功 (读到了正确的 0x47 WHO_AM_I)
 */
bool ICM42688_Init(ICM42688_t *dev, const ICM42688_Bus_t *bus);

/**
 * @brief 高速突发读取 6 轴原始数据并转换为标准单位
 */
bool ICM42688_ReadSensors(ICM42688_t *dev, const ICM42688_Bus_t *bus);

/**
 * @brief 静止状态下校准陀螺仪零偏 (采样 500 次取均值)
 */
void ICM42688_CalibrateGyro(ICM42688_t *dev, const ICM42688_Bus_t *bus, uint16_t samples);

#ifdef __cplusplus
}
#endif

#endif /* DEV_ICM42688_H */
