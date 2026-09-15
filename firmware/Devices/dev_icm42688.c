/**
 * @file dev_icm42688.c
 * @brief ICM-42688-P 寄存器配置与高速 SPI 读取驱动实现
 */

#include "dev_icm42688.h"

/* 寄存器地址映射 */
#define REG_DEVICE_CONFIG        0x11
#define REG_INT_CONFIG           0x14
#define REG_TEMP_DATA1           0x1D
#define REG_ACCEL_DATA_X1        0x1F
#define REG_GYRO_DATA_X1         0x25
#define REG_PWR_MGMT0            0x4E
#define REG_GYRO_CONFIG0         0x4F
#define REG_ACCEL_CONFIG0        0x50
#define REG_WHO_AM_I             0x75

#define DEG_TO_RAD               0.017453292519943295f

/* SPI 单寄存器写入 (最高位为 0 表示写入) */
static void write_reg(const ICM42688_Bus_t *bus, uint8_t reg, uint8_t value) {
    bus->cs_control(true);
    bus->spi_transfer(reg & 0x7F);
    bus->spi_transfer(value);
    bus->cs_control(false);
}

/* SPI 单寄存器读取 (最高位为 1 表示读取) */
static uint8_t read_reg(const ICM42688_Bus_t *bus, uint8_t reg) {
    bus->cs_control(true);
    bus->spi_transfer(reg | 0x80);
    uint8_t val = bus->spi_transfer(0x00);
    bus->cs_control(false);
    return val;
}

/* SPI 多字节突发读取 */
static void read_burst(const ICM42688_Bus_t *bus, uint8_t start_reg, uint8_t *buf, uint16_t len) {
    bus->cs_control(true);
    bus->spi_transfer(start_reg | 0x80);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = bus->spi_transfer(0x00);
    }
    bus->cs_control(false);
}

bool ICM42688_Init(ICM42688_t *dev, const ICM42688_Bus_t *bus) {
    if (dev == 0 || bus == 0) return false;

    dev->gyro_bias_dps[0] = 0.0f;
    dev->gyro_bias_dps[1] = 0.0f;
    dev->gyro_bias_dps[2] = 0.0f;
    dev->is_calibrated = false;

    bus->delay_ms(10);

    /* 1. 读取 WHO_AM_I 验证通信与芯片真伪 */
    uint8_t whoami = read_reg(bus, REG_WHO_AM_I);
    if (whoami != ICM42688_WHOAMI_VALUE) {
        return false;
    }

    /* 2. 软件复位芯片 */
    write_reg(bus, REG_DEVICE_CONFIG, 0x01);
    bus->delay_ms(10);

    /* 
     * 3. 配置陀螺仪量程与采样率 (GYRO_CONFIG0):
     *    量程: ±2000 dps (0x00 << 5)
     *    ODR: 1kHz (0x06)
     */
    write_reg(bus, REG_GYRO_CONFIG0, (0x00 << 5) | 0x06);

    /* 
     * 4. 配置加速度计量程与采样率 (ACCEL_CONFIG0):
     *    量程: ±16g (0x00 << 5)
     *    ODR: 1kHz (0x06)
     */
    write_reg(bus, REG_ACCEL_CONFIG0, (0x00 << 5) | 0x06);

    /* 
     * 5. 电源管理 (PWR_MGMT0):
     *    开启陀螺仪低噪声模式 (Low-Noise Mode: 0x0C)
     *    开启加速度计低噪声模式 (Low-Noise Mode: 0x03)
     */
    write_reg(bus, REG_PWR_MGMT0, 0x0F);
    bus->delay_ms(50);

    return true;
}

bool ICM42688_ReadSensors(ICM42688_t *dev, const ICM42688_Bus_t *bus) {
    if (dev == 0 || bus == 0) return false;

    /* 突发读取 14 字节: 温度(2B) + 加速度计(6B) + 陀螺仪(6B) */
    uint8_t raw_buf[14];
    read_burst(bus, REG_TEMP_DATA1, raw_buf, 14);

    int16_t raw_temp  = (int16_t)((raw_buf[0]  << 8) | raw_buf[1]);
    int16_t raw_acc_x = (int16_t)((raw_buf[2]  << 8) | raw_buf[3]);
    int16_t raw_acc_y = (int16_t)((raw_buf[4]  << 8) | raw_buf[5]);
    int16_t raw_acc_z = (int16_t)((raw_buf[6]  << 8) | raw_buf[7]);
    int16_t raw_gyr_x = (int16_t)((raw_buf[8]  << 8) | raw_buf[9]);
    int16_t raw_gyr_y = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
    int16_t raw_gyr_z = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

    /* 内部温度转换公式: T(℃) = (raw / 132.48) + 25 */
    dev->temperature_c = ((float)raw_temp / 132.48f) + 25.0f;

    /* ±16g 量程灵敏度: 2048 LSB/g */
    dev->accel_g[0] = (float)raw_acc_x / 2048.0f;
    dev->accel_g[1] = (float)raw_acc_y / 2048.0f;
    dev->accel_g[2] = (float)raw_acc_z / 2048.0f;

    /* ±2000 dps 量程灵敏度: 16.4 LSB/(deg/s) */
    dev->gyro_dps[0] = ((float)raw_gyr_x / 16.4f) - dev->gyro_bias_dps[0];
    dev->gyro_dps[1] = ((float)raw_gyr_y / 16.4f) - dev->gyro_bias_dps[1];
    dev->gyro_dps[2] = ((float)raw_gyr_z / 16.4f) - dev->gyro_bias_dps[2];

    /* 转换到 rad/s 供四元数微分方程计算 */
    dev->gyro_rad[0] = dev->gyro_dps[0] * DEG_TO_RAD;
    dev->gyro_rad[1] = dev->gyro_dps[1] * DEG_TO_RAD;
    dev->gyro_rad[2] = dev->gyro_dps[2] * DEG_TO_RAD;

    return true;
}

void ICM42688_CalibrateGyro(ICM42688_t *dev, const ICM42688_Bus_t *bus, uint16_t samples) {
    if (dev == 0 || bus == 0 || samples == 0) return;

    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint8_t raw_buf[6];

    for (uint16_t i = 0; i < samples; i++) {
        read_burst(bus, REG_GYRO_DATA_X1, raw_buf, 6);
        int16_t gx = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
        int16_t gy = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
        int16_t gz = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

        sum_gx += (float)gx / 16.4f;
        sum_gy += (float)gy / 16.4f;
        sum_gz += (float)gz / 16.4f;
        bus->delay_ms(2);
    }

    dev->gyro_bias_dps[0] = sum_gx / (float)samples;
    dev->gyro_bias_dps[1] = sum_gy / (float)samples;
    dev->gyro_bias_dps[2] = sum_gz / (float)samples;
    dev->is_calibrated = true;
}
