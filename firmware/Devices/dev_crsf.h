/**
 * @file dev_crsf.h
 * @brief CRSF (Crossfire / ELRS) 遥控协议解析器
 */

#ifndef DEV_CRSF_H
#define DEV_CRSF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_SYNC_BYTE           0xC8
#define CRSF_FRAMETYPE_RC        0x16
#define CRSF_FRAME_MAX_SIZE      64

typedef struct {
    uint16_t channels[16];   /**< 16 通道标准舵量 (归一化为 1000 ~ 2000 μs) */
    uint32_t last_rx_time_ms;/**< 上一次成功接收数据包的时间戳 (毫秒) */
    bool is_connected;       /**< 遥控链路是否在线 */
} CRSF_Receiver_t;

/**
 * @brief 初始化 CRSF 接收机结构
 */
void CRSF_Init(CRSF_Receiver_t *rx);

/**
 * @brief 逐字节喂入串口接收到的原始数据包
 * @param rx 接收机句柄
 * @param byte 串口收到的单个字节
 * @param current_time_ms 当前系统时间戳 (毫秒)
 * @return true 表示成功完整解析出一帧有效遥控数据
 */
bool CRSF_ProcessByte(CRSF_Receiver_t *rx, uint8_t byte, uint32_t current_time_ms);

/**
 * @brief 周期性检测遥控是否超时丢失信号
 */
void CRSF_CheckTimeout(CRSF_Receiver_t *rx, uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* DEV_CRSF_H */
