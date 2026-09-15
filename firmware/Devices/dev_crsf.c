/**
 * @file dev_crsf.c
 * @brief CRSF (ELRS) 16通道遥控数据帧与 CRC8-D5 校验解析
 */

#include "dev_crsf.h"
#include "../include/config.h"
#include <string.h>

static uint8_t rx_buffer[CRSF_FRAME_MAX_SIZE];
static uint8_t rx_index = 0;
static uint8_t expected_len = 0;

/* CRSF 标准多项式 0xD5 的 CRC8 计算 */
static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

void CRSF_Init(CRSF_Receiver_t *rx) {
    if (rx == 0) return;
    for (int i = 0; i < 16; i++) {
        rx->channels[i] = RC_CHANNEL_MID;
    }
    rx->channels[2] = RC_CHANNEL_MIN; /* 油门初始化在最低端 */
    rx->last_rx_time_ms = 0;
    rx->is_connected = false;
    rx_index = 0;
}

/**
 * @brief 解析 CRSF 16 通道 11-Bit 打包格式 (22 字节载荷)
 */
static void decode_rc_channels(CRSF_Receiver_t *rx, const uint8_t *payload) {
    /* 
     * CRSF 原始通道为 11 位，数值范围 172 ~ 1811:
     * 172  -> 988 μs (相当于 RC 1000)
     * 992  -> 1500 μs
     * 1811 -> 2012 μs
     * 映射公式: pwm = (raw - 992) * 5 / 8 + 1500
     */
    uint16_t raw[16];
    raw[0]  = ((payload[0]       | (payload[1] << 8))                            & 0x07FF);
    raw[1]  = (((payload[1] >> 3)| (payload[2] << 5))                            & 0x07FF);
    raw[2]  = (((payload[2] >> 6)| (payload[3] << 2) | (payload[4] << 10))       & 0x07FF);
    raw[3]  = (((payload[4] >> 1)| (payload[5] << 7))                            & 0x07FF);
    raw[4]  = (((payload[5] >> 4)| (payload[6] << 4))                            & 0x07FF);
    raw[5]  = (((payload[6] >> 7)| (payload[7] << 1) | (payload[8] << 9))        & 0x07FF);
    raw[6]  = (((payload[8] >> 2)| (payload[9] << 6))                            & 0x07FF);
    raw[7]  = (((payload[9] >> 5)| (payload[10] << 3))                           & 0x07FF);
    raw[8]  = ((payload[11]      | (payload[12] << 8))                           & 0x07FF);
    raw[9]  = (((payload[12] >> 3)| (payload[13] << 5))                          & 0x07FF);
    raw[10] = (((payload[13] >> 6)| (payload[14] << 2) | (payload[15] << 10))    & 0x07FF);
    raw[11] = (((payload[15] >> 1)| (payload[16] << 7))                          & 0x07FF);
    raw[12] = (((payload[16] >> 4)| (payload[17] << 4))                          & 0x07FF);
    raw[13] = (((payload[17] >> 7)| (payload[18] << 1) | (payload[19] << 9))     & 0x07FF);
    raw[14] = (((payload[19] >> 2)| (payload[20] << 6))                          & 0x07FF);
    raw[15] = (((payload[20] >> 5)| (payload[21] << 3))                          & 0x07FF);

    for (int i = 0; i < 16; i++) {
        int32_t val = (int32_t)((raw[i] - 992) * 5 / 8 + 1500);
        if (val < 1000) val = 1000;
        if (val > 2000) val = 2000;
        rx->channels[i] = (uint16_t)val;
    }
}

bool CRSF_ProcessByte(CRSF_Receiver_t *rx, uint8_t byte, uint32_t current_time_ms) {
    if (rx_index == 0) {
        if (byte == CRSF_SYNC_BYTE) {
            rx_buffer[rx_index++] = byte;
        }
        return false;
    }

    if (rx_index == 1) {
        expected_len = byte;
        if (expected_len > (CRSF_FRAME_MAX_SIZE - 2) || expected_len < 3) {
            rx_index = 0; /* 异常长度，重置同步头 */
            return false;
        }
        rx_buffer[rx_index++] = byte;
        return false;
    }

    rx_buffer[rx_index++] = byte;

    /* 检查是否接收完一整帧 (同步头 1B + 长度 1B + 载荷 expected_len) */
    if (rx_index >= (expected_len + 2)) {
        uint8_t crc = crsf_crc8(&rx_buffer[2], expected_len - 1);
        uint8_t frame_crc = rx_buffer[rx_index - 1];

        rx_index = 0; /* 接收完成，重置缓冲指针 */

        if (crc == frame_crc) {
            uint8_t type = rx_buffer[2];
            if (type == CRSF_FRAMETYPE_RC) {
                decode_rc_channels(rx, &rx_buffer[3]);
                rx->last_rx_time_ms = current_time_ms;
                rx->is_connected = true;
                return true;
            }
        }
    }

    return false;
}

void CRSF_CheckTimeout(CRSF_Receiver_t *rx, uint32_t current_time_ms) {
    if (rx == 0) return;
    if (rx->is_connected && (current_time_ms - rx->last_rx_time_ms > CRSF_TIMEOUT_MS)) {
        rx->is_connected = false;
        rx->channels[2] = RC_CHANNEL_MIN; /* 触发丢包，油门强制归零 */
    }
}
