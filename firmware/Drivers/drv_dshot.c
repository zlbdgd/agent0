/**
 * @file drv_dshot.c
 * @brief DShot 数据帧组装与 CRC4 计算算法实现
 */

#include "drv_dshot.h"

uint16_t DShot_PreparePacket(uint16_t throttle, bool telemetry_req) {
    /* 1. 油门限幅在 11 位内 (0 ~ 2047) */
    if (throttle > 2047) {
        throttle = 2047;
    }

    /* 2. 拼接油门 (11-bit) 与遥测请求标志 (1-bit) */
    uint16_t packet = (throttle << 1) | (telemetry_req ? 1 : 0);

    /* 3. 计算 4 位 CRC 校验码: (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F */
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0F;

    /* 4. 组合成最终 16 位传输帧 */
    return (packet << 4) | csum;
}

void DShot_EncodeDMABuffer(uint16_t packet, uint32_t *dma_buffer, uint32_t bit0_cvr, uint32_t bit1_cvr) {
    if (dma_buffer == 0) return;

    /* 高位优先 (MSB First) 打包 16 位 */
    for (int i = 0; i < 16; i++) {
        dma_buffer[i] = (packet & 0x8000) ? bit1_cvr : bit0_cvr;
        packet <<= 1;
    }
}
