/**
 * @file drv_dshot.h
 * @brief DShot300 / DShot600 数字电调协议驱动
 */

#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_CMD_BEEP1          1
#define DSHOT_CMD_BEEP2          2
#define DSHOT_CMD_BEEP3          3
#define DSHOT_CMD_BEEP4          4
#define DSHOT_CMD_SPIN_DIR_1     7
#define DSHOT_CMD_SPIN_DIR_2     8
#define DSHOT_CMD_SAVE_SETTINGS  12

/**
 * @brief 构建 16 位 DShot 数字数据包 (包含 11位油门 + 1位遥测 + 4位CRC)
 * @param throttle 油门值 (0 或 48~2047)
 * @param telemetry_req 是否请求电调回传 RPM 遥测
 * @return 组包后的 16-Bit DShot 帧
 */
uint16_t DShot_PreparePacket(uint16_t throttle, bool telemetry_req);

/**
 * @brief 将 16 位数据帧转换为 DMA 定时器比较寄存器 (CCR) 占空比脉宽缓存
 * @param packet 16位 DShot 帧
 * @param dma_buffer 存放 16 个 PWM 占空比数值的数组
 * @param bit0_cvr 0 对应的 CCR 计数值 (约 37% 占空比)
 * @param bit1_cvr 1 对应的 CCR 计数值 (约 75% 占空比)
 */
void DShot_EncodeDMABuffer(uint16_t packet, uint32_t *dma_buffer, uint32_t bit0_cvr, uint32_t bit1_cvr);

#ifdef __cplusplus
}
#endif

#endif /* DRV_DSHOT_H */
