/**
 * @file    as5600.c
 * @brief   AS5600 磁编码器驱动实现 (使用软件 I2C)
 * @note    底层通信由 soft_i2c.{c,h} 提供, 替代有 bug 的 STM32F1 硬件 I2C
 */

#include "as5600.h"
#include "soft_i2c.h"

/* ======================== 内部辅助 ======================== */

/**
 * @brief 从指定寄存器读取 2 字节（高字节在前）
 * @retval 16-bit 值, 失败返回 0xFFFF
 */
static uint16_t AS5600_ReadReg16(uint8_t reg_high) {
    uint8_t buf[2];
    if (!SoftI2C_ReadBytes(AS5600_I2C_ADDR, reg_high, buf, 2)) {
        return 0xFFFF;                       /* 通信失败 */
    }
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/* ======================== 公开 API ======================== */

uint16_t AS5600_ReadRawAngle(void) {
    return AS5600_ReadReg16(AS5600_REG_RAW_ANGLE_H);
}

float AS5600_ReadAngleRadians(void) {
    uint16_t raw = AS5600_ReadRawAngle();
    if (raw == 0xFFFF) return -1.0f;
    return (float)raw * 6.283185307f / 4096.0f;
}

float AS5600_ReadAngleDegrees(void) {
    uint16_t raw = AS5600_ReadRawAngle();
    if (raw == 0xFFFF) return -1.0f;
    return (float)raw * 360.0f / 4096.0f;
}

uint8_t AS5600_IsMagnetDetected(void) {
    uint8_t status[1];
    if (!SoftI2C_ReadBytes(AS5600_I2C_ADDR, AS5600_REG_STATUS, status, 1)) {
        return 0;                             /* 通信失败 → 视为未检测到 */
    }
    return (status[0] >> 3) & 0x01;           /* MD 位 (bit 3) */
}
