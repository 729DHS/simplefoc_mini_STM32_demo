/**
 * @file    as5600.c
 * @brief   AS5600 磁编码器驱动实现
 */

#include "as5600.h"

/* ======================== 内部辅助 ======================== */

/**
 * @brief 从指定寄存器读取 2 字节（高字节在前）
 */
static uint16_t AS5600_ReadReg16(I2C_HandleTypeDef *hi2c, uint8_t reg_high) {
    uint8_t buf[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)(AS5600_I2C_ADDR << 1),   /* HAL 需要左移1位的地址 */
        reg_high,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        2                                    /* 超时 2ms (100k I2C 正常<200us) */
    );
    if (status != HAL_OK) {
        return 0xFFFF;                       /* 错误标志 */
    }
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/* ======================== 公开 API ======================== */

uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c) {
    return AS5600_ReadReg16(hi2c, AS5600_REG_RAW_ANGLE_H);
}

float AS5600_ReadAngleRadians(I2C_HandleTypeDef *hi2c) {
    uint16_t raw = AS5600_ReadRawAngle(hi2c);
    if (raw == 0xFFFF) return -1.0f;
    return (float)raw * 6.283185307f / 4096.0f;
}

float AS5600_ReadAngleDegrees(I2C_HandleTypeDef *hi2c) {
    uint16_t raw = AS5600_ReadRawAngle(hi2c);
    if (raw == 0xFFFF) return -1.0f;
    return (float)raw * 360.0f / 4096.0f;
}

uint8_t AS5600_IsMagnetDetected(I2C_HandleTypeDef *hi2c) {
    uint8_t status;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)(AS5600_I2C_ADDR << 1),
        AS5600_REG_STATUS,
        I2C_MEMADD_SIZE_8BIT,
        &status,
        1,
        2
    );
    if (ret != HAL_OK) return 0;
    return (status >> 3) & 0x01;             /* MD 位 (bit 3) */
}
