/**
 * @file    as5600.h
 * @brief   AS5600 磁编码器驱动 (软件 I2C)
 * @note    I2C 地址 0x36 (7-bit), 使用 SoftI2C 模块通信
 *          PB8=SCL, PB9=SDA — 开漏 + 外部上拉电阻
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

/* ======================== 寄存器定义 ======================== */

#define AS5600_I2C_ADDR         0x36    /* 7-bit 地址 */

#define AS5600_REG_ZMCO         0x00    /* 烧写次数 */
#define AS5600_REG_ZPOS_H       0x01    /* 起始位置 高8位 */
#define AS5600_REG_ZPOS_L       0x02    /* 起始位置 低8位 */
#define AS5600_REG_MPOS_H       0x03    /* 停止位置 高8位 */
#define AS5600_REG_MPOS_L       0x04    /* 停止位置 低8位 */
#define AS5600_REG_MANG_H       0x05    /* 最大角度 高8位 */
#define AS5600_REG_MANG_L       0x06    /* 最大角度 低8位 */
#define AS5600_REG_CONF_H       0x07    /* 配置 高8位 */
#define AS5600_REG_CONF_L       0x08    /* 配置 低8位 */
#define AS5600_REG_STATUS       0x0B    /* 状态寄存器 */
#define AS5600_REG_RAW_ANGLE_H  0x0C    /* 原始角度 高8位 */
#define AS5600_REG_RAW_ANGLE_L  0x0D    /* 原始角度 低8位 */
#define AS5600_REG_ANGLE_H      0x0E    /* 缩放角度 高8位 */
#define AS5600_REG_ANGLE_L      0x0F    /* 缩放角度 低8位 */
#define AS5600_REG_AGC          0x1A    /* 自动增益控制 */
#define AS5600_REG_MAGNITUDE_H  0x1B    /* 磁场强度 高8位 */
#define AS5600_REG_MAGNITUDE_L  0x1C    /* 磁场强度 低8位 */

/* ======================== API 函数 ======================== */

/**
 * @brief  读取原始角度 (12-bit, 0 ~ 4095)
 * @return 原始角度值，失败返回 0xFFFF
 */
uint16_t AS5600_ReadRawAngle(void);

/**
 * @brief  读取角度（弧度制，0 ~ 2π）
 * @return 角度 (rad)，失败返回 -1.0f
 */
float AS5600_ReadAngleRadians(void);

/**
 * @brief  读取角度（角度制，0 ~ 360°）
 * @return 角度 (deg)，失败返回 -1.0f
 */
float AS5600_ReadAngleDegrees(void);

/**
 * @brief  检测磁铁是否存在
 * @return 1 = 磁铁检测到, 0 = 未检测到或通信失败
 */
uint8_t AS5600_IsMagnetDetected(void);

#endif /* AS5600_H */
