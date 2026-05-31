/**
 * @file    soft_i2c.h
 * @brief   软件模拟 I2C 主机 (PB8=SCL, PB9=SDA)
 * @note    替代 STM32F103 硬件 I2C, 避开外设 BUSY 卡死 bug
 *
 *          PB8/SCL, PB9/SDA — 与 CubeMX I2C1 引脚相同, 无需飞线。
 *          SoftI2C_Init() 会将两个引脚从 AF 模式切为 GPIO 开漏输出。
 */

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>

/**
 * @brief 初始化软件 I2C GPIO
 * @note  将 PB8(SCL) 和 PB9(SDA) 配置为开漏输出, 初始拉高
 *         调用时机: 在 MX_I2C1_Init() 之后 (覆盖其 AF 配置)
 */
void SoftI2C_Init(void);

/**
 * @brief 从 I2C 设备寄存器连续读取字节
 * @param  dev_addr  7-bit I2C 地址 (如 AS5600=0x36)
 * @param  reg       寄存器地址 (8-bit)
 * @param  data      读取数据缓冲区
 * @param  len       读取字节数
 * @retval 1  成功
 * @retval 0  失败 (从机无应答)
 */
uint8_t SoftI2C_ReadBytes(uint8_t dev_addr, uint8_t reg,
                          uint8_t *data, uint8_t len);

/**
 * @brief 向 I2C 设备寄存器写入单个字节
 * @param  dev_addr  7-bit I2C 地址
 * @param  reg       寄存器地址
 * @param  data      写入数据
 * @retval 1  成功
 * @retval 0  失败
 */
uint8_t SoftI2C_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t data);

#endif /* SOFT_I2C_H */
