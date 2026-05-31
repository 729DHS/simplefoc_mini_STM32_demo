/**
 * @file    soft_i2c.c
 * @brief   软件模拟 I2C 主机实现 — GPIO 位操作 (PB8=SCL, PB9=SDA)
 * @note    约 100~150kHz, 可靠驱动 AS5600
 *
 *          STM32F103 硬件 I2C 有 BUSY 状态卡死 bug.
 *          GPIO 模拟完全避开硬件外设, 不受该 bug 影响。
 *          每次读 2 字节约 1~1.5ms @ 72MHz, 控制周期 9ms, 占用 < 15%.
 */

#include "soft_i2c.h"
#include "stm32f1xx_hal.h"

/* ======================== GPIO 宏 ======================== */

#define SCL_PORT    GPIOB
#define SCL_PIN     GPIO_PIN_8
#define SDA_PORT    GPIOB
#define SDA_PIN     GPIO_PIN_9

#define SCL_H()     (SCL_PORT->BSRR = SCL_PIN)             /* SCL = HIGH */
#define SCL_L()     (SCL_PORT->BRR  = SCL_PIN)             /* SCL = LOW  */
#define SDA_H()     (SDA_PORT->BSRR = SDA_PIN)             /* SDA = HIGH */
#define SDA_L()     (SDA_PORT->BRR  = SDA_PIN)             /* SDA = LOW  */
#define SDA_READ()  ((SDA_PORT->IDR & SDA_PIN) ? 1U : 0U)  /* 读 SDA 电平 */

/* ======================== 时序延迟 ======================== */

/**
 * @brief I2C 半周期延迟 (~3~5μs @ 72MHz → 100~150kHz)
 *
 *        AS5600 支持 Standard (100k) / Fast (400k) / Fast+ (1M) 模式,
 *        容差很大, 不需要精确的周期数。
 */
static void i2c_delay(void) {
    /* 约 30 次 NOP 循环 @ 72MHz: 每次 ~4 cycles → ~1.7μs
     * 配合函数调用和 GPIO 操作的开销, 实际半周期约 3~5μs, 可接受 */
    for (volatile uint32_t i = 0; i < 30; i++) {
        __NOP();
    }
}

/* ======================== 初始化 ======================== */

void SoftI2C_Init(void) {
    /* 使能 GPIOB 时钟 (可能已由 CubeMX 使能, 这里保险再开一次) */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = SCL_PIN | SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏输出 — I2C 标准要求 */
    gpio.Pull  = GPIO_NOPULL;              /* 靠外部上拉电阻 (I2C 总线必备) */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;     /* 快速翻转 */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 初始状态: SCL 和 SDA 都拉高 (总线空闲) */
    SCL_H();
    SDA_H();
    i2c_delay();
}

/* ======================== 总线基本操作 ======================== */

/**
 * @brief 产生 I2C 起始条件 (START)
 *        SDA 在 SCL 高电平时从高→低
 */
static void i2c_start(void) {
    SDA_H();
    SCL_H();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_L();
}

/**
 * @brief 产生 I2C 停止条件 (STOP)
 *        SDA 在 SCL 高电平时从低→高
 */
static void i2c_stop(void) {
    SDA_L();
    SCL_H();
    i2c_delay();
    SDA_H();
    i2c_delay();
}

/**
 * @brief 发送 1 字节, 返回从设备 ACK
 * @retval 0  ACK (成功)
 * @retval 1  NAK (失败)
 */
static uint8_t i2c_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        i2c_delay();
        SCL_H();                    /* 数据在 SCL 上升沿被采样 */
        i2c_delay();
        SCL_L();
        i2c_delay();
        byte <<= 1;
    }

    /* 释放 SDA → 读 ACK */
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    uint8_t ack = SDA_READ();      /* 0=ACK, 1=NAK */
    SCL_L();
    i2c_delay();

    return ack;
}

/**
 * @brief 读取 1 字节, 发送 ACK/NAK
 * @param ack  0=发送 ACK, 1=发送 NAK (最后一字节)
 */
static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t byte = 0;

    SDA_H();                        /* 释放 SDA, 从设备驱动 */
    for (uint8_t i = 0; i < 8; i++) {
        i2c_delay();
        SCL_H();
        i2c_delay();
        byte <<= 1;
        if (SDA_READ()) {
            byte |= 0x01;
        }
        SCL_L();
    }

    /* 发送 ACK/NAK */
    if (ack) {
        SDA_H();                    /* NAK: 不拉低 SDA */
    } else {
        SDA_L();                    /* ACK: 拉低 */
    }
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    i2c_delay();
    SDA_H();                        /* 释放总线 */
    i2c_delay();

    return byte;
}

/* ======================== 公开 API ======================== */

/**
 * @brief 从设备寄存器读取多字节
 *
 *  I2C 序列: START → dev_addr(W) → reg → RESTART → dev_addr(R) → data[0..n] → STOP
 *  每次发完地址后检查 ACK, 无应答则返回失败。
 */
uint8_t SoftI2C_ReadBytes(uint8_t dev_addr, uint8_t reg,
                          uint8_t *data, uint8_t len) {
    if (!data || len == 0) return 0;

    /* ---- 写阶段: 发设备地址 + 寄存器地址 ---- */
    i2c_start();

    /* 发设备地址 (W) */
    if (i2c_write_byte((uint8_t)(dev_addr << 1))) {
        i2c_stop();
        return 0;                   /* 设备无应答 */
    }

    /* 发寄存器地址 */
    if (i2c_write_byte(reg)) {
        i2c_stop();
        return 0;
    }

    /* ---- 读阶段: RESTART → dev_addr(R) → 读数据 ---- */
    /* RESTART: 不产生 STOP, 直接 START */
    SDA_H();
    SCL_H();
    i2c_delay();
    SDA_L();                        /* SDA 在 SCL=H 时拉低 → START */
    i2c_delay();
    SCL_L();

    /* 发设备地址 (R) */
    if (i2c_write_byte((uint8_t)((dev_addr << 1) | 1))) {
        i2c_stop();
        return 0;
    }

    /* 读字节: 前 n-1 字节发 ACK, 最后发 NAK */
    for (uint8_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte((i == len - 1) ? 1 : 0);
    }

    i2c_stop();
    return 1;
}

/**
 * @brief 向设备寄存器写入 1 字节
 */
uint8_t SoftI2C_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    i2c_start();

    /* 发设备地址 (W) */
    if (i2c_write_byte((uint8_t)(dev_addr << 1))) {
        i2c_stop();
        return 0;
    }

    /* 发寄存器地址 */
    if (i2c_write_byte(reg)) {
        i2c_stop();
        return 0;
    }

    /* 发数据 */
    if (i2c_write_byte(data)) {
        i2c_stop();
        return 0;
    }

    i2c_stop();
    return 1;
}
