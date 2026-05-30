/**
 * @file    foc.h
 * @brief   FOC 电机控制模块
 * @note    支持两种模式:
 *          MODE_OPENLOOP_SPEED  — 开环速度模式 (三相正弦波, 速度推进电角度)
 *          MODE_POSITION_SERVO  — 位置伺服模式 (传感器换向 + PID 位置闭环)
 *
 *          硬件映射:
 *          IN1 = PA8 (TIM1_CH1), IN2 = PA9 (TIM1_CH2), IN3 = PA10 (TIM1_CH3)
 *          ENABLE = PA11
 */

#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"
#include "pid.h"

/* ======================== 控制模式 ======================== */

typedef enum {
    MODE_OPENLOOP_SPEED = 0,    /* 开环速度: 电角度 = 积分(目标转速) */
    MODE_POSITION_SERVO = 1     /* 位置伺服: 电角度 = 传感器 * 极对数 + 90° */
} ControlMode;

/* ======================== 电机控制器 ======================== */

typedef struct {
    /* ---- 控制模式 ---- */
    ControlMode mode;

    /* ---- 用户设定 ---- */
    float   voltage_limit;          /* 电压幅值 (0.0 ~ 1.0)             */
    float   target_speed_rpm;       /* 目标转速 (RPM, 开环模式)         */
    float   target_position;        /* 目标位置 (rad, 伺服模式)          */
    uint8_t pole_pairs;             /* 电机极对数                        */
    uint8_t enabled;                /* 使能标志                          */

    /* ---- 运行时状态 ---- */
    float   electrical_angle;       /* 电角度 (rad, 0 ~ 2π)             */
    float   mechanical_angle;       /* 机械角度 (rad, 编码器反馈)        */

    /* ---- PID 位置环 ---- */
    PIDController pid;              /* 位置 PID 控制器                   */

    /* ---- 控制参数 ---- */
    float   dt;                     /* 控制周期 (秒)                     */
    uint16_t pwm_period;            /* PWM 周期 (TIM1->ARR)              */
    float   pwm_half_period;        /* PWM 半周期 (预计算)               */
} MotorController;

/* ======================== API ======================== */

void Motor_Init(MotorController *motor, uint8_t pole_pairs,
                uint16_t pwm_period, float dt);

void Motor_UpdatePWM(MotorController *motor, TIM_HandleTypeDef *htim);

/* ---- 通用 ---- */
void Motor_SetVoltage(MotorController *motor, float voltage);
void Motor_Enable(MotorController *motor, uint8_t en);
void Motor_SetMechanicalAngle(MotorController *motor, float angle_rad);

/* ---- 开环速度模式 ---- */
void Motor_SetSpeed(MotorController *motor, float rpm);

/* ---- 位置伺服模式 ---- */
void Motor_SetMode(MotorController *motor, ControlMode mode);
void Motor_SetTargetPosition(MotorController *motor, float pos_rad);
void Motor_SetPID(MotorController *motor, float Kp, float Ki, float Kd);

#endif /* FOC_H */
