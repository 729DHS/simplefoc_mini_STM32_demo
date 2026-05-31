/**
 * @file    foc.c
 * @brief   FOC 电机控制实现 — 双模式
 *
 *          位置伺服模式原理:
 *          θ_elec = θ_mech * pole_pairs + π/2
 *          误差 = target - θ_mech (归一化到 [-π, π])
 *          力矩 = PID(误差)
 *          三相电压 = 力矩 * sin(θ_elec - phase*120°)
 *
 *          π/2 偏移让定子磁场超前转子磁场 90°，产生最大力矩。
 */

#include "foc.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

#define M_2PI     (2.0f * M_PI)
#define M_2PI_3   (2.094395102f)      /* 2π / 3 (120°) */

/* ======================== 初始化 ======================== */

void Motor_Init(MotorController *motor, uint8_t pole_pairs,
                uint16_t pwm_period, float dt)
{
    motor->mode              = MODE_POSITION_SERVO;  /* 默认位置伺服 */
    motor->voltage_limit     = 0.3f;
    motor->target_speed_rpm  = 0.0f;
    motor->target_position   = 0.0f;
    motor->pole_pairs        = pole_pairs;
    motor->enabled           = 0;
    motor->electrical_angle  = 0.0f;
    motor->mechanical_angle  = 0.0f;
    motor->dt                = dt;
    motor->pwm_period        = pwm_period;
    motor->pwm_half_period   = (float)pwm_period * 0.5f;

    /* 初始化位置 PID: Kp=0.1, Ki=0.02, Kd=0.01, 积分限幅±0.3, 输出限幅±1.0
     * 2804 12N14P 电机内阻低响应快, 用小增益起步。
     * D 项采用测量值微分+低通滤波。Kd 从 0.01 起步, 实测 0.03 会抖。 */
    PID_Init(&motor->pid, 0.1f, 0.02f, 0.01f, 0.3f, 1.0f);
}

/* ======================== PWM 更新 (核心) ======================== */

void Motor_UpdatePWM(MotorController *motor, TIM_HandleTypeDef *htim)
{
    if (!motor->enabled) {
        uint16_t neutral = motor->pwm_period >> 1;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, neutral);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, neutral);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, neutral);
        return;
    }

    float Va, Vb, Vc;

    if (motor->mode == MODE_OPENLOOP_SPEED) {
        /* ================================================
         * 开环速度模式: 电角度根据目标转速自由推进
         * ================================================ */
        float elec_speed = motor->target_speed_rpm * (M_2PI / 60.0f)
                           * (float)motor->pole_pairs;
        motor->electrical_angle += elec_speed * motor->dt;
        while (motor->electrical_angle >= M_2PI) motor->electrical_angle -= M_2PI;
        while (motor->electrical_angle < 0.0f)  motor->electrical_angle += M_2PI;

        float theta = motor->electrical_angle;
        float amp   = motor->voltage_limit;

        Va = amp * sinf(theta);
        Vb = amp * sinf(theta - M_2PI_3);
        Vc = amp * sinf(theta + M_2PI_3);

    } else {
        /* ================================================
         * 位置伺服模式: 传感器换向 + PID 位置闭环
         * θ_elec = θ_mech * P + π/2 (最大力矩对齐)
         * ================================================ */
        float theta_m = motor->mechanical_angle;
        float theta_e = theta_m * (float)motor->pole_pairs + M_PI_2;

        /* PID 计算力矩指令
         * 直接传目标位置和实测位置，内部 error = setpoint - measurement */
        float torque = PID_Update(&motor->pid, motor->target_position, theta_m, motor->dt);

        /* 力矩 = q 轴电压幅值 */
        float amp = torque;
        if (amp >  motor->voltage_limit) amp =  motor->voltage_limit;
        if (amp < -motor->voltage_limit) amp = -motor->voltage_limit;

        /* 三相正弦 (传感器对齐换向) */
        Va = amp * sinf(theta_e);
        Vb = amp * sinf(theta_e - M_2PI_3);
        Vc = amp * sinf(theta_e + M_2PI_3);

        /* 保存电角度供调试显示 */
        motor->electrical_angle = theta_e;
    }

    /* ---- 映射到 PWM 比较值 ---- */
    float hlf = motor->pwm_half_period;
    float max_val = (float)motor->pwm_period;

    float ccr1 = hlf * (1.0f + Va);
    float ccr2 = hlf * (1.0f + Vb);
    float ccr3 = hlf * (1.0f + Vc);

    if (ccr1 < 0.0f)  ccr1 = 0.0f;
    if (ccr1 > max_val) ccr1 = max_val;
    if (ccr2 < 0.0f)  ccr2 = 0.0f;
    if (ccr2 > max_val) ccr2 = max_val;
    if (ccr3 < 0.0f)  ccr3 = 0.0f;
    if (ccr3 > max_val) ccr3 = max_val;

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)ccr1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)ccr2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint16_t)ccr3);
}

/* ======================== API ======================== */

void Motor_SetVoltage(MotorController *motor, float voltage) {
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 1.0f) voltage = 1.0f;
    motor->voltage_limit = voltage;
}

void Motor_Enable(MotorController *motor, uint8_t en) {
    motor->enabled = en;
    if (en) {
        motor->electrical_angle = 0.0f;
        PID_Reset(&motor->pid);
        /* 记录当前位置为目标位置（防止上电跳变） */
        motor->target_position = motor->mechanical_angle;
    }
}

void Motor_SetMechanicalAngle(MotorController *motor, float angle_rad) {
    motor->mechanical_angle = angle_rad;
}

void Motor_SetSpeed(MotorController *motor, float rpm) {
    motor->target_speed_rpm = rpm;
}

void Motor_SetMode(MotorController *motor, ControlMode mode) {
    motor->mode = mode;
    PID_Reset(&motor->pid);
    motor->electrical_angle = 0.0f;
}

void Motor_SetTargetPosition(MotorController *motor, float pos_rad) {
    motor->target_position = pos_rad;
}

void Motor_SetPID(MotorController *motor, float Kp, float Ki, float Kd) {
    motor->pid.Kp = Kp;
    motor->pid.Ki = Ki;
    motor->pid.Kd = Kd;
    PID_Reset(&motor->pid);
}
