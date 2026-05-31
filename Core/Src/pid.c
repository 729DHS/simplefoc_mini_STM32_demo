/**
 * @file    pid.c
 * @brief   PID 控制器实现
 *
 *          - 比例:   Kp * error
 *          - 积分:   Ki * ∫error·dt, 条件积分防饱和
 *          - 微分:   Kd * d(measurement)/dt, 对测量值微分 + 一阶低通滤波
 *
 *          为什么要对 measurement 微分而不是 error?
 *          error = setpoint - measurement, 对它微分 = -d(measurement)/dt
 *          当 setpoint 突变 (如发 T90) 时, error 瞬间跳变 → D 项产生巨大尖峰
 *          → 电机剧烈抖动。对 measurement 微分则只反映电机实际速度,
 *          setpoint 变化不影响 D 项, 这才是纯阻尼。
 *
 *          低通滤波: 抑制 AS5600 的 1-2 LSB 量化噪声,
 *          截止频率 = 1/(2π·tau) ≈ 20Hz, 远高于 110Hz 控制频率的奈奎斯特。
 */

#include "pid.h"

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd,
              float integral_limit, float output_limit)
{
    pid->Kp               = Kp;
    pid->Ki               = Ki;
    pid->Kd               = Kd;
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->deriv_filtered   = 0.0f;
    pid->integral_limit   = integral_limit;
    pid->output_limit     = output_limit;
}

/**
 * @brief 计算 PID 输出
 *
 * @note  D term: -Kd * (filtered velocity of measurement)
 *        负号: 测量值增大时产生负向力矩 (阻尼)。
 *        一阶低通: filtered = filtered + α*(raw - filtered)
 *        α = dt / (dt + tau), tau ≈ 0.008s → 截止 ~20Hz
 */
float PID_Update(PIDController *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* === 比例项 === */
    float P_out = pid->Kp * error;

    /* === 积分项 === */
    float I_contrib = pid->Ki * pid->integral;

    /* === 微分项 (对测量值微分 + 低通滤波) ===
     * 测量值变化率 = 速度, 再乘低通滤波系数抑制噪声 */
    float D_out = 0.0f;
    if (dt > 1e-6f) {
        float velocity = (measurement - pid->prev_measurement) / dt;
        /* 一阶低通滤波: tau=0.008s, alpha = dt/(dt+tau) @ dt≈9ms → alpha≈0.53 */
        const float tau = 0.008f;
        float alpha = dt / (dt + tau);
        pid->deriv_filtered += alpha * (velocity - pid->deriv_filtered);
        /* 阻尼: 速度为正 → 输出负力矩 (刹车) */
        D_out = -pid->Kd * pid->deriv_filtered;
    }
    pid->prev_measurement = measurement;

    /* === 合成输出 === */
    float output = P_out + I_contrib + D_out;

    /* === 条件积分 (anti-windup) ===
     * 输出饱和时冻结积分, 防止 windup 导致棘轮手感 */
    int saturated = 0;
    if (output > pid->output_limit) {
        output = pid->output_limit;
        saturated = 1;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        saturated = 1;
    }

    if (!saturated) {
        pid->integral += error * dt;
    }
    /* 积分限幅始终生效 */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    return output;
}

void PID_Reset(PIDController *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->deriv_filtered   = 0.0f;
}
