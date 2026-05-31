/**
 * @file    pid.c
 * @brief   PID 控制器实现 (带条件积分防饱和)
 */

#include "pid.h"

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd,
              float integral_limit, float output_limit)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
}

/**
 * @brief 计算 PID 输出
 *
 *         采用条件积分 (conditional integration) 防饱和:
 *         - 当 P+I+D 未超过输出限幅时，正常累加积分
 *         - 当输出已饱和时，暂停积分累加 (防止 windup)
 *
 *         微分采用"测量值微分" (derivative on measurement) 避免
 *         setpoint 突变引起的微分冲击。
 */
float PID_Update(PIDController *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* === 比例项 === */
    float P_out = pid->Kp * error;

    /* === 积分项 (先不加 Ki*integral, 用于判断饱和) === */
    float I_contrib = pid->Ki * pid->integral;

    /* === 微分项 (基于误差变化率) === */
    float derivative = (dt > 1e-6f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;
    float D_out = pid->Kd * derivative;

    /* === 合成输出 === */
    float output = P_out + I_contrib + D_out;

    /* === 条件积分 (anti-windup) ===
     * 仅当输出在限幅范围内时才累加积分。
     * 如果输出超出限幅，积分冻结，防止一直往同方向累加。 */
    int saturated = 0;
    if (output > pid->output_limit) {
        output = pid->output_limit;
        saturated = 1;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        saturated = 1;
    }

    if (!saturated) {
        /* 正常: 累加积分 */
        pid->integral += error * dt;
    }
    /* 饱和: 不累加 (冻结), 积分限幅仍然生效 */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    return output;
}

void PID_Reset(PIDController *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
