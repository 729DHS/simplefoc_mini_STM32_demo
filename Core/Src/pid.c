/**
 * @file    pid.c
 * @brief   PID 控制器实现
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

float PID_Update(PIDController *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* 比例项 */
    float P = pid->Kp * error;

    /* 积分项 (带限幅) */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    /* 微分项 (带 dt 防除零) */
    float derivative = (dt > 1e-6f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;
    float D = pid->Kd * derivative;

    /* 合成并限幅输出 */
    float output = P + I + D;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID_Reset(PIDController *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
