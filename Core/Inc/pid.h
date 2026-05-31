/**
 * @file    pid.h
 * @brief   简单 PID 控制器
 */

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;               /* 比例系数            */
    float Ki;               /* 积分系数            */
    float Kd;               /* 微分系数            */
    float integral;         /* 积分累加值          */
    float prev_measurement; /* 上一次测量值 (D 用)   */
    float deriv_filtered;   /* 滤波后的微分值        */
    float integral_limit;   /* 积分限幅            */
    float output_limit;     /* 输出限幅 (±)        */
} PIDController;

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd,
              float integral_limit, float output_limit);

/**
 * @brief 计算 PID 输出
 * @param  pid         PID 句柄
 * @param  setpoint    目标值
 * @param  measurement 当前测量值
 * @param  dt          时间间隔 (秒)
 * @return PID 输出值
 */
float PID_Update(PIDController *pid, float setpoint, float measurement, float dt);

/**
 * @brief 重置 PID 状态（积分清零）
 */
void PID_Reset(PIDController *pid);

#endif /* PID_H */
