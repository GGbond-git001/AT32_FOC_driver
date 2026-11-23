#ifndef PID_H
#define PID_H

#include <math.h>
#include <stdio.h>
#include <stdbool.h> // 引入 bool 类型支持

// 内联函数限幅
static inline float f_limit(float input, float min, float max) {
    if (input > max) return max;
    if (input < min) return min;
    return input;
}

// PID 结构体精简优化
typedef struct
{
    // PID 参数
    float Kp;
    float Ki;
    float Kd;
    float N;              // 微分项低通滤波系数 (0.0 ~ 1.0, 1.0为不滤波)

    // 饱和限制
    float out_max;        // 输出限幅
    float i_max;          // 积分限幅

    // 历史状态
    float prev_error;     // 上一次误差 (梯形积分用)
    float prev_feedback;  // 上一次反馈值 (微分先行用)
    float integral;       // 积分项累加器
    float prev_d_out;     // 上一次微分输出 (低通滤波用)

    // 输出监控 
    float p_out;          // 比例项输出
    float i_out;          // 积分项输出
    float d_out;          // 微分项输出
    float out;            // 总输出

} PID_TypeDef;

float cyclic_clamp(float value, float min_val, float max_val);
void PID_Init(PID_TypeDef * pid, const float PID[3], float out_max, float i_max, float N);
float PID_Calc(PID_TypeDef * pid, float setpoint, float feedback, float dt);
void PID_Clear(PID_TypeDef * pid);

#endif
