#include "pid.h"

//单精度循环限幅
float cyclic_clamp(float value, float min_val, float max_val) 
{
    float range_size = max_val - min_val;
    if (range_size <= 0.0f) 
    {
        return min_val; 
    }
    
    float offset = fmodf(value - min_val, range_size);
    
    if (offset < 0.0f) 
    {
        offset += range_size;
    }
    
    return min_val + offset;
}


//pid初始化（N为微分低通滤波系数）
void PID_Init(PID_TypeDef * pid, const float PID[3], float out_max, float i_max, float N)
{
    if (pid == NULL || PID == NULL) return;
    
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->N = f_limit(N, 0.0f, 1.0f);
    pid->out_max = out_max;
    pid->i_max = i_max;
    
    PID_Clear(pid);
}


//PID 状态清除 (失能电机时调用，防止重新使能时积分爆冲)
void PID_Clear(PID_TypeDef * pid)
{
    if (pid == NULL) return;
    
    pid->prev_error = 0.0f;
    pid->prev_feedback = 0.0f;
    pid->integral = 0.0f;
    pid->prev_d_out = 0.0f;
    
    pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0f;
}

/**
 * @brief          核心 PID 计算 (附带 dt，梯形积分，条件抗饱和，微分先行)
 * @param[in]      dt: 控制周期 (单位秒，如 1ms 传入 0.001f)
 */
float PID_Calc(PID_TypeDef * pid, float setpoint, float feedback, float dt)
{
    if (pid == NULL || dt <= 0.0f) return 0.0f;

    float error = setpoint - feedback;

    //P项
    pid->p_out = pid->Kp * error;

    //I项
    if(fabsf(pid->Ki) > 1e-6f) 
    {
        // 动态抗积分饱和: 
        // 如果输出已经达到最大限制，并且当前误差还在试图推动输出向同方向增加，则停止积分
        bool is_saturated = ((pid->out >= pid->out_max) && (error > 0.0f)) || 
                            ((pid->out <= -pid->out_max) && (error < 0.0f));
        
        if (!is_saturated) {
            // 梯形积分
            pid->integral += (error + pid->prev_error) * 0.5f * dt;
            // 静态限幅保障基本安全
            pid->integral = f_limit(pid->integral, -pid->i_max / pid->Ki, pid->i_max / pid->Ki);
        }
        pid->i_out = pid->Ki * pid->integral;
    } 
    else 
    {
        pid->i_out = 0.0f;
        pid->integral = 0.0f;
    }

    //D项
    // 微分先行
    float derivative = -(feedback - pid->prev_feedback) / dt;
    
    //计算原始微分输出
    float raw_d_out = pid->Kd * derivative;
    
    //D项低通滤波
    pid->d_out = pid->N * raw_d_out + (1.0f - pid->N) * pid->prev_d_out;

    //总输出
    pid->out = pid->p_out + pid->i_out + pid->d_out;
    pid->out = f_limit(pid->out, -pid->out_max, pid->out_max);

    //更新历史状态 
    pid->prev_error = error;
    pid->prev_feedback = feedback;
    pid->prev_d_out = pid->d_out;

    return pid->out;
}
