#include "foc_encoder.h"
#include "bsp_spi.h"
#include "bsp_tim.h"
#include "svpwm.h"
#include "wk_system.h"

foc_encoder_t FOC_Encoder = {
    .pole_pairs        = 7,
    .dir               = 1,
    .elec_angle_offset = 0.0f
};

/**
 * @brief  验证 MT6816 16位数据帧的偶校验位
 */
static bool mt6816_check_parity(uint16_t frame16)
{
    uint16_t val = frame16;
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return ((val & 0x01U) == 0U);
}

/**
 * @brief  初始化编码器参数并读取首帧有效角度
 */
void FOC_Encoder_Init(uint8_t pole_pairs, int8_t dir)
{
    FOC_Encoder.pole_pairs = pole_pairs;
    FOC_Encoder.dir        = (dir >= 0) ? 1 : -1;
    FOC_Encoder.err_cnt    = 0;

    uint16_t frame = BSP_MT6816_Read_Raw16();
    if (mt6816_check_parity(frame))
    {
        FOC_Encoder.raw_cnt  = frame >> 2;
        FOC_Encoder.raw_prev = FOC_Encoder.raw_cnt;
    }
    FOC_Encoder.mech_speed       = 0.0f;
    FOC_Encoder.elec_speed       = 0.0f;
    FOC_Encoder.total_mech_angle = 0.0f;
}

/**
 * @brief  24kHz 编码器数据校验、电角度解算与滤波测速
 * @param  dt  控制周期 (24kHz 下为 1.0f / 24000.0f)
 * @return true 校验通过并成功更新; false 校验失败(保持上一拍角度)
 */
bool FOC_Encoder_Update(uint16_t raw_frame16, float dt)
{
    /* 偶校验与弱磁诊断位解析 */
    FOC_Encoder.parity_ok   = mt6816_check_parity(raw_frame16);
    FOC_Encoder.no_mag_warn = ((raw_frame16 & 0x0002U) != 0U);

    if (!FOC_Encoder.parity_ok) {
        FOC_Encoder.err_cnt++;
        return false;
    }

    /* 提取高 14 位有效角度码值 (0 ~ 16383) */
    uint16_t raw = raw_frame16 >> 2;
    FOC_Encoder.raw_cnt = raw;

    /* 计算机械角度 [0, 2*PI) */
    float mech_rad = (float)raw * MT6816_RAW_TO_RAD;
    if (FOC_Encoder.dir < 0) {
        mech_rad = FOC_2PI - mech_rad;
    }
    FOC_Encoder.mech_angle = FOC_Normalize_Angle(mech_rad);

    /* 计算校准后的电角度 [0, 2*PI) */
    float raw_elec = FOC_Encoder.mech_angle * (float)FOC_Encoder.pole_pairs;
    FOC_Encoder.elec_angle = FOC_Normalize_Angle(raw_elec - FOC_Encoder.elec_angle_offset);

    /* 跨零点差分计算角速度并做一阶低通滤波 */
    int32_t delta_cnt = (int32_t)raw - (int32_t)FOC_Encoder.raw_prev;
    FOC_Encoder.raw_prev = raw;

    if (delta_cnt > 8192) {
        delta_cnt -= 16384;
    } else if (delta_cnt < -8192) {
        delta_cnt += 16384;
    }

    FOC_Encoder.total_mech_angle += (float)(delta_cnt * FOC_Encoder.dir) * MT6816_RAW_TO_RAD;

    float raw_speed = ((float)(delta_cnt * FOC_Encoder.dir) * MT6816_RAW_TO_RAD) / dt;
    FOC_Encoder.mech_speed = 0.925f * FOC_Encoder.mech_speed + 0.075f * raw_speed;
    FOC_Encoder.elec_speed = FOC_Encoder.mech_speed * (float)FOC_Encoder.pole_pairs;

    return true;
}

/**
 * @brief  连续采样 16 次并做防跨零点圆周平均，返回 14 位有效码值
 */
bool FOC_Encoder_Read_Avg_Raw(uint16_t *avg_raw_out)
{
    if (avg_raw_out == NULL) return false;

    int32_t  base_raw  = -1;
    int32_t  sum_diff  = 0;
    uint16_t valid_cnt = 0;

    for (uint8_t i = 0; i < 16; i++) {
        uint16_t frame = BSP_MT6816_Read_Raw16();
        if (mt6816_check_parity(frame) && ((frame & 0x0002U) == 0U)) {
            int32_t cur = (int32_t)(frame >> 2);
            if (base_raw < 0) {
                base_raw = cur;
            }

            /* 展开到 [-8192, +8192] 区间，彻底消除 0/16383 跨零点跳变 */
            int32_t diff = cur - base_raw;
            if (diff > 8192)       diff -= 16384;
            else if (diff < -8192) diff += 16384;

            sum_diff += diff;
            valid_cnt++;
        }
        wk_delay_ms(1);
    }

    if (valid_cnt < 8) {
        return false; /* 通信异常或未检测到磁铁 */
    }

    int32_t avg = base_raw + (sum_diff / (int32_t)valid_cnt);
    if (avg < 0)           avg += 16384;
    else if (avg >= 16384) avg -= 16384;

    *avg_raw_out = (uint16_t)avg;
    return true;
}

/**
 * @brief  根据正反两次吸合零位的编码器读数，计算圆周均值并锁定电角度零偏
 */
bool FOC_Encoder_Set_Zero_From_Two_Raw(uint16_t raw_cw, uint16_t raw_ccw)
{
    int32_t diff = (int32_t)raw_ccw - (int32_t)raw_cw;
    if (diff > 8192)       diff -= 16384;
    else if (diff < -8192) diff += 16384;

    int32_t avg = (int32_t)raw_cw + (diff / 2);
    if (avg < 0)           avg += 16384;
    else if (avg >= 16384) avg -= 16384;

    uint16_t avg_raw = (uint16_t)avg;
    FOC_Encoder.raw_cnt  = avg_raw;
    FOC_Encoder.raw_prev = avg_raw;

    float mech_rad = (float)avg_raw * MT6816_RAW_TO_RAD;
    if (FOC_Encoder.dir < 0) {
        mech_rad = FOC_2PI - mech_rad;
    }
    FOC_Encoder.mech_angle        = FOC_Normalize_Angle(mech_rad);
    FOC_Encoder.elec_angle_offset = FOC_Normalize_Angle(FOC_Encoder.mech_angle * (float)FOC_Encoder.pole_pairs);
    FOC_Encoder.total_mech_angle  = 0.0f;

    return true;
}

/**
 * @brief  单次吸合标定接口
 */
bool FOC_Encoder_Set_Zero_Offset(void)
{
    uint16_t avg_raw = 0;
    if (!FOC_Encoder_Read_Avg_Raw(&avg_raw)) {
        return false;
    }
    return FOC_Encoder_Set_Zero_From_Two_Raw(avg_raw, avg_raw);
}

/**
 * @brief  编码器零位双向自动标定与方向自检（阻塞执行，需在开启ADC中断前调用）
 * @param  v_bus    当前直流母线电压 (V)
 * @param  align_v  吸合对齐电压 (V)，建议 1.0f ~ 1.5f
 * @return true 标定成功; false 编码器通信异常或弱磁故障
 */
bool FOC_Encoder_Auto_Calibrate(float v_bus, float align_v)
{
    svpwm_out_t pwm_out = {0};
    uint16_t raw_p90 = 0, raw_0_a = 0, raw_0_b = 0;

    //0->90
    svpwm_calc(v_bus, FOC_PI * 0.5f, align_v, 0.0f, &pwm_out);
    svpwm_set(&pwm_out);
    wk_delay_ms(250);
    if (!FOC_Encoder_Read_Avg_Raw(&raw_p90)) {
        BSP_TIM_SetZeroVector();
        return false;
    }
    //90->0
    svpwm_calc(v_bus, 0.0f, align_v, 0.0f, &pwm_out);
    svpwm_set(&pwm_out);
    wk_delay_ms(300);
    if (!FOC_Encoder_Read_Avg_Raw(&raw_0_a)) {
        BSP_TIM_SetZeroVector();
        return false;
    }

    //确定方向
    int32_t dir_check = (int32_t)raw_p90 - (int32_t)raw_0_a;
    if (dir_check > 8192)       dir_check -= 16384;
    else if (dir_check < -8192) dir_check += 16384;
    FOC_Encoder.dir = (dir_check >= 0) ? 1 : -1;

    //0->-90
    svpwm_calc(v_bus, FOC_PI * 1.5f, align_v, 0.0f, &pwm_out);
    svpwm_set(&pwm_out);
    wk_delay_ms(250);

    //-90->0
    svpwm_calc(v_bus, 0.0f, align_v, 0.0f, &pwm_out);
    svpwm_set(&pwm_out);
    wk_delay_ms(300);
    if (!FOC_Encoder_Read_Avg_Raw(&raw_0_b)) {
        BSP_TIM_SetZeroVector();
        return false;
    }

   //校准
    bool ok = FOC_Encoder_Set_Zero_From_Two_Raw(raw_0_a, raw_0_b);

    /* 恢复 50% 零矢量 */
    BSP_TIM_SetZeroVector();

    return ok;
}

