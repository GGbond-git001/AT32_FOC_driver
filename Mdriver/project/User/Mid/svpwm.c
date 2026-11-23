#include "svpwm.h"
#include "bsp_tim.h"

/**
 * @brief  SVPWM计算函数(传统办法计算)
 * @param  v_bus        直流母线电压 (V)
 * @param  target_theta 目标电角度 (rad)
 * @param  ud           D 轴目标电压 (V)
 * @param  uq           Q 轴目标电压 (V)
 * @param  out          输出结构体指针，存放算出的扇区与三相 CCR
 * @param  PWM_PERIOD   需要在.h文件中确认pwm计数值是否正确
 */
void svpwm_calc(float v_bus, float target_theta, float ud, float uq, svpwm_out_t *out)
{
    if (v_bus < 1.0f || out == NULL) {
        return;
    }
	//内切圆限幅：限制 sqrt(ud^2 + uq^2) <= v_bus / sqrt(3)
    float u_max_circle = v_bus * FOC_1_SQRT3;
    float u_sq = ud * ud + uq * uq;
    if (u_sq > u_max_circle * u_max_circle) 
	{
        float scale = u_max_circle / sqrtf(u_sq);
        ud *= scale;
        uq *= scale;
    }

    //算出角度的 sin 和 cos 
    Angle_t angle_sc = FOC_Angle_Calc(target_theta);

    //给定Ud, Uq，逆Park变换得到 U_alpha,U_beta 
    dq_t u_dq = { .d = ud, .q = uq };
    alphabeta_t u_ab;
    FOC_Inv_Park(&u_dq, &u_ab, &angle_sc);

    //N = 4C + 2B + A 
    float u1 =  u_ab.beta;
    float u2 =  FOC_SQRT3_2 * u_ab.alpha - 0.5f * u_ab.beta;
    float u3 = -FOC_SQRT3_2 * u_ab.alpha - 0.5f * u_ab.beta;

    uint8_t a = (u1 > 0.0f) ? 1 : 0;
    uint8_t b = (u2 > 0.0f) ? 1 : 0;
    uint8_t c = (u3 > 0.0f) ? 1 : 0;
    uint8_t n = (c << 2) | (b << 1) | a;

    //计算相邻两矢量作用时间
    float k_svpwm = (FOC_SQRT3 * PWM_PERIOD) / v_bus;
    float t_nx = 0.0f;
    float t_ny = 0.0f;

    switch (n) {
        case 3:  //  I 扇区  
            out->sector = 1;
            t_nx =  k_svpwm * u2;
            t_ny =  k_svpwm * u1;
            break;
        case 1:  //  II 扇区 
            out->sector = 2;
            t_nx = -k_svpwm * u2;
            t_ny = -k_svpwm * u3;
            break;
        case 5:  //  III 扇区 
            out->sector = 3;
            t_nx =  k_svpwm * u1;
            t_ny =  k_svpwm * u3;
            break;
        case 4:  //  IV 扇区 
            out->sector = 4;
            t_nx = -k_svpwm * u1;
            t_ny = -k_svpwm * u2;
            break;
        case 6:  //  V 扇区 
            out->sector = 5;
            t_nx =  k_svpwm * u3;
            t_ny =  k_svpwm * u2;
            break;
        case 2:  //  VI 扇区 
            out->sector = 6;
            t_nx = -k_svpwm * u3;
            t_ny = -k_svpwm * u1;
            break;
        default:
            out->sector = 0;
            t_nx = 0.0f;
            t_ny = 0.0f;
            break;
    }

    //过调制比例缩放保护
    float t_sum = t_nx + t_ny;
    if (t_sum > PWM_PERIOD) {
        float scale = PWM_PERIOD / t_sum;
        t_nx *= scale;
        t_ny *= scale;
    }

    //计算7段式对称切换时间并分配三相CCR
    float t_min = (PWM_PERIOD - t_nx - t_ny) * 0.5f;
    float t_mid = t_min + t_ny;
    float t_max = t_mid + t_nx;

    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    switch (n) {
        case 3:  ta = t_max; tb = t_mid; tc = t_min; break; //  I 扇区 
        case 1:  ta = t_mid; tb = t_max; tc = t_min; break; //  II 扇区 
        case 5:  ta = t_min; tb = t_max; tc = t_mid; break; //  III 扇区 
        case 4:  ta = t_min; tb = t_mid; tc = t_max; break; //  IV 扇区 
        case 6:  ta = t_mid; tb = t_min; tc = t_max; break; //  V 扇区 
        case 2:  ta = t_max; tb = t_min; tc = t_mid; break; //  VI 扇区
        default: ta = tb = tc = PWM_HALF_PERIOD;   break;
    }

    out->ccr_a = (uint32_t)ta;
    out->ccr_b = (uint32_t)tb;
    out->ccr_c = (uint32_t)tc;
}

/**
 * @brief  SVPWM计算函数(中点平移法)
 * @param  v_bus        直流母线电压 (V)
 * @param  target_theta 目标电角度 (rad)
 * @param  ud           D 轴目标电压 (V)
 * @param  uq           Q 轴目标电压 (V)
 * @param  out          输出结构体指针，存放算出的扇区与三相 CCR
 * @param  PWM_PERIOD   需要在.h文件中确认pwm计数值是否正确
 */
void svpwm_Translation_Method_calc(float v_bus, float target_theta, float ud, float uq, svpwm_out_t *out)
{
	if (v_bus < 1.0f || out == NULL) {
        return;
    }
	
	//内切圆限幅：限制 sqrt(ud^2 + uq^2) <= v_bus / sqrt(3)
    float u_max_circle = v_bus * FOC_1_SQRT3;
    float u_sq = ud * ud + uq * uq;
    if (u_sq > u_max_circle * u_max_circle) 
	{
        float scale = u_max_circle / sqrtf(u_sq);
        ud *= scale;
        uq *= scale;
    }

    //算出角度的 sin 和 cos 
    Angle_t angle_sc = FOC_Angle_Calc(target_theta);

    //给定Ud, Uq，逆Park变换得到 U_alpha,U_beta 
    dq_t u_dq = { .d = ud, .q = uq };
    alphabeta_t u_ab;
    FOC_Inv_Park(&u_dq, &u_ab, &angle_sc);
	
	// 逆 Park 得到 u_ab.alpha 和 u_ab.beta 后，直接计算三相相电压：
	float u_a = u_ab.alpha;
	float u_b = -0.5f * u_ab.alpha + FOC_SQRT3_2 * u_ab.beta;
	float u_c = -0.5f * u_ab.alpha - FOC_SQRT3_2 * u_ab.beta;

	// 求最大最小值（注入三次谐波/零序分量）
	float u_max = fmaxf(u_a, fmaxf(u_b, u_c));
	float u_min = fminf(u_a, fminf(u_b, u_c));
	float u_offset = 0.5f * (u_max + u_min);

	float u_span = u_max - u_min;
	float k = PWM_PERIOD / fmaxf(v_bus, u_span);

	// 归一化到 [0, PWM_PERIOD]
	float ta = PWM_HALF_PERIOD + (u_a - u_offset) * k;
	float tb = PWM_HALF_PERIOD + (u_b - u_offset) * k;
	float tc = PWM_HALF_PERIOD + (u_c - u_offset) * k;

	// 限幅保护防止溢出
	out->ccr_a = (uint32_t)fminf(fmaxf(ta, 0.0f), PWM_PERIOD);
	out->ccr_b = (uint32_t)fminf(fmaxf(tb, 0.0f), PWM_PERIOD);
	out->ccr_c = (uint32_t)fminf(fmaxf(tc, 0.0f), PWM_PERIOD);
}



void svpwm_set(const svpwm_out_t *out)
{
   BSP_PWM_Set(out);
}
