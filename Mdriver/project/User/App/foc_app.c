#include "foc_app.h"

svpwm_out_t FOC_Open = {0, PWM_HALF_PERIOD, PWM_HALF_PERIOD, PWM_HALF_PERIOD};

spmsm_id_cfg_t id_cfg = {
    2.0f,    /* d轴注入幅值 (V) */
    550.0f,  /* 注入频率 (Hz) */
    50,      /* 稳态等待周期 */
    80,      /* 积分采样周期 */
    8.0f,    /* 过流保护阈值 (A) */
    1000.0f,  /* 电流环带宽 (Hz) */
    0.5f     /* 机械辨识q轴电流 (A) */
};

static volatile foc_sys_state_t foc_state = FOC_SYS_IDLE;

//volatile float openloop_speed = 40.0f;  // 开环电角速度 (rad/s)
//volatile float openloop_uq    = 1.0f;   // 开环 Q 轴给定电压(V) 
//static float openloop_theta   = 0.0f;

volatile float target_id_a = 0.0f;   // d轴目标电流 (A)
volatile float target_iq_a = 0.2f;   // q轴目标电流 (A)

volatile float foc_id_real = 0.0f;   // 实时 d 轴反馈电流 (A)
volatile float foc_iq_real = 0.0f;   // 实时 q 轴反馈电流 (A)

static volatile bool foc_init_done = false;  

void ALL_Init(void)
{
	foc_init_done = false;
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, FALSE);

    BSP_TIM_Start();
    ADC_Init();
    BSP_SPI_Init();

    FOC_Encoder_Init(7, 1);

    FOC_ADC_Calibrate(500);

    adc_raw_t raw_init;
    for (uint8_t i = 0; i < 16; i++) {
        ADC_Get_Data(&raw_init);
        Voltage_Calc(&raw_init);
        wk_delay_ms(1);
    }

    //编码器零位双向标定与方向自检
    FOC_Encoder_Auto_Calibrate(FOC_ADC_Sensor.v_bus, 1.0f);

    /* ================================================================ */

    SPMSM_ID_Init(&SPMSM_ID, &id_cfg);
    SPMSM_ElecID_Start(&SPMSM_ID, BSP_ADC_SAMPLE_FREQ_HZ);
    foc_state = FOC_SYS_PARAM_ID;
	
	foc_init_done = true;

    /* 清除标志并开启 24kHz ADC 中断 */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
}

void ADC_User_Callback(const adc_raw_t *data, float dt)
{
    Current_Calc(data);
    Voltage_Calc(data);

    uint16_t raw_enc = BSP_MT6816_Read_Raw16();
    FOC_Encoder_Update(raw_enc, dt);
}

void FOC_User_Callback(float dt)
{
	if (!foc_init_done)
    {
        return;
    }
	
	//将三相静止电流转换为旋转 dq 轴电流
    alphabeta_t i_ab;
    dq_t        i_dq;
    Angle_t     sc = FOC_Angle_Calc(FOC_Encoder.elec_angle);

    FOC_Clarke(&FOC_ADC_Sensor.i_abc, &i_ab);
    FOC_Park(&i_ab, &i_dq, &sc);
	
	  foc_id_real = i_dq.d;
    foc_iq_real = i_dq.q;
	
    /* Clarke + Park 变换获取实时 dq 轴电流 */
    switch (foc_state)
    {
        case FOC_SYS_PARAM_ID:
        {
            float ud = 0.0f;
            float uq = 0.0f;

            //单步执行正弦电压注入与正交投影，计算定子电阻 Rs 与电感 Ls
            spmsm_id_state_t id_st = SPMSM_ElecID_Step(&SPMSM_ID, i_dq.d, i_dq.q, dt, &ud, &uq);

            if (id_st == SPMSM_ID_ELEC_SETTLE || id_st == SPMSM_ID_ELEC_SAMPLE)
            {
                // d 轴正弦激励电压
                svpwm_Translation_Method_calc(FOC_ADC_Sensor.v_bus, FOC_Encoder.elec_angle, ud, uq, &FOC_Open);
                svpwm_set(&FOC_Open);
            }
            else if (id_st == SPMSM_ID_ELEC_DONE)
            {
                // 整定 dq 轴电流环 Kp 与 Ki
                if (SPMSM_Apply_CurrentLoop_PI(&SPMSM_ID, FOC_ADC_Sensor.v_bus)) 
				{
                    PID_Clear(&SPMSM_ID.pid_id);
                    PID_Clear(&SPMSM_ID.pid_iq);
                    foc_state = FOC_SYS_CURR_LOOP;
                } 
				else 
				{
                    BSP_TIM_SetZeroVector();
                    foc_state = FOC_SYS_ERROR;
                }
            }
            else
            {
                // 辨识异常或触发过流
                BSP_TIM_SetZeroVector();
                BSP_TIM_Disable();
                foc_state = FOC_SYS_ERROR;
            }
            break;
        }

        case FOC_SYS_CURR_LOOP:
        {
            // 过流保护检测
            if (fabsf(i_dq.d) > id_cfg.i_limit_a || fabsf(i_dq.q) > id_cfg.i_limit_a) 
			{
                BSP_TIM_SetZeroVector();
                BSP_TIM_Disable();
                foc_state = FOC_SYS_ERROR;
                break;
            }

            // 限制电流范围
            float id_ref = f_limit(target_id_a, -5.0f, 5.0f);
            float iq_ref = f_limit(target_iq_a, -5.0f, 5.0f);

            //计算目标电压 ud、uq
            float ud_pi = PID_Calc(&SPMSM_ID.pid_id, id_ref, i_dq.d, dt);
            float uq_pi = PID_Calc(&SPMSM_ID.pid_iq, iq_ref, i_dq.q, dt);

            // 高速交叉耦合前馈解耦
            float we = FOC_Encoder.elec_speed;
            float Ls = SPMSM_ID.param.Ls;
            float ud = ud_pi - we * Ls * i_dq.q;
            float uq = uq_pi + we * Ls * i_dq.d;

            //高速数字控制 1.5 拍角度延迟补偿
            float theta_comp = FOC_Normalize_Angle(FOC_Encoder.elec_angle + we * 1.5f * dt);

            //生成三相 PWM 占空比驱动电机
            svpwm_Translation_Method_calc(FOC_ADC_Sensor.v_bus, FOC_Encoder.elec_angle, ud, uq, &FOC_Open);
            svpwm_set(&FOC_Open);
            break;
        }

        case FOC_SYS_IDLE:
        case FOC_SYS_ERROR:
        default:
            break;
    }
//----------------------d周强拖---------------------
//      svpwm_Translation_Method_calc(FOC_ADC_Sensor.v_bus, 0, 1.0f, 0.0f, &FOC_Open);
//      svpwm_set(&FOC_Open);

}


