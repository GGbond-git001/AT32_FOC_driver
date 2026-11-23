#include "foc_app.h"

svpwm_out_t FOC_Open = {0, PWM_HALF_PERIOD, PWM_HALF_PERIOD, PWM_HALF_PERIOD};

spmsm_id_cfg_t id_cfg = {
    2.0f,    /* d轴注入幅值 (V) */
    550.0f,  /* 注入频率 (Hz) */
    50,      /* 稳态等待周期 */
    80,      /* 积分采样周期 */
    8.0f,    /* 过流保护阈值 (A) */
    500.0f,  /* 电流环带宽 (Hz) */
    2.0f     /* 机械辨识q轴电流 (A) */
};

static volatile foc_sys_state_t foc_state = FOC_SYS_IDLE;

volatile float openloop_speed = 40.0f;  /* 开环电角速度 (rad/s) */
volatile float openloop_uq    = 1.0f;   /* 开环 Q 轴给定电压(V) */
static float openloop_theta   = 0.0f;

void ALL_Init(void)
{
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
    uint16_t raw_p90 = 0, raw_0_a = 0, raw_0_b = 0;
    const float align_v = 1.0f;

	//0->90
    svpwm_calc(FOC_ADC_Sensor.v_bus, FOC_PI * 0.5f, align_v, 0.0f, &FOC_Open);
    svpwm_set(&FOC_Open);
    wk_delay_ms(250);
    FOC_Encoder_Read_Avg_Raw(&raw_p90);

    //90->0
    svpwm_calc(FOC_ADC_Sensor.v_bus, 0.0f, align_v, 0.0f, &FOC_Open);
    svpwm_set(&FOC_Open);
    wk_delay_ms(300);
    FOC_Encoder_Read_Avg_Raw(&raw_0_a);

    //确定方向
    int32_t dir_check = (int32_t)raw_p90 - (int32_t)raw_0_a;
    if (dir_check > 8192)       dir_check -= 16384;
    else if (dir_check < -8192) dir_check += 16384;
    FOC_Encoder.dir = (dir_check >= 0) ? 1 : -1;

    //0->-90
    svpwm_calc(FOC_ADC_Sensor.v_bus, FOC_PI * 1.5f, align_v, 0.0f, &FOC_Open);
    svpwm_set(&FOC_Open);
    wk_delay_ms(250);
    //-90->0
    svpwm_calc(FOC_ADC_Sensor.v_bus, 0.0f, align_v, 0.0f, &FOC_Open);
    svpwm_set(&FOC_Open);
    wk_delay_ms(300);
    FOC_Encoder_Read_Avg_Raw(&raw_0_b);

    //校准
    FOC_Encoder_Set_Zero_From_Two_Raw(raw_0_a, raw_0_b);

    /* 校准完成,恢复 50% 零矢量 */
    BSP_TIM_SetZeroVector();
    /* ================================================================ */

    SPMSM_ID_Init(&SPMSM_ID, &id_cfg);
    SPMSM_ElecID_Start(&SPMSM_ID, BSP_ADC_SAMPLE_FREQ_HZ);
    foc_state = FOC_SYS_PARAM_ID;

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
    /* Clarke + Park 变换获取实时 dq 轴电流 */
    alphabeta_t i_ab;
    dq_t        i_dq;
    Angle_t     sc = FOC_Angle_Calc(FOC_Encoder.elec_angle);

    FOC_Clarke(&FOC_ADC_Sensor.i_abc, &i_ab);
    FOC_Park(&i_ab, &i_dq, &sc);

    switch (foc_state)
    {
        case FOC_SYS_PARAM_ID:
        {
            float ud = 0.0f;
            float uq = 0.0f;
            spmsm_id_state_t id_st = SPMSM_ElecID_Step(&SPMSM_ID, i_dq.d, i_dq.q, dt, &ud, &uq);

            if (id_st == SPMSM_ID_ELEC_SETTLE || id_st == SPMSM_ID_ELEC_SAMPLE)
            {
                /* 正在辨识：输出注入的 d 轴正弦电压 */
                svpwm_Translation_Method_calc(FOC_ADC_Sensor.v_bus, FOC_Encoder.elec_angle, ud, uq, &FOC_Open);
                svpwm_set(&FOC_Open);
            }
            else if (id_st == SPMSM_ID_ELEC_DONE)
            {
                /* 辨识成功：自动整定电流环 PI 参数，关断 PWM 主输出并退出到 IDLE */
                SPMSM_Apply_CurrentLoop_PI(&SPMSM_ID, FOC_ADC_Sensor.v_bus);
                BSP_TIM_SetZeroVector();
                foc_state = FOC_SYS_IDLE;
            }
            else
            {
                /* 过流或计算异常：立即关管保护并进入 ERROR */
                BSP_TIM_SetZeroVector();
                BSP_TIM_Disable();
                foc_state = FOC_SYS_ERROR;
            }
            break;
        }

        case FOC_SYS_IDLE:
        case FOC_SYS_ERROR:
        default:
            break;
    }
}
