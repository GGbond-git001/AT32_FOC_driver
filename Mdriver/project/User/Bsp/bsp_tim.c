#include "bsp_tim.h"

/**
 * @brief  启动 TMR1 三相互补 PWM 输出与 24kHz TRGO 触发
 */
void BSP_TIM_Start(void)
{
    /* 初始置为 50% 零矢量，保证上电瞬间电机无电流冲击 */
    BSP_TIM_SetZeroVector();

    /* 三路正负互补通道全部使能 */
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);

    /* 开启高级定时器主输出 (MOE) 与计数器 */
    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}

/**
 * @brief  输出 50% 零矢量（用于开机 ADC 静态零偏校准，或需要刹车锁死电机时）
 */
void BSP_TIM_SetZeroVector(void)
{
    TMR1->c3dt = BSP_PWM_HALF_PERIOD;
    TMR1->c2dt = BSP_PWM_HALF_PERIOD;
    TMR1->c1dt = BSP_PWM_HALF_PERIOD;
}

/**
 * @brief  紧急关断三相 PWM 主输出（上下桥臂全关，高阻态自由滑行/过流保护）
 */
void BSP_TIM_Disable(void)
{
    tmr_output_enable(TMR1, FALSE);
}
