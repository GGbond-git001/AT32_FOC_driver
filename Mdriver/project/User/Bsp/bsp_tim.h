#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include <stdint.h>
#include "at32f435_437.h"
#include "svpwm.h"

/* 严格对应 Work Bench 中 TMR1 的周期与半周期计数值 */
#define BSP_PWM_PERIOD      6000U
#define BSP_PWM_HALF_PERIOD 3000U

void BSP_TIM_Start(void);
void BSP_TIM_Disable(void);
void BSP_TIM_SetZeroVector(void);

/**
 * @brief  将 SVPWM 算出的三相 CCR 写入 TMR1 比较寄存器
 *         
 */
static inline void BSP_PWM_Set(const svpwm_out_t *out)
{
    if (out == NULL) {
        return;
    }
    TMR1->c1dt = out->ccr_a;  /* PA8  -> PWMA */
    TMR1->c2dt = out->ccr_b;  /* PA9  -> PWMB */
    TMR1->c3dt = out->ccr_c;  /* PA10 -> PWMC */
}

#endif 
