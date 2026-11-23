#ifndef __SVPWM_H
#define __SVPWM_H

#include "foc_math.h"
#include "foc_const.h"
#include "at32f435_437.h"

#define PWM_PERIOD        6000.0f   //pwm计数值
#define PWM_HALF_PERIOD   3000.0f   //pwm半计数值

//SVPWM 计算输出结构体
typedef struct {
    uint8_t  sector;    /* 当前所在扇区*/
    uint32_t ccr_a;     /* A 相比较值 */
    uint32_t ccr_b;     /* B 相比较值 */
    uint32_t ccr_c;     /* C 相比较值 */
} svpwm_out_t;

void svpwm_calc(float v_bus, float target_theta, float ud, float uq, svpwm_out_t *out);
void svpwm_Translation_Method_calc(float v_bus, float target_theta, float ud, float uq, svpwm_out_t *out);

void svpwm_set(const svpwm_out_t *out);

#endif 