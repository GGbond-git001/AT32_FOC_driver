#ifndef __FOC_APP_H
#define __FOC_APP_H

#include "bsp_adc.h"
#include "bsp_spi.h"
#include "bsp_tim.h"
#include "foc_sensor.h"
#include "foc_encoder.h"
#include "pid.h"
#include "svpwm.h"
#include "spmsm.h"
#include "wk_system.h"

extern svpwm_out_t FOC_Open;

typedef enum {
    FOC_SYS_IDLE = 0,       /* 空闲状态（无电流输出） */
    FOC_SYS_PARAM_ID,       /* 参数辨识中 */
    FOC_SYS_ERROR           /* 故障状态 */
} foc_sys_state_t;

void ALL_Init(void);

#endif 