#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include "at32f435_437.h"

//24kHz 采样频率与周期
#define BSP_ADC_SAMPLE_FREQ_HZ  24000.0f
#define BSP_ADC_SAMPLE_DT       (1.0f / BSP_ADC_SAMPLE_FREQ_HZ)

typedef struct {
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;
    uint16_t vbus;
    uint16_t temp1;
    uint16_t temp2;
} adc_raw_t;

void ADC_Init(void);
void ADC_Get_Data(adc_raw_t *data);
void user_adc_irq_handler(void);

void ADC_User_Callback(const adc_raw_t *data, float dt);
void FOC_User_Callback(float dt);

#endif

