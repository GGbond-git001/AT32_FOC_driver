#ifndef __FOC_SENSOR_H
#define __FOC_SENSOR_H

#include <stdint.h>
#include "foc_math.h"
#include "bsp_adc.h"

//标定参数(Rs=5mR, 运放增益=50, 母线分压=10k/1k) 
#define ADC_VREF_VOLT       3.3f
#define ADC_FULL_SCALE      4095.0f
#define CURRENT_LSB_TO_AMP  ((ADC_VREF_VOLT / ADC_FULL_SCALE) / (0.005f * 50.0f))
#define VBUS_LSB_TO_VOLT    ((ADC_VREF_VOLT / ADC_FULL_SCALE) * 11.0f)

typedef struct {
    abc_t i_abc;        /* 三相相电流 (A) */
    float v_bus;        /* 母线电压 (V) */
    float offset_a;     /* A相静态零偏码值 */
    float offset_b;     /* B相静态零偏码值 */
    float offset_c;     /* C相静态零偏码值 */
} foc_sensor_t;

extern foc_sensor_t FOC_ADC_Sensor;

void FOC_ADC_Calibrate(uint16_t sample_count);

static inline void Current_Calc(const adc_raw_t *raw)
{
    FOC_ADC_Sensor.i_abc.a = -((float)raw->ia - FOC_ADC_Sensor.offset_a) * CURRENT_LSB_TO_AMP;
    FOC_ADC_Sensor.i_abc.b = -((float)raw->ib - FOC_ADC_Sensor.offset_b) * CURRENT_LSB_TO_AMP;
    FOC_ADC_Sensor.i_abc.c = -((float)raw->ic - FOC_ADC_Sensor.offset_c) * CURRENT_LSB_TO_AMP;
}

static inline void Voltage_Calc(const adc_raw_t *raw)
{
    float v_raw = (float)raw->vbus * VBUS_LSB_TO_VOLT;
    FOC_ADC_Sensor.v_bus = 0.9f * FOC_ADC_Sensor.v_bus + 0.1f * v_raw;
}

#endif 
