#include "foc_sensor.h"

foc_sensor_t FOC_ADC_Sensor = {
    .offset_a = 2048.0f,
    .offset_b = 2048.0f,
    .offset_c = 2048.0f,
    .v_bus    = 12.0f
};

void FOC_ADC_Calibrate(uint16_t sample_count)
{
    uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
    adc_raw_t raw;

    if (sample_count == 0) return;

    for (uint16_t i = 0; i < sample_count; i++)
    {
        //等待采样完成 
        while (adc_flag_get(ADC1, ADC_PCCE_FLAG) == RESET);
        adc_flag_clear(ADC1, ADC_PCCE_FLAG);

        ADC_Get_Data(&raw);
        sum_a += raw.ia;
        sum_b += raw.ib;
        sum_c += raw.ic;
    }

    FOC_ADC_Sensor.offset_a = (float)sum_a / (float)sample_count;
    FOC_ADC_Sensor.offset_b = (float)sum_b / (float)sample_count;
    FOC_ADC_Sensor.offset_c = (float)sum_c / (float)sample_count;
}


