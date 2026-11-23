#include "bsp_adc.h"

static volatile uint16_t s_dma_buf[3] = {0};

void ADC_Init(void)
{
    /* 关闭 ADC2/3 重复中断，仅保留 ADC1 触发统一中断 */
    adc_interrupt_enable(ADC2, ADC_PCCE_INT, FALSE);
    adc_interrupt_enable(ADC3, ADC_PCCE_INT, FALSE);

    /* 启动 DMA1 通道 3 搬运普通组 */
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    DMA1_CHANNEL3->dtcnt = 3;
    DMA1_CHANNEL3->paddr = (uint32_t)&(ADC1->odt);
    DMA1_CHANNEL3->maddr = (uint32_t)s_dma_buf;
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    /* 软件触发启动 ADC1 普通组 */
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

void ADC_Get_Data(adc_raw_t *data)
{
    data->ia    = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
    data->ib    = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
    data->ic    = adc_preempt_conversion_data_get(ADC3, ADC_PREEMPT_CHANNEL_1);
    data->vbus  = s_dma_buf[0];
    data->temp1 = s_dma_buf[1];
    data->temp2 = s_dma_buf[2];
}

__attribute__((weak)) void ADC_User_Callback(const adc_raw_t *data, float dt)
{
    (void)data;
    (void)dt;
}

__attribute__((weak)) void FOC_User_Callback(float dt)
{
    (void)dt;
}

void user_adc_irq_handler(void)
{
    adc_raw_t raw_data;
    ADC_Get_Data(&raw_data);

    ADC_User_Callback(&raw_data, BSP_ADC_SAMPLE_DT);
    FOC_User_Callback(BSP_ADC_SAMPLE_DT);
}
