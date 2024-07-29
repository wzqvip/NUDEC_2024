#include "..\HARDWARE\ADC\ADC.h"

volatile uint16_t gAdcResult[1];

void ADC_Init(void)
{
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
}


void ADC_start(void)
{
  DL_ADC12_startConversion(ADC12_0_INST);	//开始转换
}


void ADC12_0_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC12_0_INST)==DL_ADC12_IIDX_MEM0_RESULT_LOADED) //等待中断序列0
    {
        gAdcResult[0] = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
        DL_ADC12_enableConversions(ADC12_0_INST);
    }
}
















