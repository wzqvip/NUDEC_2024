#include "timer.h"

void Timer_Init(void)
{
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	DL_TimerG_startCounter(TIMER_0_INST);
}


void TIMER_0_INST_IRQHandler(void)
{
    if(DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
      LED_togglePins;
      Key_Scan();
    }
}









