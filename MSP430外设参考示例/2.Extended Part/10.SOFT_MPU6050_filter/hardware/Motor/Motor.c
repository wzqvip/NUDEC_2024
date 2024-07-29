#include "Motor.h"

//#define  ANI1_A(x)     GPIO_WriteBit(GPIOB,GPIO_Pin_14,(BitAction)x);
//#define  ANI1_B(x)     GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)x);
//#define  BNI1_B(x)     GPIO_WriteBit(GPIOB,GPIO_Pin_13,(BitAction)x);
//#define  BNI1_A(x)     GPIO_WriteBit(GPIOB,GPIO_Pin_12,(BitAction)x);

void Motor_Init(void)
{
  DL_Timer_startCounter(Motor_INST);
}

void Motor0_Ctrl(int duty0,int duty1)
{
    duty0=duty0>3199?3199:(duty0<-3199?(-3199):duty0); 
    duty1=duty1>3199?3199:(duty1<-3199?(-3199):duty1); 
    
    if(duty0 >= 0)
    {
   
      DL_Timer_setCaptureCompareValue(Motor_INST, duty0, DL_TIMER_CC_0_INDEX);
      DL_Timer_setCaptureCompareValue(Motor_INST, duty1, DL_TIMER_CC_1_INDEX);
    }
    else    
    {
       DL_Timer_setCaptureCompareValue(Motor_INST, duty0, DL_TIMER_CC_0_INDEX);
       DL_Timer_setCaptureCompareValue(Motor_INST, duty1, DL_TIMER_CC_1_INDEX);
    }
}

void Motor1_Ctrl(int duty0,int duty1)
{
    duty0=duty0>3199?3199:(duty0<-3199?(-3199):duty0); 
    duty1=duty1>3199?3199:(duty1<-3199?(-3199):duty1); 
    
    if(duty0 >= 0)
    {
   
      DL_Timer_setCaptureCompareValue(Motor_INST, duty0, DL_TIMER_CC_2_INDEX);
      DL_Timer_setCaptureCompareValue(Motor_INST, duty1, DL_TIMER_CC_3_INDEX);
    }
    else    
    {
       DL_Timer_setCaptureCompareValue(Motor_INST, duty0, DL_TIMER_CC_2_INDEX);
       DL_Timer_setCaptureCompareValue(Motor_INST, duty1, DL_TIMER_CC_3_INDEX);
    }
}

