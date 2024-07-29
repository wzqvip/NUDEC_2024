#include "Sever.h"


void Sever_Init(void)
{
    DL_Timer_startCounter(Sever_INST);
}

void Servo_Ctrl (int duty)//舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
{
//    if (duty >= Servo_Left_Max) duty = Servo_Left_Max;                   //限制幅值
//    else if (duty <= Servo_Right_Min) duty = Servo_Right_Min;            //限制幅值
    duty =   duty / 180 * 2000 + 500;
    DL_Timer_setCaptureCompareValue(Sever_INST, duty, DL_TIMER_CC_0_INDEX);	
  
}


