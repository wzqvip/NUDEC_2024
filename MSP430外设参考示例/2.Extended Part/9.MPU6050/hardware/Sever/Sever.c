#include "Sever.h"


void Sever_Init(void)
{
    DL_Timer_startCounter(Sever_INST);
}

void Servo_Ctrl (int duty)//������۷�ΧΪ��0.5ms--2.5ms�������ʵ�ʱ������ΧС
{
//    if (duty >= Servo_Left_Max) duty = Servo_Left_Max;                   //���Ʒ�ֵ
//    else if (duty <= Servo_Right_Min) duty = Servo_Right_Min;            //���Ʒ�ֵ
    duty =   duty / 180 * 2000 + 500;
    DL_Timer_setCaptureCompareValue(Sever_INST, duty, DL_TIMER_CC_0_INDEX);	
  
}


