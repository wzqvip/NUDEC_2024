#include "Encoder.h"

int16_t Encoder_Count;

void Encoder_Init(void)
{
    NVIC_EnableIRQ(Encoder_INT_IRQN);
}

int16_t Encoder_Get(void)
{
	int16_t Temp;
	Temp = Encoder_Count;
	Encoder_Count = 0;
	return Temp;
}
//���жϺ����밴���жϳ�ͻ   ����һ������
void GROUP1_IRQHandler(void)
{
    uint8_t Encodeg_flag,A_flag,B_flag,direction_flag;
	switch(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) 
	{
		case Encoder_INT_IIDX:
		if(DL_GPIO_getEnabledInterruptStatus(GPIOA, Encoder_A_PIN))
		{
            Encodeg_flag = 1;//A�ж�
            if(DL_GPIO_readPins(GPIOA, Encoder_A_PIN)) A_flag = 1;
            else A_flag = 0;
            if(DL_GPIO_readPins(GPIOA, Encoder_B_PIN)) B_flag = 1;
            else B_flag = 0;
            direction_flag = A_flag+B_flag+Encodeg_flag;//����ж�ת������ż����ת��������ת
            if(direction_flag == 0 || direction_flag ==2)Encoder_Count++;
            else Encoder_Count--;
            
			DL_GPIO_clearInterruptStatus(GPIOA, Encoder_A_PIN);
		}
		
		if(DL_GPIO_getEnabledInterruptStatus(GPIOA, Encoder_B_PIN))
		{
            Encodeg_flag = 0;
            if(DL_GPIO_readPins(GPIOA, Encoder_A_PIN)) A_flag = 1;
            else A_flag = 0;
            if(DL_GPIO_readPins(GPIOA, Encoder_B_PIN)) B_flag = 1;
            else B_flag = 0;
            direction_flag = A_flag+B_flag+Encodeg_flag;
            if(direction_flag == 0 || direction_flag ==2)Encoder_Count++;
            else Encoder_Count--;
            
			DL_GPIO_clearInterruptStatus(GPIOA, Encoder_B_PIN);
		}
 
		break;
	}
}
