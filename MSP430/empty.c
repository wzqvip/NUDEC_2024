/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "stdio.h"
#include "key.h"
#include "led.h"
#include "motor.h"
#include "CCD.h"
int32_t Get_Encoder_countA, encoderA_cnt, PWMA, Get_Encoder_countB, encoderB_cnt, PWMB;
uint8_t Key_Num = 0;
extern uint16_t ADV[128];
extern volatile bool gCheckADC;
extern uint8_t PID_Send;
int Motor_A, Motor_B, Target_A, Target_B; // �������������
float Velocity = 5, Turn;
uint8_t CCD_Zhongzhi;
void Kinematic_Analysis(float velocity, float turn);
void APP_Show(void);
void CCD_Mode(void);
float Velocity_KP = 0.037, Velocity_KI = 0.007; // �ٶȿ���PID����
int main(void)
{
	int i = 0;
	int tslp = 0;
	SYSCFG_DL_init();
	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	//	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	//	//ʹ�ܴ����ж�
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(ADC_VOLTAGE_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	while (1)
	{
		DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN);
		delay_us(5);										 // TSL_CLK=1;
		DL_GPIO_clearPins(GPIO_SI_PORT, GPIO_SI_PIN_25_PIN); // TSL_SI=0;
		delay_us(10);

		DL_GPIO_clearPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN);
		delay_us(5);									   // TSL_CLK=0;
		DL_GPIO_setPins(GPIO_SI_PORT, GPIO_SI_PIN_25_PIN); // TSL_SI=1;
		delay_us(30);

		DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN);
		delay_us(5);									   // TSL_CLK=1;
		DL_GPIO_setPins(GPIO_SI_PORT, GPIO_SI_PIN_25_PIN); // TSL_SI=1;
		delay_us(10);

		DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN);
		delay_us(5);										 // TSL_CLK=1;
		DL_GPIO_clearPins(GPIO_SI_PORT, GPIO_SI_PIN_25_PIN); // TSL_SI=0;
		delay_us(10);
		DL_GPIO_clearPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=0;
		delay_us(100);

		for (i = 0; i < 128; i++) // ��ȡ128�����ص��ѹֵ
		{
			DL_GPIO_clearPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=0;
			delay_us(100);										   // �����ع�ʱ��

			ADV[i] = (adc_getValue()) >> 4; // ����4λ��/4�����������ݷ�Χ��0-4096ѹ����0-256�������ݴ���

			DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=1;
			delay_us(20);
		}
		Find_CCD_Median();
		CCD_Mode(); // CCDѲ��PID
		APP_Show();
		// printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);
		delay_ms(5);
	}
}

void TIMER_0_INST_IRQHandler(void)
{
	if (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
	{
		if (DL_TIMER_IIDX_ZERO)
		{
			encoderA_cnt = Get_Encoder_countA;
			encoderB_cnt = Get_Encoder_countB;
			Get_Encoder_countA = 0;
			Get_Encoder_countB = 0;
			LED_Flash(100);						// LED1��˸
			Kinematic_Analysis(Velocity, Turn); // С���˶�ѧ����
			PWMA = Velocity_A(-Target_A, encoderA_cnt);
			PWMB = Velocity_B(-Target_B, encoderB_cnt);
			Set_PWM(PWMA, PWMB);
		}
	}
}

// ADC�жϷ�����
void ADC_VOLTAGE_INST_IRQHandler(void)
{
	// ��ѯ�����ADC�ж�
	switch (DL_ADC12_getPendingInterrupt(ADC_VOLTAGE_INST))
	{
	// ����Ƿ�������ݲɼ�
	case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
		gCheckADC = true; // ����־λ��1
		break;
	default:
		break;
	}
}

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float velocity, float turn)
{

	Target_A = (velocity + turn);
	Target_B = (velocity - turn); // ���ֲ���
}

void CCD_Mode(void)
{
	static float Bias, Last_Bias;
	Bias = CCD_Zhongzhi - 64;									  // ��ȡƫ��
	Turn = Bias * Velocity_KP + (Bias - Last_Bias) * Velocity_KI; // PD����
	Last_Bias = Bias;											  // ������һ�ε�ƫ��
}

void APP_Show(void)
{
	static uint8_t flag;
	flag = !flag;
	if (PID_Send == 1) // ����PID����
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_KP * 1000), (int)(Velocity_KI * 1000), (int)Velocity, 0, 0, 0, 0, 0, 0); // ��ӡ��APP����
		PID_Send = 0;
	}
	else
	{
		// printf("{A%%d:%d:%d:%d}$", CCD_Zhongzhi, encoderA_cnt, encoderB_cnt); // ��ӡ��APP���� ��ʾ����
			printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);

	}
}
