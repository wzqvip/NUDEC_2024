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
#include "stdlib.h"

void *__stack_chk_guard = (void *)0xdeadbeef;

void __stack_chk_fail(void)
{
	while(1)
    printf("===================Stack smashing detected.=============\n");
}

int track_num = 0;
int DEBUG = 0;
int DEBUG_PWM = 0;

// 6050
void Get_Angle(uint8_t way);
#define Pi 3.14159265
uint8_t Way_Angle = 2;						  // ��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
float Angle_Balance, Gyro_Balance, Gyro_Turn; // ƽ����� ƽ�������� ת��������
float Acceleration_Z;
int color = 1; // 1,2,3

float Total_Turns = 0;

int Initial_Turn = 0;	// ��ʼ�Ƕ�Ϊ0.
int Reverse_Turn = 180; // ���ؽǶ�
int Cross_Turn = 45;	// �Խ���
int Recross_Turn = 135; // ����

int32_t Get_Encoder_countA, encoderA_cnt, PWMA, Get_Encoder_countB, encoderB_cnt, PWMB;
uint8_t Key_Num = 0;
extern uint16_t ADV[128];
extern volatile bool gCheckADC;
extern uint8_t PID_Send;
int Motor_A, Motor_B, Target_A, Target_B; // �������������
float Velocity = 0, Turn = 0;
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

	MPU6050_initialize();
	DMP_Init();

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

	printf("Sys init done. Beep!\n");

	// ������һ��
	LED_Blink(0, 100);

	LED_ON(1);

	// ���������ѡ�������Ĵ��롣
	while (1)
	{
		// ��ⰴ��
		if (DL_GPIO_readPins(EXTENAL_KEY_PORT, EXTENAL_KEY_BUTTON_PIN) == 0)
		{
			delay_ms(50);
			int cnt = 0;
			while (DL_GPIO_readPins(EXTENAL_KEY_PORT, EXTENAL_KEY_BUTTON_PIN) == 0)
			{
				cnt++;
				delay_ms(100);
				if (cnt > 10) {
					LED_Blink(0, 100);
				}
			}
			if (cnt > 10) // ����ǳ���
			{
				LED_Blink(0, 200);
				break; // GO
				
			} else {
				LED_Blink(0, 50);
				track_num++;
				if (track_num > 3) {
					track_num = 0;
				}
				LED_Blink(track_num+1, 200);
			}
		}
	}
	LED_ON(track_num+1);
	Velocity = 15;

	while (1)
	{
				delay_ms(25);
		// printf("CCD\n");
		// �����CCD�Ĵ���
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
		// ����CCD

		// printf("AGL\n");
		// MPU6050��ȡ�ǶȵĴ���
		Get_Angle(2); // 6050
		// ����6050

		// APP_Show();
		//        printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);

		// printf("%f,%f,%f,%d\n",Pitch,Roll,Yaw,CCD_Zhongzhi);
		printf("%f,%d,%d,%d,%d,%d,%d,%d\n", Total_Turns, CCD_Zhongzhi, Target_A, encoderA_cnt, PWMA, Target_B, encoderB_cnt, PWMB);
	}
}

// �˶�ѧʹ������ط���ѭ����ʱ��������
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
			// LED_Flash(100, 2);					// LEDS��˸
			Kinematic_Analysis(Velocity, Turn); // С���˶�ѧ����
			PWMA = Velocity_A(-Target_A, -encoderA_cnt);
			PWMB = Velocity_B(-Target_B, encoderB_cnt);

			// printf("%d %d %d %d %d %d %d %d %d %d\n",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP,Velocity_KI,Velocity);
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
	float Bias, Last_Bias; // ����ԭ����static float!!
	printf("BIAS: %f, %f \n", Bias, Last_Bias);
	Bias = CCD_Zhongzhi - 64;									  // ��ȡƫ��
	Turn = Bias * Velocity_KP + (Bias - Last_Bias) * Velocity_KI; // PD����
	Total_Turns += Turn;
	Turn = 0;
	Last_Bias = Bias; // ������һ�ε�ƫ��
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
		printf("{A%%d:%d:%d:%d:%f:%f:%f}$", CCD_Zhongzhi, encoderA_cnt, encoderB_cnt, Pitch, Roll, Yaw); // ��ӡ��APP���� ��ʾ����
	}
}

/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(uint8_t way)
{
	float gyro_x, gyro_y, accel_x, accel_y, accel_z;
	float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
	if (way == 1) // DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{
		Read_DMP();				   // ��ȡ���ٶȡ����ٶȡ����
		Angle_Balance = Pitch;	   // ����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance = gyro[0];	   // ����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn = gyro[2];	   // ����ת����ٶ�
		Acceleration_Z = accel[2]; // ����Z����ٶȼ�
	}
	else
	{
		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);	// ��ȡX��������
		Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);	// ��ȡY��������
		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);	// ��ȡZ��������
		Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); // ��ȡX����ٶȼ�
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); // ��ȡX����ٶȼ�
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); // ��ȡZ����ٶȼ�
		if (Gyro_X > 32768)
			Gyro_X -= 65536; // ��������ת��  Ҳ��ͨ��shortǿ������ת��
		if (Gyro_Y > 32768)
			Gyro_Y -= 65536; // ��������ת��  Ҳ��ͨ��shortǿ������ת��
		if (Gyro_Z > 32768)
			Gyro_Z -= 65536; // ��������ת��
		if (Accel_X > 32768)
			Accel_X -= 65536; // ��������ת��
		if (Accel_Y > 32768)
			Accel_Y -= 65536; // ��������ת��
		if (Accel_Z > 32768)
			Accel_Z -= 65536;								// ��������ת��
		Gyro_Balance = -Gyro_X;								// ����ƽ����ٶ�
		Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / Pi; // ������ǣ�ת����λΪ��
		Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / Pi; // ������ǣ�ת����λΪ��
		accel_x = Accel_X / 1671.84;
		accel_y = Accel_Y / 1671.84;
		accel_z = Accel_Z / 1671.84;
		gyro_x = Gyro_X / 16.4; // ����������ת��
		gyro_y = Gyro_Y / 16.4; // ����������ת��
		if (Way_Angle == 2)
		{
			Pitch = -Kalman_Filter_x(Accel_Angle_x, gyro_x); // �������˲�
			Roll = -Kalman_Filter_y(Accel_Angle_y, gyro_y);
		}
		else if (Way_Angle == 3)
		{
			Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x); // �����˲�
			Roll = -Complementary_Filter_y(Accel_Angle_y, gyro_y);
		}
		Angle_Balance = Pitch;	  // ����ƽ�����
		Gyro_Turn = Gyro_Z;		  // ����ת����ٶ�
		Acceleration_Z = Accel_Z; // ����Z����ٶȼ�
	}
}
