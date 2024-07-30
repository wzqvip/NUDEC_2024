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
int Motor_A, Motor_B, Target_A, Target_B; // 电机舵机控制相关
float Velocity = 15, Turn;
uint8_t CCD_Zhongzhi;
void Kinematic_Analysis(float velocity, float turn);
void APP_Show(void);
void CCD_Mode(void);
float Velocity_KP = 0.037, Velocity_KI = 0.007; // 速度控制PID参数

uint8_t SciBuf[200];
void slove_data(void);
char binToHex_low(uint8_t num);
char binToHex_high(uint8_t num);
void sendToPc(void);
int Turn_Flag_CNT;
int Turn_Flag;



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
	//	//使能串口中断
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(ADC_VOLTAGE_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	while (1)
	{
		Turn_Flag_CNT = 0;
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
		delay_us(50);

		for (i = 0; i < 128; i++) // 读取128个像素点电压值
		{
			DL_GPIO_clearPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=0;
			delay_us(15);										   // 调节曝光时间

			ADV[i] = (adc_getValue()) >> 4; // 右移4位是/4操作，将数据范围从0-4096压缩到0-256方便数据处理

			DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=1;

			if (ADV[i] < 200)
			{
				Turn_Flag_CNT += 1;
			}
			delay_us(20);
		}
		Find_CCD_Median();
		CCD_Mode(); // CCD巡线PID
		
		if(Turn_Flag_CNT > 5)
		{
			Turn_Flag = 1;
		}
		else
		{
			Turn_Flag = 0;
		}
		
		switch (Turn_Flag)
		{
		case 1:
			Set_PWM(500 + 110 * Turn, 500 - 110 * Turn);
			delay_ms(5);
			break;

		case 0:
			Set_PWM(500, 500);
			delay_ms(100);
			break;
		default:
			Set_PWM(500, 500);
			delay_ms(100);
			break;
		}

		 APP_Show();
		//         printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);
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
			LED_Flash(100);						// LED1闪烁
			Kinematic_Analysis(Velocity, Turn); // 小车运动学分析
			PWMA = Velocity_A(-Target_A, encoderA_cnt);
			PWMB = Velocity_B(-Target_B, encoderB_cnt);

			//			if(Turn_Flag)
			//			{
			//			Set_PWM(500 + 110 * Turn, 500 - 110 * Turn);
			//			}
			//			if(!Turn_Flag)
			//			//else
			//			{
			//			Set_PWM(500, 500);
			//			}
		}
	}
}

// ADC中断服务函数
void ADC_VOLTAGE_INST_IRQHandler(void)
{
	// 查询并清除ADC中断
	switch (DL_ADC12_getPendingInterrupt(ADC_VOLTAGE_INST))
	{
	// 检查是否完成数据采集
	case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
		gCheckADC = true; // 将标志位置1
		break;
	default:
		break;
	}
}

/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float velocity, float turn)
{

	Target_A = (velocity + turn);
	Target_B = (velocity - turn); // 后轮差速
}

void CCD_Mode(void)
{
	static float Bias, Last_Bias;
	Bias = CCD_Zhongzhi - 64;									  // 提取偏差
	Turn = Bias * Velocity_KP + (Bias - Last_Bias) * Velocity_KI; // PD控制
	Last_Bias = Bias;											  // 保存上一次的偏差
}

void APP_Show(void)
{
	static uint8_t flag;
	flag = !flag;
	if (PID_Send == 1) // 发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)(Velocity_KP * 1000), (int)(Velocity_KI * 1000), (int)Velocity, 0, 0, 0, 0, 0, 0); // 打印到APP上面
		PID_Send = 0;
	}
	else
	{
		// printf("{A%d:%d}$", encoderA_cnt, encoderB_cnt); // 打印到APP上面 显示波形
		printf("%d:%d:%d\r\n", CCD_Zhongzhi, Turn_Flag, Turn_Flag_CNT);
		// delay_ms(100);
		//sendToPc();
	}
}

/******************************************************************************
***
* FUNCTION NAME: void sendToPc(void) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将待发送的信息通过串口发送至上位机*
* MODIFY DATE : NONE *
* INPUT : void *
* OUTPUT : NONE *
* RETURN : NONE *
*******************************************************************************
**/
void sendToPc(void)
{
	int i;
	slove_data();
	printf("*");
	printf("LD");
	for (i = 2; i < 134; i++)
	{
		printf("%c", binToHex_high(SciBuf[i])); // 以字符形式发送高4位对应的16进制
		printf("%c", binToHex_low(SciBuf[i]));	// 以字符形式发送低?位对应的16进制
	}
	printf("00"); // 通信协议要求
	printf("#");  // 通信协议要求
}

void slove_data(void)
{
	int i;
	RD_TSL();
	SciBuf[0] = 0;
	SciBuf[1] = 132;
	SciBuf[2] = 0;
	SciBuf[3] = 0;
	SciBuf[4] = 0;
	SciBuf[5] = 0;
	for (i = 0; i < 128; i++)
		SciBuf[6 + i] = ADV[i];
}

/******************************************************************************
 ***
 * FUNCTION NAME: binToHex_low(u8 num) *
 * CREATE DATE : 20170707 *
 * CREATED BY : XJU *
 * FUNCTION : 将二进制低8位转换16进制*
 * MODIFY DATE : NONE *
 * INPUT : u8 *
 * OUTPUT : NONE *
 * RETURN : char *
 *******************************************************************************
 **/
char binToHex_low(uint8_t num)
{
	uint8_t low_four;
	low_four = num & 0x0f;
	if (low_four == 0)
		return ('0');
	else if (low_four == 1)
		return ('1');
	else if (low_four == 2)
		return ('2');
	else if (low_four == 3)
		return ('3');
	else if (low_four == 4)
		return ('4');
	else if (low_four == 5)
		return ('5');
	else if (low_four == 6)
		return ('6');
	else if (low_four == 7)
		return ('7');
	else if (low_four == 8)
		return ('8');
	else if (low_four == 9)
		return ('9');
	else if (low_four == 10)
		return ('A');
	else if (low_four == 11)
		return ('B');
	else if (low_four == 12)
		return ('C');
	else if (low_four == 13)
		return ('D');
	else if (low_four == 14)
		return ('E');
	else if (low_four == 15)
		return ('F');
}

/******************************************************************************
***
* FUNCTION NAME: binToHex_low(u8 num) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将二进制高8位转换16进制*
* MODIFY DATE : NONE *
* INPUT : u8 *
* OUTPUT : NONE *
* RETURN : char *
*******************************************************************************
**/
char binToHex_high(uint8_t num)
{
	uint8_t high_four;
	high_four = (num >> 4) & 0x0f;
	if (high_four == 0)
		return ('0');
	else if (high_four == 1)
		return ('1');
	else if (high_four == 2)
		return ('2');
	else if (high_four == 3)
		return ('3');
	else if (high_four == 4)
		return ('4');
	else if (high_four == 5)
		return ('5');
	else if (high_four == 6)
		return ('6');
	else if (high_four == 7)
		return ('7');
	else if (high_four == 8)
		return ('8');
	else if (high_four == 9)
		return ('9');
	else if (high_four == 10)
		return ('A');
	else if (high_four == 11)
		return ('B');
	else if (high_four == 12)
		return ('C');
	else if (high_four == 13)
		return ('D');
	else if (high_four == 14)
		return ('E');
	else if (high_four == 15)
		return ('F');
}
