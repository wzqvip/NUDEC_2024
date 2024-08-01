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
// #include "stdlib.h"

bool Turn_Flag = 0;
// bool last_state__ = 0;

int Total_A_CNT = 0;
int Total_B_CNT = 0;
int Delta_Target = 0;
int Total_turns = 0;

// int last_distance = 0;

// 6050
// void Get_Angle(uint8_t way);
// #define Pi 3.14159265
// uint8_t Way_Angle = 2;						  // 获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
// float Angle_Balance, Gyro_Balance, Gyro_Turn; // 平衡倾角 平衡陀螺仪 转向陀螺仪
// float Acceleration_Z;
// int color = 1; // 1,2,3

int32_t Get_Encoder_countA, encoderA_cnt, PWMA, Get_Encoder_countB, encoderB_cnt, PWMB;
uint8_t Key_Num = 0;
extern uint16_t ADV[128];
extern volatile bool gCheckADC;
extern uint8_t PID_Send;
int Motor_A, Motor_B, Target_A, Target_B; // 电机舵机控制相关
float Velocity = 0, Turn = 0;
uint8_t CCD_Zhongzhi;
void Kinematic_Analysis(float velocity, float turn);
void APP_Show(void);
void CCD_Mode(void);
float Velocity_KP = 0.1, Velocity_KI = 0.00; // 速度控制PID参数
// float calibrate_offset(float curr_velocity);
float offset = 0;

int main(void)
{
	int i = 0;
	int tslp = 0;
	static int last_state__ = 0;
	static int last_distance = 0;
	static int track_num = 0;
	int Diff_Delta = 1270;
	int min_distance = 4000;
	static int beep_counter = 0;
	SYSCFG_DL_init();

	// MPU6050_initialize();
	// DMP_Init();

	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	//	//使能串口中断
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	NVIC_EnableIRQ(ADC_VOLTAGE_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

	printf("Sys init done. Beep!\n");

	// 开机叫一声
	LED_Blink(0, 100);

	LED_ON(1);

	// 从这里插入选择赛道的代码。
	while (1)
	{
		// 检测按键
		if (DL_GPIO_readPins(EXTENAL_KEY_PORT, EXTENAL_KEY_BUTTON_PIN) == 0)
		{
			delay_ms(50);
			int cnt = 0;
			while (DL_GPIO_readPins(EXTENAL_KEY_PORT, EXTENAL_KEY_BUTTON_PIN) == 0)
			{
				cnt++;
				delay_ms(100);
				if (cnt > 10)
				{
					LED_Blink(0, 100);
				}
			}
			if (cnt > 10) // 如果是长按
			{
				LED_Blink(0, 200);
				break; // GO
			}
			else
			{
				// LED_Blink(0, 50);
				for (int k = 0; k < track_num + 1; k++)
				{
					LED_Blink(0, 50);
					delay_1ms(50);
				}
				track_num++;
				if (track_num > 3)
				{
					track_num = 0;
				}
				printf("Track_num: %d", track_num);
				LED_Blink(track_num + 1, 200);
			}
		}
	}
	// LED_ON(track_num + 1);
	Velocity = 15;
	last_distance = 0;

	while (1)
	{
		delay_ms(25);
		// printf("CCD\n");
		// 这段是CCD的代码
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

		for (i = 0; i < 128; i++) // 读取128个像素点电压值
		{
			DL_GPIO_clearPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=0;
			delay_us(100);										   // 调节曝光时间

			ADV[i] = (adc_getValue()) >> 4; // 右移4位是/4操作，将数据范围从0-4096压缩到0-256方便数据处理

			DL_GPIO_setPins(GPIO_CLK_PORT, GPIO_CLK_PIN_23_PIN); // TSL_CLK=1;
			delay_us(20);
		}
		Find_CCD_Median();

		// 赛道区别
		if (track_num == 0)
		{
			if (Turn_Flag == 1) // 第一次进入寻线模式就可以结束了。
			{

				Turn = 0;
				for (int k = 0; k < 10; k++)
				{
					Set_PWM(3000, 3000); // 保证到达A点
					LED_Blink(0, 100);
				}
				Velocity = 0;
				Set_PWM(0, 0);
				while (1)
					;
			}
		}

		if (track_num == 1)
		{
			// 0 - 3000~4000 直线

			if (ABS(Total_A_CNT) < 7000)
			{
				Total_turns = 0;
			}
			else if (ABS(Total_A_CNT) < 17000)
			{
				Total_turns = 1;
			}

			if (ABS(Total_A_CNT) > 3600 && beep_counter == 0)
			{
				LED_Blink(0, 3);
				beep_counter = 1;
			}
			if (ABS(Total_A_CNT) > 8800 && beep_counter == 1)
			{
				LED_Blink(0, 3);
				beep_counter = 2;
			}
			if (ABS(Total_A_CNT) > 12300 && beep_counter == 2)
			{
				LED_Blink(0, 3);
				beep_counter = 3;
			}

			// 结束

			if (Total_turns == 2 || ABS(Total_A_CNT) > 16500) // 转了两次弯之后（出寻线模式的时候会+1）
			{
				Turn = 0;
				Velocity = 0;
				Set_PWM(0, 0);
				LED_Blink(0, 100);
				while (1)
					;
			}
		}

		if (track_num == 2)
		{
			if (ABS(Total_A_CNT) < 5200 && ABS(Total_B_CNT) < 5200)
			{
				Turn_Flag = 0;
				Total_turns = 0;
			}
			Turn_Flag = 1;
		}

		if (Turn_Flag == 1)
		{ // 巡线模式
			if (last_state__ == 0)
			{
				last_state__ = 1;
				int travel_dist = -(Total_A_CNT - last_distance);
				if (travel_dist > min_distance)
				{ // 走距离足够了。
					// LED_Blink(0, 3);
					Total_turns++;
					last_distance = -(Total_A_CNT);
				}
				else
				{ // 如果不小心出巡线模式又回来
				}
			}
		}
		else
		{ // 直线模式。
			if (last_state__ == 1)
			{
				last_state__ = 0;
				// LED_Blink(0, 3);
			}
		}

		// CCD_Mode(); // CCD巡线PID

		if (Turn_Flag)
		{
			static float Bias, Last_Bias;								  // 这里原来是static float!!
			Bias = CCD_Zhongzhi - 59;									  // 提取偏差
			Turn = Bias * Velocity_KP + (Bias - Last_Bias) * Velocity_KI; // PD控制
			Last_Bias = Bias;											  // 保存上一次的偏差
																		  // printf("BIAS: %d, %d \n", (int)Bias, (int)Last_Bias);
		}
		else
		{
			if (track_num == 1)
			{
				Delta_Target = Total_turns * Diff_Delta;
				// A 是负的，走的多。
				if (Total_A_CNT + Total_B_CNT < -Delta_Target - 30)
				{
					Turn = -0.7;
				}
				else if (Total_B_CNT + Total_B_CNT > Delta_Target + 30)
				{
					Turn = 0.7;
				}
				else
				{
					Turn = 0;
				}
			}
			else if (track_num == 2)
			{
				if (ABS(Total_A_CNT) < 4696 && ABS(Total_B_CNT) < 4137)
				{
					Delta_Target = 560;
					if (Total_A_CNT + Total_B_CNT < -Delta_Target - 30)
					{
						Turn = -2;
					}
					else if (Total_B_CNT + Total_B_CNT > Delta_Target + 30)
					{
						Turn = 2;
					}
					else
					{
						Turn = 0;
					}
				}
				else
				{
					Delta_Target = 0;
					if (Total_A_CNT + Total_B_CNT < -Delta_Target - 30)
					{
						// Turn = -2;
						Turn =  Velocity;
					}
					else if (Total_B_CNT + Total_B_CNT > Delta_Target + 30)
					{
						// Turn = 2;
						Turn =  Velocity;
					}
					else
					{
						Turn = 0;
					}
					
				}
				// A 是负的，走的多。
				
			}
		}

		// 结束CCD

		// printf("AGL\n");
		// MPU6050获取角度的代码
		// Get_Angle(1); // 6050
		// 结束6050

		// APP_Show();
		//        printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);

		// printf("%f,%f,%f,%d\n",Pitch,Roll,Yaw,CCD_Zhongzhi);

		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", Turn_Flag, last_state__, Total_A_CNT, Total_B_CNT, Target_A, Target_B, PWMA, PWMB, Total_A_CNT + Total_B_CNT, Delta_Target, Total_turns);
		// printf("%d,%d, %d, %d, %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", Turn_Flag, last_state__, Total_A_CNT, Total_B_CNT, Total_turns, CCD_Zhongzhi, Target_A, encoderA_cnt, PWMA, Target_B, encoderB_cnt, PWMB, Total_A_CNT + Total_B_CNT, Delta_Target, -(Total_A_CNT)-last_distance);
	}
}

// 运动学【！！！！不要！！！】使用这个地方的循环定时器来处理。
void TIMER_0_INST_IRQHandler(void)
{
	if (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
	{
		if (DL_TIMER_IIDX_ZERO)
		{
			encoderA_cnt = Get_Encoder_countA;
			encoderB_cnt = Get_Encoder_countB;
			Total_A_CNT += encoderA_cnt;
			Total_B_CNT += encoderB_cnt;
			Get_Encoder_countA = 0;
			Get_Encoder_countB = 0;
			// LED_Flash(100, 2);					// LED1闪烁
			Kinematic_Analysis(Velocity, Turn); // 小车运动学分析
			PWMA = Velocity_A(-Target_A, encoderA_cnt);
			PWMB = Velocity_B(-Target_B, -encoderB_cnt);
			Set_PWM(PWMA, PWMB);
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

	if (Turn_Flag)
	{
		if (Turn > 0)
		{ // 右转。
			Target_A = (velocity + turn * 0.4);
			Target_B = (velocity - turn * 4); // 后轮差速
		}
		else if (Turn < 0)
		{
			Target_A = (velocity + turn * 4);
			Target_B = (velocity - turn * 0.4); // 后轮差速
		}
		else
		{
			Target_A = velocity;
			Target_B = velocity;
		}
	}
	else
	{
		if (Turn > 0)
		{ // 右转。
			Target_A = (velocity + turn * 1.6);
			Target_B = (velocity - turn * 1.6); // 后轮差速
		}
		else if (Turn < 0)
		{
			Target_A = (velocity + turn * 1.6);
			Target_B = (velocity - turn * 1.6); // 后轮差速
		}
		else
		{
			Target_A = velocity;
			Target_B = velocity;
		}
	}
}

void CCD_Mode(void)
{
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
		// sendToPc();
		printf("{A%%d:%d:%d:%d}$", CCD_Zhongzhi, encoderA_cnt, encoderB_cnt, Pitch, Roll, Yaw); // 打印到APP上面 显示波形
	}
}

/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
// void Get_Angle(uint8_t way)
// {
// 	float gyro_x, gyro_y, accel_x, accel_y, accel_z;
// 	float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
// 	if (way == 1) // DMP的读取在数据采集中断读取，严格遵循时序要求
// 	{
// 		Read_DMP();				   // 读取加速度、角速度、倾角
// 		Angle_Balance = Pitch;	   // 更新平衡倾角,前倾为正，后倾为负
// 		Gyro_Balance = gyro[0];	   // 更新平衡角速度,前倾为正，后倾为负
// 		Gyro_Turn = gyro[2];	   // 更新转向角速度
// 		Acceleration_Z = accel[2]; // 更新Z轴加速度计
// 	}
// 	else
// 	{
// 		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);	// 读取X轴陀螺仪
// 		Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);	// 读取Y轴陀螺仪
// 		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);	// 读取Z轴陀螺仪
// 		Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); // 读取X轴加速度计
// 		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); // 读取X轴加速度计
// 		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); // 读取Z轴加速度计
// 		if (Gyro_X > 32768)
// 			Gyro_X -= 65536; // 数据类型转换  也可通过short强制类型转换
// 		if (Gyro_Y > 32768)
// 			Gyro_Y -= 65536; // 数据类型转换  也可通过short强制类型转换
// 		if (Gyro_Z > 32768)
// 			Gyro_Z -= 65536; // 数据类型转换
// 		if (Accel_X > 32768)
// 			Accel_X -= 65536; // 数据类型转换
// 		if (Accel_Y > 32768)
// 			Accel_Y -= 65536; // 数据类型转换
// 		if (Accel_Z > 32768)
// 			Accel_Z -= 65536;								// 数据类型转换
// 		Gyro_Balance = -Gyro_X;								// 更新平衡角速度
// 		Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / Pi; // 计算倾角，转换单位为度
// 		Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / Pi; // 计算倾角，转换单位为度
// 		accel_x = Accel_X / 1671.84;
// 		accel_y = Accel_Y / 1671.84;
// 		accel_z = Accel_Z / 1671.84;
// 		gyro_x = Gyro_X / 16.4; // 陀螺仪量程转换
// 		gyro_y = Gyro_Y / 16.4; // 陀螺仪量程转换
// 		if (Way_Angle == 2)
// 		{
// 			Pitch = -Kalman_Filter_x(Accel_Angle_x, gyro_x); // 卡尔曼滤波
// 			Roll = -Kalman_Filter_y(Accel_Angle_y, gyro_y);
// 		}
// 		else if (Way_Angle == 3)
// 		{
// 			Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x); // 互补滤波
// 			Roll = -Complementary_Filter_y(Accel_Angle_y, gyro_y);
// 		}
// 		Angle_Balance = Pitch;	  // 更新平衡倾角
// 		Gyro_Turn = Gyro_Z;		  // 更新转向角速度
// 		Acceleration_Z = Accel_Z; // 更新Z轴加速度计
// 	}
// }

// /******************************************************************************
// ***
// * FUNCTION NAME: void sendToPc(void) *
// * CREATE DATE : 20170707 *
// * CREATED BY : XJU *
// * FUNCTION : 将待发送的信息通过串口发送至上位机*
// * MODIFY DATE : NONE *
// * INPUT : void *
// * OUTPUT : NONE *
// * RETURN : NONE *
// *******************************************************************************
// **/
// void sendToPc(void)
// {
// 	int i;
// 	slove_data();
// 	printf("*");
// 	printf("LD");
// 	for (i = 2; i < 134; i++)
// 	{
// 		printf("%c", binToHex_high(SciBuf[i])); // 以字符形式发送高4位对应的16进制
// 		printf("%c", binToHex_low(SciBuf[i]));	// 以字符形式发送低?位对应的16进制
// 	}
// 	printf("00"); // 通信协议要求
// 	printf("#");  // 通信协议要求
// }

// void slove_data(void)
// {
// 	int i;
// 	SciBuf[0] = 0;
// 	SciBuf[1] = 132;
// 	SciBuf[2] = 0;
// 	SciBuf[3] = 0;
// 	SciBuf[4] = 0;
// 	SciBuf[5] = 0;
// 	for (i = 0; i < 128; i++)
// 		SciBuf[6 + i] = ADV[i];
// }

// /******************************************************************************
//  ***
//  * FUNCTION NAME: binToHex_low(u8 num) *
//  * CREATE DATE : 20170707 *
//  * CREATED BY : XJU *
//  * FUNCTION : 将二进制低8位转换16进制*
//  * MODIFY DATE : NONE *
//  * INPUT : u8 *
//  * OUTPUT : NONE *
//  * RETURN : char *
//  *******************************************************************************
//  **/
// char binToHex_low(uint8_t num)
// {
// 	uint8_t low_four;
// 	low_four = num & 0x0f;
// 	if (low_four == 0)
// 		return ('0');
// 	else if (low_four == 1)
// 		return ('1');
// 	else if (low_four == 2)
// 		return ('2');
// 	else if (low_four == 3)
// 		return ('3');
// 	else if (low_four == 4)
// 		return ('4');
// 	else if (low_four == 5)
// 		return ('5');
// 	else if (low_four == 6)
// 		return ('6');
// 	else if (low_four == 7)
// 		return ('7');
// 	else if (low_four == 8)
// 		return ('8');
// 	else if (low_four == 9)
// 		return ('9');
// 	else if (low_four == 10)
// 		return ('A');
// 	else if (low_four == 11)
// 		return ('B');
// 	else if (low_four == 12)
// 		return ('C');
// 	else if (low_four == 13)
// 		return ('D');
// 	else if (low_four == 14)
// 		return ('E');
// 	else if (low_four == 15)
// 		return ('F');
// }

// /******************************************************************************
// ***
// * FUNCTION NAME: binToHex_low(u8 num) *
// * CREATE DATE : 20170707 *
// * CREATED BY : XJU *
// * FUNCTION : 将二进制高8位转换16进制*
// * MODIFY DATE : NONE *
// * INPUT : u8 *
// * OUTPUT : NONE *
// * RETURN : char *
// *******************************************************************************
// **/
// char binToHex_high(uint8_t num)
// {
// 	uint8_t high_four;
// 	high_four = (num >> 4) & 0x0f;
// 	if (high_four == 0)
// 		return ('0');
// 	else if (high_four == 1)
// 		return ('1');
// 	else if (high_four == 2)
// 		return ('2');
// 	else if (high_four == 3)
// 		return ('3');
// 	else if (high_four == 4)
// 		return ('4');
// 	else if (high_four == 5)
// 		return ('5');
// 	else if (high_four == 6)
// 		return ('6');
// 	else if (high_four == 7)
// 		return ('7');
// 	else if (high_four == 8)
// 		return ('8');
// 	else if (high_four == 9)
// 		return ('9');
// 	else if (high_four == 10)
// 		return ('A');
// 	else if (high_four == 11)
// 		return ('B');
// 	else if (high_four == 12)
// 		return ('C');
// 	else if (high_four == 13)
// 		return ('D');
// 	else if (high_four == 14)
// 		return ('E');
// 	else if (high_four == 15)
// 		return ('F');
// }
