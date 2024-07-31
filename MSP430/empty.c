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
uint8_t Way_Angle = 2;						  // 获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
float Angle_Balance, Gyro_Balance, Gyro_Turn; // 平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;
int color = 1; // 1,2,3

float Total_Turns = 0;

int Initial_Turn = 0;	// 初始角度为0.
int Reverse_Turn = 180; // 返回角度
int Cross_Turn = 45;	// 对角线
int Recross_Turn = 135; // 返回

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
float Velocity_KP = 0.037, Velocity_KI = 0.007; // 速度控制PID参数

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
				if (cnt > 10) {
					LED_Blink(0, 100);
				}
			}
			if (cnt > 10) // 如果是长按
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
		CCD_Mode(); // CCD巡线PID
		// 结束CCD

		// printf("AGL\n");
		// MPU6050获取角度的代码
		Get_Angle(2); // 6050
		// 结束6050

		// APP_Show();
		//        printf("%d %d %d %d %d %d %d %f\n\r",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP);

		// printf("%f,%f,%f,%d\n",Pitch,Roll,Yaw,CCD_Zhongzhi);
		printf("%f,%d,%d,%d,%d,%d,%d,%d\n", Total_Turns, CCD_Zhongzhi, Target_A, encoderA_cnt, PWMA, Target_B, encoderB_cnt, PWMB);
	}
}

// 运动学使用这个地方的循环定时器来处理。
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
			// LED_Flash(100, 2);					// LEDS闪烁
			Kinematic_Analysis(Velocity, Turn); // 小车运动学分析
			PWMA = Velocity_A(-Target_A, -encoderA_cnt);
			PWMB = Velocity_B(-Target_B, encoderB_cnt);

			// printf("%d %d %d %d %d %d %d %d %d %d\n",CCD_Zhongzhi,Target_A,encoderA_cnt,PWMA,Target_B,encoderB_cnt,PWMB,Velocity_KP,Velocity_KI,Velocity);
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

	Target_A = (velocity + turn);
	Target_B = (velocity - turn); // 后轮差速
}

void CCD_Mode(void)
{
	float Bias, Last_Bias; // 这里原来是static float!!
	printf("BIAS: %f, %f \n", Bias, Last_Bias);
	Bias = CCD_Zhongzhi - 64;									  // 提取偏差
	Turn = Bias * Velocity_KP + (Bias - Last_Bias) * Velocity_KI; // PD控制
	Total_Turns += Turn;
	Turn = 0;
	Last_Bias = Bias; // 保存上一次的偏差
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
		printf("{A%%d:%d:%d:%d:%f:%f:%f}$", CCD_Zhongzhi, encoderA_cnt, encoderB_cnt, Pitch, Roll, Yaw); // 打印到APP上面 显示波形
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
void Get_Angle(uint8_t way)
{
	float gyro_x, gyro_y, accel_x, accel_y, accel_z;
	float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
	if (way == 1) // DMP的读取在数据采集中断读取，严格遵循时序要求
	{
		Read_DMP();				   // 读取加速度、角速度、倾角
		Angle_Balance = Pitch;	   // 更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance = gyro[0];	   // 更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn = gyro[2];	   // 更新转向角速度
		Acceleration_Z = accel[2]; // 更新Z轴加速度计
	}
	else
	{
		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);	// 读取X轴陀螺仪
		Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);	// 读取Y轴陀螺仪
		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);	// 读取Z轴陀螺仪
		Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); // 读取X轴加速度计
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); // 读取X轴加速度计
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); // 读取Z轴加速度计
		if (Gyro_X > 32768)
			Gyro_X -= 65536; // 数据类型转换  也可通过short强制类型转换
		if (Gyro_Y > 32768)
			Gyro_Y -= 65536; // 数据类型转换  也可通过short强制类型转换
		if (Gyro_Z > 32768)
			Gyro_Z -= 65536; // 数据类型转换
		if (Accel_X > 32768)
			Accel_X -= 65536; // 数据类型转换
		if (Accel_Y > 32768)
			Accel_Y -= 65536; // 数据类型转换
		if (Accel_Z > 32768)
			Accel_Z -= 65536;								// 数据类型转换
		Gyro_Balance = -Gyro_X;								// 更新平衡角速度
		Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / Pi; // 计算倾角，转换单位为度
		Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / Pi; // 计算倾角，转换单位为度
		accel_x = Accel_X / 1671.84;
		accel_y = Accel_Y / 1671.84;
		accel_z = Accel_Z / 1671.84;
		gyro_x = Gyro_X / 16.4; // 陀螺仪量程转换
		gyro_y = Gyro_Y / 16.4; // 陀螺仪量程转换
		if (Way_Angle == 2)
		{
			Pitch = -Kalman_Filter_x(Accel_Angle_x, gyro_x); // 卡尔曼滤波
			Roll = -Kalman_Filter_y(Accel_Angle_y, gyro_y);
		}
		else if (Way_Angle == 3)
		{
			Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x); // 互补滤波
			Roll = -Complementary_Filter_y(Accel_Angle_y, gyro_y);
		}
		Angle_Balance = Pitch;	  // 更新平衡倾角
		Gyro_Turn = Gyro_Z;		  // 更新转向角速度
		Acceleration_Z = Accel_Z; // 更新Z轴加速度计
	}
}
