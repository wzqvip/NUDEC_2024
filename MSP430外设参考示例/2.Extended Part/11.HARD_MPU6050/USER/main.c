#include "A_include.h"
/********************************************************************************************************************
*                  ------------------------------------
LEDS      A0
LEDR      B26
LEDG      B27
LEDB      B22
KEY0      A18
KEY1      B21
UART0TX   A10 
UART0RX   A11
OLED_SCL  B14
OLED_SDA  PA30

********************************************************************************************************************/
/*
*KEY1�ⲿ�жϿ���LED��ת
*/

int16_t accel[3],gyro[3];
float temperature;
int main(void)
{
	SYSCFG_DL_init(); //оƬ��Դ��ʼ��,��SysConfig��������Զ�����
    UART_Init();
    KEY_Init();
    Exti_Init();
    OLED_Init();   
    mpu6050_init();//mpu6050��ʼ��
	while(1)
	{
        
        
        mpu6050_read(gyro,accel,&temperature);
        float ax,ay,az;
        ax=accel[0];
        ay=accel[1];
        az=accel[2];
        float roll=-57.3f*atan(ax/sqrtf(ay*ay+az*az)); //�����
        float pitch= 57.3f*atan(ay/sqrtf(ax*ax+az*az));//������		
        OLED_ShowSignedNum(0,2*16,roll,4,OLED_8X16);
        OLED_ShowSignedNum(0,1*16,pitch,4,OLED_8X16);
        OLED_Update();  
//        printf("hello\r\n");
//        for(uint8_t i=0;i<128;i++)
//        {
//        OLED_ShowString(i,0*16,"Hello Word",OLED_8X16);
//        OLED_Update();  
//        }
	}
}


