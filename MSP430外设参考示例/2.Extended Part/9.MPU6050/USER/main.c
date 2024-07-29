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
OLED_SCL  A15
OLED_SDA  A16
ADC       A25

OLED_SCL(D0)  B16
//MOSI()      B7
OLED_MISO(D1) B15
OLED_RES      B17
OLED_DC       B1
OLED_SS(CS)   A29  

********************************************************************************************************************/
/*
*驱动电机10KHZPWM           B8 A9 B0 B13
*驱动舵机 50hz 20ms PWM     A21
*DAC开启内部通道放大才能正常输出 A22
*定时器控制LEDB*闪烁
*软件SPIO  LED显示
*/

float mpu_data[3];
short mpu_acc[3];
int main(void)
{
	SYSCFG_DL_init(); //芯片资源初始化,由SysConfig配置软件自动生成
    UART_Init();
    KEY_Init();
    ADC_Init();
    DAC_Init();
    Sever_Init();
    Motor_Init();
    OLED_Init(); 
    mpu_dmp_init();                               // mpu6050初始1化     
    Timer_Init();
	while(1)
	{   
        
        mpu_dmp_get_data(&mpu_data[0],&mpu_data[1],&mpu_data[2]);
        MPU_Get_Gyroscope(&mpu_acc[0],&mpu_acc[1],&mpu_acc[2]);//陀螺仪 
        OLED_ShowSignedNum(0,2*16,mpu_data[0],4,OLED_8X16);
        OLED_Update();        
        
//        DAC_start(3300);    //2.7V
//        Servo_Ctrl(0);
//        Motor0_Ctrl(400,800);
//        Motor1_Ctrl(1600,2400);
//        printf("QVQ\r\n");
//        ADC_start();
//        OLED_ShowSignedNum(0,2*16,gAdcResult[0],4,OLED_8X16);
//        OLED_ShowString(0,3*16,"QVQ",OLED_8X16);
//        OLED_Update();        
//        if(KEY0)LEDR(1);
//        else LEDR(0);
//        if(!KEY1)LEDB(1);
//        else LEDB(0);
	}
}


