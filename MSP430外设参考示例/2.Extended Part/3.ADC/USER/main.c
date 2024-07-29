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
********************************************************************************************************************/
/*
*AD转换
*OLED 显示
*/
int main(void)
{
	SYSCFG_DL_init(); //芯片资源初始化,由SysConfig配置软件自动生成
    UART_Init();
    KEY_Init();
    Exti_Init();
    ADC_Init();
    OLED_Init();   
	while(1)
	{
        printf("hello\r\n");
        ADC_start();
        OLED_ShowSignedNum(0,2*16,gAdcResult[0],4,OLED_8X16);
        OLED_ShowString(0,3*16,"Hello Word",OLED_8X16);
        OLED_Update();  
	}
}


