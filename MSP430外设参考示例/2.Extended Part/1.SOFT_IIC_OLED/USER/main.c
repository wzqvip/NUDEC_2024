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
*KEY1外部中断控制LED翻转
*/
int main(void)
{
	SYSCFG_DL_init(); //芯片资源初始化,由SysConfig配置软件自动生成
    UART_Init();
    KEY_Init();
    Exti_Init();
    OLED_Init();   
	while(1)
	{
        printf("hello\r\n");
        for(uint8_t i=0;i<128;i++){
        OLED_ShowString(i,0*16,"Hello Word",OLED_8X16);
        OLED_Update();  }
	}
}


