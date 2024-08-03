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
int main(void)
{
	SYSCFG_DL_init(); //оƬ��Դ��ʼ��,��SysConfig��������Զ�����
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


