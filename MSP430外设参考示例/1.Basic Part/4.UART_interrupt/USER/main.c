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
*                  ------------------------------------
********************************************************************************************************************/

/*定时器控制LEDB*闪烁
*按键KEY0短按开启LEDR 长按关闭
*按键KEY1短按开启LEDG 长按关闭
*串口打印hello
*发送数据，数据中断回环
*/
int main(void)
{
	SYSCFG_DL_init(); //芯片资源初始化,由SysConfig配置软件自动生成
    UART_Init();
    KEY_Init();
    Timer_Init();
	while(1)
	{
        printf("hello\r\n");
        if(Key0_Flag==1){Key0_Flag=0; LEDR(1); }
        else if(Key0_Flag==2){Key0_Flag=0; LEDR(0); }
        if(Key1_Flag==1){Key1_Flag=0; LEDG(1); }
        else if(Key1_Flag==2){Key1_Flag=0; LEDG(0); }
	}
}
