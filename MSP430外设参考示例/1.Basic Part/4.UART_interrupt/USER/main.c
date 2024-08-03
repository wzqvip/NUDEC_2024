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

/*��ʱ������LEDB*��˸
*����KEY0�̰�����LEDR �����ر�
*����KEY1�̰�����LEDG �����ر�
*���ڴ�ӡhello
*�������ݣ������жϻػ�
*/
int main(void)
{
	SYSCFG_DL_init(); //оƬ��Դ��ʼ��,��SysConfig��������Զ�����
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
