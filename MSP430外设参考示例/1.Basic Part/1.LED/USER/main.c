#include "A_include.h"

/********************************************************************************************************************
*                  ------------------------------------
LEDS      A0
LEDR      B26
LEDG      B27
LEDB      B22
*                  ------------------------------------
********************************************************************************************************************/

/*LED*闪烁*/
int main(void)
{
	SYSCFG_DL_init(); //芯片资源初始化,由SysConfig配置软件自动生成                           
	while(1)
	{
        LED_Test();
	}
}


