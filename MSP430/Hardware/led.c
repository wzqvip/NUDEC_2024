#include "led.h"
/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2024-07-019

All rights reserved
***********************************************/
void LED_ON(void)
{
	DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_R_PIN);
}

void LED_OFF(void)
{
	DL_GPIO_setPins(LEDS_PORT,LEDS_LED_R_PIN);
}

void LED_Toggle(void)
{
	DL_GPIO_togglePins(LEDS_PORT,LEDS_LED_R_PIN);
}

void LED_Flash(uint16_t time)
{
	static uint16_t temp;
	if(time==0) LED_ON();
	else if(++temp==time) LED_Toggle(),temp=0;
}


