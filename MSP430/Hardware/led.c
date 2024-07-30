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
void LED_ON(int LED)
{
	if(LED==1) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_R_PIN);
	else if(LED==2) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_G_PIN);
	else if(LED==3) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_B_PIN);
}

void LED_OFF(int LED)
{
	if(LED==1) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_R_PIN);
	else if(LED==2) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_G_PIN);
	else if(LED==3) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_B_PIN);
}

void LED_Toggle(int LED)
{
	if(LED==1) DL_GPIO_togglePins(LEDS_PORT,LEDS_LED_R_PIN);
	else if(LED==2) DL_GPIO_togglePins(LEDS_PORT,LEDS_LED_G_PIN);
	else if(LED==3) DL_GPIO_togglePins(LEDS_PORT,LEDS_LED_B_PIN);
}

void LED_Flash(uint16_t time, int LED) 
{
	static uint16_t temp;
	if(time==0) LED_ON(LED);
	else if(++temp==time) LED_Toggle(LED),temp=0;
}


