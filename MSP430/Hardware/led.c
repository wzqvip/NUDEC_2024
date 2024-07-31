#include "led.h"

/*
LED0 蜂鸣器
LED1 红色
LED2 绿色
LED3 蓝色
*/

// 注意： 板载的灯设置低电平是亮。蜂鸣器是高电平。
void LED_ON(int LED)
{
	if(LED == 0) DL_GPIO_setPins(BEEPER_PORT,BEEPER_BEEP_PIN);
	if(LED==1) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_R_PIN);
	else if(LED==2) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_G_PIN);
	else if(LED==3) DL_GPIO_clearPins(LEDS_PORT,LEDS_LED_B_PIN);
}

void LED_OFF(int LED)
{
	if(LED == 0) DL_GPIO_clearPins(BEEPER_PORT,BEEPER_BEEP_PIN);
	if(LED==1) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_R_PIN);
	else if(LED==2) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_G_PIN);
	else if(LED==3) DL_GPIO_setPins(LEDS_PORT,LEDS_LED_B_PIN);
}

void LED_Toggle(int LED)
{
	if(LED == 0) DL_GPIO_togglePins(BEEPER_PORT,BEEPER_BEEP_PIN);
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


