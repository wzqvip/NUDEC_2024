#include "led.h"

/*
LED0 ·äÃùÆ÷
LED1 ºìÉ«
LED2 ÂÌÉ«
LED3 À¶É«
*/

void LED_ON(int LED)
{
	if (LED == 0)
		DL_GPIO_setPins(BEEPER_PORT, BEEPER_BEEP_PIN);
		DL_GPIO_setPins(LEDS_PORT, LEDS_LED_G_PIN);
	if (LED == 1)
		DL_GPIO_setPins(LEDS_PORT, LEDS_LED_R_PIN);
	else if (LED == 2)
		DL_GPIO_setPins(LEDS_PORT, LEDS_LED_G_PIN);
	else if (LED == 3)
		DL_GPIO_setPins(LEDS_PORT, LEDS_LED_B_PIN);
}

void LED_OFF(int LED)
{
	if (LED == 0)
		DL_GPIO_clearPins(BEEPER_PORT, BEEPER_BEEP_PIN);
		DL_GPIO_setPins(LEDS_PORT, LEDS_LED_G_PIN);
	if (LED == 1)
		DL_GPIO_clearPins(LEDS_PORT, LEDS_LED_R_PIN);
	else if (LED == 2)
		DL_GPIO_clearPins(LEDS_PORT, LEDS_LED_G_PIN);
	else if (LED == 3)
		DL_GPIO_clearPins(LEDS_PORT, LEDS_LED_B_PIN);
}

void LED_Blink(int LED, int ms)
{
	LED_ON(LED);
	delay_ms(ms);
	LED_OFF(LED);
}

void LED_Toggle(int LED)
{
	if (LED == 0)
		DL_GPIO_togglePins(BEEPER_PORT, BEEPER_BEEP_PIN);
	if (LED == 1)
		DL_GPIO_togglePins(LEDS_PORT, LEDS_LED_R_PIN);
	else if (LED == 2)
		DL_GPIO_togglePins(LEDS_PORT, LEDS_LED_G_PIN);
	else if (LED == 3)
		DL_GPIO_togglePins(LEDS_PORT, LEDS_LED_B_PIN);
}

void LED_Flash(uint16_t time, int LED)
{
	static uint16_t temp;
	if (time == 0)
		LED_ON(LED);
	else if (++temp == time)
		LED_Toggle(LED), temp = 0;
}
