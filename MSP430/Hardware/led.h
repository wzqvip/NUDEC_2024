#ifndef _LED_H
#define _LED_H
#include "ti_msp_dl_config.h"
#include "board.h"

void LED_ON(int LED);
void LED_OFF(int LED);
void LED_Toggle(int LED);
void LED_Flash(uint16_t time, int LED);
#endif 