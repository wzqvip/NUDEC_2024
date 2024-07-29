#ifndef _A_include_H_
#define _A_include_H_

/******************系统头文件*******************/
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
/******************用户文件*******************/
#include "A_include.h"
/******************software*******************/
#include "..\software\delay\delay.h"
#include "..\software\Timer\Timer.h"
#include "..\software\uart\uart.h"
#include "..\software\MYIIC\MYIIC.h"
/******************HARDWARE*******************/
#include "..\HARDWARE\LED\LED.h"
#include "..\HARDWARE\key\key.h"
#include "..\HARDWARE\OLED\OLED.h"
#include "..\HARDWARE\Exti\Exti.h"
#include "..\HARDWARE\ADC\ADC.h"

#endif
