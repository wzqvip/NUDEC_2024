#ifndef __MYIIC_H_
#define __MYIIC_H_

#include "A_include.h"

#define IIC_Read         DL_GPIO_readPins(GPIOA,  DL_GPIO_PIN_30)
#define IIC_SCL(x)      ((x)?(DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_14)) : (DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_14)) )
#define IIC_SDA(x)      ((x)?(DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_30)) : (DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_30)) )

#endif



