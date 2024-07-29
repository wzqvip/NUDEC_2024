#ifndef __HMC5883_H
#define __HMC5883_H

#include "A_include.h"

//#define HMC_SCL(x)       sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN6, x)
//#define HMC_SDA(x)       sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN7, x)
//#define HMC_Read_SDA     sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN7)

#define WRITE_ADDRESS  0x3c
#define READ_ADDRESS   0x3d

#define CONFIGA 0x00//配置寄存器 A 
#define CONFIGB 0x01//配置寄存器 B
#define MODE    0x02//模式寄存器
#define DATA_X_H 0x03//数据输出 X MSB 寄存器
#define DATA_X_L 0x04//数据输出X LSB 寄存器
#define DATA_Y_H 0x05//数据输出 Y MSB寄存器
#define DATA_Y_L 0x06//数据输出Y LSB 寄存器
#define DATA_Z_H 0x07//数据输出Z MSB 寄存器
#define DATA_Z_L 0x08//数据输出Z LSB 寄存器
#define STATE   0x09//状态寄存器

void HMC5883_Init(void);
void Read_HMC5883(void);

#endif


















