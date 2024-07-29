#ifndef __HMC5883_H
#define __HMC5883_H

#include "A_include.h"

//#define HMC_SCL(x)       sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN6, x)
//#define HMC_SDA(x)       sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN7, x)
//#define HMC_Read_SDA     sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN7)

#define WRITE_ADDRESS  0x3c
#define READ_ADDRESS   0x3d

#define CONFIGA 0x00//���üĴ��� A 
#define CONFIGB 0x01//���üĴ��� B
#define MODE    0x02//ģʽ�Ĵ���
#define DATA_X_H 0x03//������� X MSB �Ĵ���
#define DATA_X_L 0x04//�������X LSB �Ĵ���
#define DATA_Y_H 0x05//������� Y MSB�Ĵ���
#define DATA_Y_L 0x06//�������Y LSB �Ĵ���
#define DATA_Z_H 0x07//�������Z MSB �Ĵ���
#define DATA_Z_L 0x08//�������Z LSB �Ĵ���
#define STATE   0x09//״̬�Ĵ���

void HMC5883_Init(void);
void Read_HMC5883(void);

#endif


















