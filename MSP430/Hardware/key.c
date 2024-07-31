#include "key.h"
/*
按钮默认0.
1： 右侧
2： 左侧
3： 外部按钮
*/

uint8_t click(void)
{
	uint8_t key_num=0; 
	if(DL_GPIO_readPins(KEYS_KEY_R_PORT,KEYS_KEY_R_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(KEYS_KEY_R_PORT,KEYS_KEY_R_PIN)==0);
		delay_ms(50);
		key_num=1; //右侧按钮
		printf("KeyR\n");
	}
	else if(DL_GPIO_readPins(KEYS_KEY_L_PORT,KEYS_KEY_L_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(KEYS_KEY_L_PORT,KEYS_KEY_L_PIN)==0);
		delay_ms(50);
		key_num=2; // 左侧按钮
		printf("KeyL\n");
	}
	else if(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0);
		delay_ms(50);
		key_num=3; // 外部按钮-启动
		printf("KeyEXT\n");
	}
	else {
		printf("NAN");
	}
	return key_num;
}



