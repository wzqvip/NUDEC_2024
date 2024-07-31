#include "key.h"
/*
��ťĬ��0.
1�� �Ҳ�
2�� ���
3�� �ⲿ��ť
*/

uint8_t click(void)
{
	uint8_t key_num=0; 
	if(DL_GPIO_readPins(KEYS_KEY_R_PORT,KEYS_KEY_R_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(KEYS_KEY_R_PORT,KEYS_KEY_R_PIN)==0);
		delay_ms(50);
		key_num=1; //�Ҳఴť
		printf("KeyR\n");
	}
	else if(DL_GPIO_readPins(KEYS_KEY_L_PORT,KEYS_KEY_L_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(KEYS_KEY_L_PORT,KEYS_KEY_L_PIN)==0);
		delay_ms(50);
		key_num=2; // ��ఴť
		printf("KeyL\n");
	}
	else if(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0);
		delay_ms(50);
		key_num=3; // �ⲿ��ť-����
		printf("KeyEXT\n");
	}
	else {
		printf("NAN");
	}
	return key_num;
}



