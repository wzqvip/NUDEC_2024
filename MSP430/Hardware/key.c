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
	if(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(EXTENAL_KEY_PORT,EXTENAL_KEY_BUTTON_PIN)==0);
		delay_ms(50);
		key_num=3; // �ⲿ��ť-����
		// printf("KeyEXT\n");
	}

	return key_num;
}



