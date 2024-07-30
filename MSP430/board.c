#include "board.h"
#include "stdio.h"
#include "string.h"


extern UART_Regs current_uart_port;

#define RE_0_BUFF_LEN_MAX	128
extern float Velocity,Turn;
volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;
extern float Velocity_KP,Velocity_KI;	       //�ٶȿ���PID����
uint8_t PID_Send;
//void board_init(void)
//{
//	// SYSCFG��ʼ��
//	SYSCFG_DL_init();
//	//��������жϱ�־
//	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//	//ʹ�ܴ����ж�
//	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
//	
//	printf("Board Init [[ ** LCKFB ** ]]\r\n");
//}

//����δ�ʱ��ʵ�ֵľ�ȷus��ʱ
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // ������Ҫ��ʱ���� = �ӳ�΢���� * ÿ΢���ʱ����
    ticks = __us * (32000000 / 1000000);

    // ��ȡ��ǰ��SysTickֵ
    told = SysTick->VAL;

    while (1)
    {
        // �ظ�ˢ�»�ȡ��ǰ��SysTickֵ
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // ����ﵽ����Ҫ��ʱ���������˳�ѭ��
            if (tcnt >= ticks)
                break;
        }
    }
}
//����δ�ʱ��ʵ�ֵľ�ȷms��ʱ
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//���ڷ��͵����ַ�
void uart0_send_char(char ch)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_0_INST) == true );
	//���͵����ַ�
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//���ڷ����ַ���
void uart0_send_string(char* str)
{
	//��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while(*str!=0&&str!=0)
	{
		//�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart0_send_char(*str++);
	}
}


#if !defined(__MICROLIB)
//��ʹ��΢��Ļ�����Ҫ�������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
//�����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif


// //printf�����ض���
// int fputc(int ch, FILE *stream)
// {
// 	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
// 	while( DL_UART_isBusy(UART_1_INST) == true );
	
// 	DL_UART_Main_transmitData(UART_1_INST, ch);
	
// 	return ch;
// }

//printf�����ض���
int fputc(int ch, FILE *stream)
{
    // ������æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
    while(DL_UART_isBusy(UART_0_INST) == true);
    
    DL_UART_Main_transmitData(UART_0_INST, ch);
    
    return ch;
}



//���ڵ��жϷ�����
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//��������˴����ж�
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://����ǽ����ж�
			
			// ���շ��͹��������ݱ���
			receivedData = DL_UART_Main_receiveData(UART_0_INST);

			// ��黺�����Ƿ�����
			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
			{
				recv0_buff[recv0_length++] = receivedData;

				// ������������ٷ��ͳ�ȥ������ش�����ע�͵�
				uart0_send_char(receivedData);
			}
			else
			{
				recv0_length = 0;
			}

			// ��ǽ��ձ�־
			recv0_flag = 1;
		
			break;
		
		default://�����Ĵ����ж�
			break;
	}
}

//���ڵ��жϷ�����
void UART_1_INST_IRQHandler(void)
{
	uint8_t  UART1_Data = 0;
	static uint8_t Flag_PID,i,j,Receive[50],Last_Usart1_Data;
	static float Data;
	//��������˴����ж�
	switch( DL_UART_getPendingInterrupt(UART_1_INST) )
	{
		case DL_UART_IIDX_RX://����ǽ����ж�
			
			// ���շ��͹��������ݱ���
			UART1_Data = DL_UART_Main_receiveData(UART_1_INST);
            		//��������APP���Խ���ͨѶ
		if(UART1_Data==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
		if(UART1_Data==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ

		 if(Flag_PID==1)  //�ɼ�����
		 {
			Receive[i]=UART1_Data;
			i++;
		 }
		 if(Flag_PID==2)  //��������
		 {
			 if(Receive[3]==0x50) 	 PID_Send=1;
			 else  if(Receive[1]!=0x23) 
			 {								
				for(j=i;j>=4;j--)
				{
				  Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  Velocity_KP=Data/1000;break;
					 case 0x31:  Velocity_KI=Data/1000;break;
					 case 0x32:  Velocity=Data;break;
					 case 0x33:  break;
					 case 0x34:  break;
					 case 0x35:  break;
					 case 0x36:  break;
					 case 0x37:  break; //Ԥ��
					 case 0x38:  break; //Ԥ��
				 }
				}				 
			 Flag_PID=0;//��ر�־λ����
			 i=0;
			 j=0;
			 Data=0;
			 memset(Receive, 0, sizeof(uint8_t)*50);//��������
		 } 	 			            
		
			break;
		
		default://�����Ĵ����ж�
			break;
	}
}

