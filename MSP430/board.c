#include "board.h"
#include "stdio.h"
#include "string.h"


extern UART_Regs current_uart_port;

#define RE_0_BUFF_LEN_MAX	128
extern float Velocity,Turn;
volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;
extern float Velocity_KP,Velocity_KI;	       //速度控制PID参数
uint8_t PID_Send;
//void board_init(void)
//{
//	// SYSCFG初始化
//	SYSCFG_DL_init();
//	//清除串口中断标志
//	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//	//使能串口中断
//	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
//	
//	printf("Board Init [[ ** LCKFB ** ]]\r\n");
//}

//搭配滴答定时器实现的精确us延时
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
    ticks = __us * (32000000 / 1000000);

    // 获取当前的SysTick值
    told = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的SysTick值
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // 如果达到了需要的时钟数，就退出循环
            if (tcnt >= ticks)
                break;
        }
    }
}
//搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//串口发送单个字符
void uart0_send_char(char ch)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送单个字符
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//串口发送字符串
void uart0_send_string(char* str)
{
	//当前字符串地址不在结尾 并且 字符串首地址不为空
	while(*str!=0&&str!=0)
	{
		//发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart0_send_char(*str++);
	}
}


#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif


// //printf函数重定义
// int fputc(int ch, FILE *stream)
// {
// 	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
// 	while( DL_UART_isBusy(UART_1_INST) == true );
	
// 	DL_UART_Main_transmitData(UART_1_INST, ch);
	
// 	return ch;
// }

//printf函数重定义
int fputc(int ch, FILE *stream)
{
    // 当串口忙的时候等待，不忙的时候再发送传进来的字符
    while(DL_UART_isBusy(UART_0_INST) == true);
    
    DL_UART_Main_transmitData(UART_0_INST, ch);
    
    return ch;
}



//串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//如果产生了串口中断
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断
			
			// 接收发送过来的数据保存
			receivedData = DL_UART_Main_receiveData(UART_0_INST);

			// 检查缓冲区是否已满
			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
			{
				recv0_buff[recv0_length++] = receivedData;

				// 将保存的数据再发送出去，不想回传可以注释掉
				uart0_send_char(receivedData);
			}
			else
			{
				recv0_length = 0;
			}

			// 标记接收标志
			recv0_flag = 1;
		
			break;
		
		default://其他的串口中断
			break;
	}
}

//串口的中断服务函数
void UART_1_INST_IRQHandler(void)
{
	uint8_t  UART1_Data = 0;
	static uint8_t Flag_PID,i,j,Receive[50],Last_Usart1_Data;
	static float Data;
	//如果产生了串口中断
	switch( DL_UART_getPendingInterrupt(UART_1_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断
			
			// 接收发送过来的数据保存
			UART1_Data = DL_UART_Main_receiveData(UART_1_INST);
            		//以下是与APP调试界面通讯
		if(UART1_Data==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(UART1_Data==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
			Receive[i]=UART1_Data;
			i++;
		 }
		 if(Flag_PID==2)  //分析数据
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
					 case 0x37:  break; //预留
					 case 0x38:  break; //预留
				 }
				}				 
			 Flag_PID=0;//相关标志位清零
			 i=0;
			 j=0;
			 Data=0;
			 memset(Receive, 0, sizeof(uint8_t)*50);//数组清零
		 } 	 			            
		
			break;
		
		default://其他的串口中断
			break;
	}
}

