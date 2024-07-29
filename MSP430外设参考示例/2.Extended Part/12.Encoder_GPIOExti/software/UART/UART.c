#include "uart.h"
/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{

    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)ch);
    return ch;
}
#endif
void UART0_Text(void)
{
    uint8_t send_test[13]={"hello world!\n"};
//    uint8_t *p={"hello world!\n"};
    UART0_Send_Bytes(send_test,sizeof(send_test));
//    UART0_Send_Bytes(p,sizeof(p));
}


void UART0_Send_Bytes(uint8_t *buf, int len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(UART_0_INST, *buf);
    buf++;
  }
}


void UART_Init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);//清中断挂起
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);//使能串口中断
}


void UART_0_INST_IRQHandler(void)
{
    if(DL_UART_Main_getPendingInterrupt(UART_0_INST)==DL_UART_MAIN_IIDX_RX) 
    {
			uint8_t Data=DL_UART_Main_receiveData(UART_0_INST);//串口数据接受单个字节数据
			DL_UART_Main_transmitData(UART_0_INST, Data);//将本次解析的到数据ch发送出去
    }
}




