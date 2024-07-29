#include "HMC5883.h"

#define HMC_SDA_OUT  {                                                      \
                        DL_GPIO_initDigitalOutput(MYIIC_MYSDA_IOMUX);           \
                        DL_GPIO_enableOutput(MYIIC_MYSDA_PORT, MYIIC_MYSDA_PIN);\
                        }
#define HMC_SDA_IN        DL_GPIO_initDigitalInput(MYIIC_MYSDA_IOMUX)
#define HMC_SCL(x)      ( (x) ? (DL_GPIO_setPins(MYIIC_MYSCL_PORT,MYIIC_MYSCL_PIN)) : (DL_GPIO_clearPins(MYIIC_MYSCL_PORT,MYIIC_MYSCL_PIN)) )
#define HMC_SDA(x)      ( (x) ? (DL_GPIO_setPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN)) : (DL_GPIO_clearPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN)) )
#define HMC_Read_SDA        (((DL_GPIO_readPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN) & MYIIC_MYSDA_PIN ) > 0 ) ? 1 : 0 )

static void HMC_delay_us(uint32_t x)
{
    delay_us(x);
}
//����IIC��ʼ�ź�
static void HMC5883_Start(void)
{
	HMC_SDA_OUT; 
	HMC_SDA(1);	  	  
	HMC_SCL(1);
	HMC_delay_us(4);
 	HMC_SDA(0);//START:when CLK is high,DATA change form high to low 
	HMC_delay_us(4);
	HMC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
static void HMC5883_Stop(void)
{
    HMC_SDA_OUT; 
	HMC_SCL(0);
	HMC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	HMC_delay_us(4);
	HMC_SCL(1); 
	HMC_SDA(1);//����I2C���߽����ź�
	HMC_delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static uint8_t HMC5883_RecvACK(void)
{
	uint8_t ucErrTime=0;
	HMC_SDA_IN;
	HMC_SDA(1);
    HMC_delay_us(1);	   
	HMC_SCL(1);
    HMC_delay_us(1);	 
	while(HMC_Read_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			HMC5883_Stop();
			return 1;
		}
	}
	HMC_SCL(0);//ʱ�����0 	   
	return 0;  
} 

static void HMC5883_SendACK(uint8_t ack)
{
    HMC_SCL(0);  
    HMC_SDA_OUT; 
    HMC_SDA(ack);   //дӦ���ź�        
    HMC_SDA(1);                    //����ʱ����
    HMC_delay_us(5);                 //��ʱ
    HMC_SCL(0);                 //����ʱ����
    HMC_delay_us(5);                 //��ʱ
}

	  
static void Single_Write_HMC5883(uint8_t txd)
{                        
    uint8_t t;   
    HMC_SDA_OUT; 
    HMC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //HMC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)HMC_SDA(1);
		else HMC_SDA(0);
			
		txd<<=1; 	  
		HMC_delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		HMC_SCL(1);
		HMC_delay_us(2); 
		HMC_SCL(0);	
		HMC_delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static uint8_t IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	HMC_SDA_IN;
    for(i=0;i<8;i++ )
	{
        HMC_SCL(0);
        HMC_delay_us(5);
		HMC_SCL(1);
        receive<<=1;
        if(HMC_Read_SDA)receive++;   
		HMC_delay_us(5); 
    }
	return receive;
}

/*static IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        HMC_SCL(0);
        HMC_delay_us(2);
		HMC_SCL(1);
        receive<<=1;
        if(HMC_Read_SDA)receive++;   
		HMC_HMC_delay_us(1); 
    }					 
    if (!ack)
        HMC5883_SendACK(0);//����nACK
    else
        HMC5883_SendACK(); //����ACK   
    return receive;
}*/
static void HMC_Write_Reg(uint8_t reg,uint8_t data)
{
	HMC5883_Start();
	Single_Write_HMC5883(WRITE_ADDRESS);
	HMC5883_RecvACK();
	Single_Write_HMC5883(reg);
	HMC5883_RecvACK();
	Single_Write_HMC5883(data);
	HMC5883_RecvACK();
	HMC5883_Stop();
}

static uint8_t HMC_Read_Reg(uint8_t reg)
{
	unsigned char data;
	HMC5883_Start();
	Single_Write_HMC5883(WRITE_ADDRESS);
	HMC5883_RecvACK();
	Single_Write_HMC5883(reg);
	HMC5883_RecvACK();
	HMC5883_Stop();
	HMC5883_Start();
	Single_Write_HMC5883(READ_ADDRESS);
	HMC5883_RecvACK();
	data = IIC_Read_Byte();
	HMC5883_SendACK(0);
	HMC5883_Stop();
	return data;
}



void HMC5883_Init(void)
{
//   RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
//   sys_gpio_set(GPIOB, SYS_GPIO_PIN8 | SYS_GPIO_PIN9,
//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_OD, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);   
	HMC_Write_Reg(CONFIGA,0x18);//���üĴ��� A   ������ʺͲ������� �������50HZ ������������
	HMC_Write_Reg(CONFIGB,0xC0);//���üĴ��� B   ����  ��4.5Ga   390   0xF800�C0x07FF (-2048�C2047 ) 
	HMC_Write_Reg(MODE,0x00);	//����ת��ģʽ
}

#define PI 3.14159265
double x,y,z,h;

void Read_HMC5883(void)
{
	int32_t data[3];
	uint16_t temp;
	temp=HMC_Read_Reg(DATA_X_H);//��� X �Ĵ��� A ��B
	data[0]=(temp<<8)+HMC_Read_Reg(DATA_X_L);
	temp=HMC_Read_Reg(DATA_Y_H);//��� Y �Ĵ��� A �� B
	data[1]=(temp<<8)+HMC_Read_Reg(DATA_Y_L);
	temp=HMC_Read_Reg(DATA_Z_H);//��� Z �Ĵ��� A �� B
	data[2]=(temp<<8)+HMC_Read_Reg(DATA_Z_L);
    
	if(data[0]>=32768)  //�Զ����ƵĲ�����ʽ
  {  
    data[0] = -(0xFFFF - data[0]+ 1);  
  }  
    
  if(data[1]>=32768)  
  {  
    data[1] = -(0xFFFF - data[1] + 1);  
  }  
  if(data[2]>=32768)  
  {  
    data[2] = -(0xFFFF - data[2] + 1);  
  }  
  
	x = data[0]/2*PI;
	y = data[1]/2*PI;
	z = data[2]/2*PI;
	h = sqrt(x*x+y*y+z*z);//ƽ����
  
    OLED_ShowSignedNum(6*4,8*1,x,6,OLED_6X8);  
    OLED_ShowSignedNum(6*4,8*2,y,6,OLED_6X8);  
    OLED_ShowSignedNum(6*4,8*3,z,6,OLED_6X8);          
    OLED_ShowSignedNum(6*4,8*4,h,6,OLED_6X8); 
    OLED_Update();  
}

