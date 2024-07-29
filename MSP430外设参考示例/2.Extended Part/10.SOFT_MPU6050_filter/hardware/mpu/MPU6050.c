#include "MPU6050.h"
                                  
static void MPU_delay(uint32_t x)
{
    delay_us(x);
}
/******************************************************************
 * 函 数 名 称：MyI2C_Start
 * 函 数 说 明：IIC起始时序
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
static void MyI2C_Start(void)
{
        SDA_OUT();
        SCL(1); 
        SDA(0);
        SDA(1);
        delay_us(5);
        SDA(0);
        delay_us(5);   
        SCL(0);
}
/******************************************************************
 * 函 数 名 称：MyI2C_Stop
 * 函 数 说 明：IIC停止信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
static void MyI2C_Stop(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        SCL(1);
        MPU_delay(5);
        SDA(1);
        MPU_delay(5);
}
/******************************************************************
 * 函 数 名 称：MyI2C_SendByte
 * 函 数 说 明：写入一个字节
 * 函 数 形 参：dat要写人的数据
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
static void MyI2C_SendByte(uint8_t dat)
{
    int i = 0;
    SDA_OUT();
    SCL(0);//拉低时钟开始数据传输
    
    for( i = 0; i < 8; i++ )
    {
            SDA( (dat & 0x80) >> 7 );
            delay_us(1);
            SCL(1);
            delay_us(5);
            SCL(0);
            delay_us(5);
            dat<<=1;
    }        
}
/******************************************************************
 * 函 数 名 称：MyI2C_ReceiveByte
 * 函 数 说 明：IIC读
 * 函 数 形 参：无
 * 函 数 返 回：读到的数据
 * 作       者：LC
 * 备       注：无
******************************************************************/
static uint8_t MyI2C_ReceiveByte(void)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
    {
    SCL(0);
    delay_us(5);
    SCL(1);
    delay_us(5);
    receive<<=1;
    if( SDA_GET() )
    {        
    receive|=1;   
    }
    delay_us(5); 
    }                                         
    SCL(0); 
    return receive;
}
/******************************************************************
 * 函 数 名 称：MyI2C_SendAck
 * 函 数 说 明：主机发送应答或者非应答信号
 * 函 数 形 参：0发送应答  1发送非应答
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
static void MyI2C_SendAck(uint8_t ack)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_us(5);
        SDA(ack);
        SCL(1);
        delay_us(5);
        SCL(0);
        SDA(1);
}
/******************************************************************
 * 函 数 名 称：MyI2C_ReceiveAck
 * 函 数 说 明：等待从机应答
 * 函 数 形 参：无
 * 函 数 返 回：0有应答  1超时无应答
 * 作       者：LC
 * 备       注：无
******************************************************************/
static uint8_t MyI2C_ReceiveAck(void)
{
    char ack = 0;
    unsigned char ack_flag = 10;
    SCL(0);
    SDA(1);
    SDA_IN();
    SCL(1);
    while( (SDA_GET()==1) && ( ack_flag ) )
    {
            ack_flag--;
            delay_us(5);
    }
    if( ack_flag <= 0 )
    {
            MyI2C_Stop();
            return 1;
    }
    else
    {
            SCL(0);
            SDA_OUT();
    }
    return ack;
}

/**********以上为IIC驱动函数**************************************************/


#define MPU6050_ADDRESS		0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();				//起始信号
	MyI2C_SendByte(MPU6050_ADDRESS);	//从机地址
	MyI2C_ReceiveAck();			//应答
	MyI2C_SendByte(RegAddress);			//指定从机下的指定寄存器地址
	MyI2C_ReceiveAck();			//应答
	MyI2C_SendByte(Data);				//指定寄存器地址下写入数据
	MyI2C_ReceiveAck();			//应答
	MyI2C_Stop();				//停止信号
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
}

void MPU6050_Init(void)
{
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}














#define kp 				20.00f
#define ki 				0.001f
#define cycle_T 		0.005f//200hz
#define half_T 			0.0025f

int16_t Ax,Ay,Az,Gx,Gy,Gz;
param_imu imu_data;
param_Angle imu_Angle;

float q[4] = {1.0,0.0,0.0,0.0};
float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;

//////////////////////////////////////////////////////////////////////////////////
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);  //0x5f3759df是一个平方根倒数速算法
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//////////////////////////////////////////////////////////////////////////////////

void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
    
	ax = imu_temp->AX;
	ay = imu_temp->AY;
	az = imu_temp->AZ;
	gx = imu_temp->GX;
	gy = imu_temp->GY;
	gz = imu_temp->GZ;
	
	float vx, vy, vz; 
	float ex, ey, ez; 
	
	float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];
	
	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
	
	vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q2*q3 + q0*q1);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
    ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
	
	exInt += ki * ex;
	eyInt += ki * ey;
	ezInt += ki * ez;
	
	gx += kp * ex + exInt;
	gy += kp * ey + eyInt;
	gz += kp * ez + ezInt;
	
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;
	
	norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;
    
    imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
	imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;
	imu_Angle.Yaw  += imu_data.GZ* 57.2957* cycle_T* 4;
	
}

void IMU_getEuleranAngles(void)
{
     MPU6050_GetData(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
	 imu_data.AX = ((float)Ax)/2048;
	 imu_data.AY = ((float)Ay)/2048;
     imu_data.AZ = ((float)Az)/2048;
	
     imu_data.GX = ((float)Gx)*0.001064;
     imu_data.GY = ((float)Gy)*0.001064;
     imu_data.GZ = ((float)Gz)*0.001064;
	 IMU_AHRSupdate(&imu_data);
	
}   
    






