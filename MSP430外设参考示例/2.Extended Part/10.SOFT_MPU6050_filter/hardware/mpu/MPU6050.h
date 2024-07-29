#ifndef __MPU6050_H
#define __MPU6050_H


#include "A_include.h"



#define SDA_OUT()  {DL_GPIO_initDigitalOutput(MYIIC_MYSDA_IOMUX);           \
                    DL_GPIO_enableOutput(MYIIC_MYSDA_PORT, MYIIC_MYSDA_PIN);}
#define SDA_IN()                 DL_GPIO_initDigitalInput(MYIIC_MYSDA_IOMUX)
#define SCL(x)              ( (x) ? (DL_GPIO_setPins(MYIIC_MYSCL_PORT,MYIIC_MYSCL_PIN)) : (DL_GPIO_clearPins(MYIIC_MYSCL_PORT,MYIIC_MYSCL_PIN)) )
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN)) : (DL_GPIO_clearPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN)) )
#define SDA_GET()      (((DL_GPIO_readPins(MYIIC_MYSDA_PORT,MYIIC_MYSDA_PIN) & MYIIC_MYSDA_PIN ) > 0 ) ? 1 : 0 )

#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75


typedef struct{
float AX;
float AY;
float AZ;
float GX;
float GY;
float GZ;
}param_imu;

typedef struct{
float Pitch;
float Roll;
float Yaw;
}param_Angle;

extern param_Angle imu_Angle;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);


void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void IMU_getEuleranAngles(void);

#endif
