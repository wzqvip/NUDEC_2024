#ifndef DFROBOT_ICM42688_H
#define DFROBOT_ICM42688_H

#include "board.h"
#include <stdio.h>

#define DFRobot_ICM42688_I2C_L_ADDR 0x68 
#define DFRobot_ICM42688_I2C_H_ADDR 0x69
#define DFRobot_ICM42688_ID  0x47 

#define ICM42688_DEVICE_CONFIG          0x11
#define ICM42688_DRIVE_CONFIG           0x13

#define ICM42688_SIGNAL_PATH_RESET      0x4B

#define ICM42688_PWR_MGMT0              0x4E

#define ICM42688_INT_CONFIG             0x14
#define ICM42688_INT_STATUS             0x2D
#define ICM42688_INT_STATUS2            0x37
#define ICM42688_INT_STATUS3            0x38
#define ICM42688_INT_CONFIG0            0x63
#define ICM42688_INT_CONFIG1            0x64
#define ICM42688_INT_SOURCE0            0x65
#define ICM42688_INT_SOURCE1            0x66
#define ICM42688_INT_SOURCE3            0x68
#define ICM42688_INT_SOURCE4            0x69
#define ICM42688_INT_SOURCE6            0x4D
#define ICM42688_INT_SOURCE7            0x4E
#define ICM42688_INT_SOURCE8            0x4F
#define ICM42688_INT_SOURCE9            0x50
#define ICM42688_INT_SOURCE10           0x51

#define ICM42688_TEMP_DATA1             0x1D
#define ICM42688_TEMP_DATA0             0x1E
#define ICM42688_ACCEL_DATA_X1          0x1F
#define ICM42688_ACCEL_DATA_X0          0x20
#define ICM42688_ACCEL_DATA_Y1          0x21
#define ICM42688_ACCEL_DATA_Y0          0x22
#define ICM42688_ACCEL_DATA_Z1          0x23
#define ICM42688_ACCEL_DATA_Z0          0x24
#define ICM42688_GYRO_DATA_X1           0x25
#define ICM42688_GYRO_DATA_X0           0x26
#define ICM42688_GYRO_DATA_Y1           0x27
#define ICM42688_GYRO_DATA_Y0           0x28
#define ICM42688_GYRO_DATA_Z1           0x29
#define ICM42688_GYRO_DATA_Z0           0x30

#define ICM42688_TMST_FSYNCH            0x43
#define ICM42688_TMST_FSYNCL            0x44

#define ICM42688_GYRO_CONFIG_STATIC2    0x0B
#define ICM42688_GYRO_CONFIG_STATIC3    0x0C
#define ICM42688_GYRO_CONFIG_STATIC4    0x0D
#define ICM42688_GYRO_CONFIG_STATIC5    0x0E
#define ICM42688_GYRO_CONFIG_STATIC6    0x0F
#define ICM42688_GYRO_CONFIG_STATIC7    0x10
#define ICM42688_GYRO_CONFIG_STATIC8    0x11
#define ICM42688_GYRO_CONFIG_STATIC9    0x12
#define ICM42688_GYRO_CONFIG_STATIC10   0x13


    
#define ICM42688_GYRO_CONFIG0           0x4F
#define ICM42688_ACCEL_CONFIG0          0x50
#define ICM42688_GYRO_CONFIG1           0x51
#define ICM42688_GYRO_ACCEL_CONFIG0     0x52
#define ICM42688_ACCEL_CONFIG1          0x53
    
#define ICM42688_TMST_CONFIG            0x54
    

#define ICM42688_SMD_CONFIG             0x57

#define ICM42688_FIFO_CONFIG            0x16
#define ICM42688_FIFO_COUNTH            0x2E
#define ICM42688_FIFO_COUNTL            0x2F
#define ICM42688_FIFO_DATA              0x30
#define ICM42688_FIFO_CONFIG1           0x5F
#define ICM42688_FIFO_CONFIG2           0x60
#define ICM42688_FIFO_CONFIG3           0x61
#define ICM42688_FIFO_LOST_PKT0         0x6C
#define ICM42688_FIFO_LOST_PKT1         0x6D

#define ICM42688_FSYNC_CONFIG           0x62
    

    

#define ICM42688_SELF_TEST_CONFIG       0x70
#define ICM42688_WHO_AM_I               0x75
#define ICM42688_REG_BANK_SEL           0x76   
#define ICM42688_SENSOR_CONFIG0         0x03



#define ICM42688_XG_ST_DATA             0x5F
#define ICM42688_YG_ST_DATA             0x60
#define ICM42688_ZG_ST_DATA             0x61

#define ICM42688_TMSTVAL0               0x62
#define ICM42688_TMSTVAL1               0x63
#define ICM42688_TMSTVAL2               0x64

#define ICM42688_INTF_CONFIG0           0x4C
#define ICM42688_INTF_CONFIG1           0x4D
#define ICM42688_INTF_CONFIG4           0x7A
#define ICM42688_INTF_CONFIG5           0x7B
#define ICM42688_INTF_CONFIG6           0x7C

#define ICM42688_ACCEL_CONFIG_STATIC2   0x03
#define ICM42688_ACCEL_CONFIG_STATIC3   0x04
#define ICM42688_ACCEL_CONFIG_STATIC4   0x05

#define ICM42688_XA_ST_DATA             0x3B
#define ICM42688_YA_ST_DATA             0x3C
#define ICM42688_ZA_ST_DATA             0x3D

#define ICM42688_APEX_DATA0             0x31
#define ICM42688_APEX_DATA1             0x32
#define ICM42688_APEX_DATA2             0x33
#define ICM42688_APEX_DATA3             0x34
#define ICM42688_APEX_DATA4             0x35
#define ICM42688_APEX_DATA5             0x36
#define ICM42688_APEX_CONFIG0           0x56
#define ICM42688_APEX_CONFIG1           0x40 
#define ICM42688_APEX_CONFIG2           0x41
#define ICM42688_APEX_CONFIG3           0x42
#define ICM42688_APEX_CONFIG4           0x43
#define ICM42688_APEX_CONFIG5           0x44
#define ICM42688_APEX_CONFIG6           0x45
#define ICM42688_APEX_CONFIG7           0x46
#define ICM42688_APEX_CONFIG8           0x47
#define ICM42688_APEX_CONFIG9           0x48

#define ICM42688_ACCEL_WOM_X_THR        0x4A
#define ICM42688_ACCEL_WOM_Y_THR        0x4B
#define ICM42688_ACCEL_WOM_Z_THR        0x4C

#define ICM42688_OFFSET_USER0           0x77
#define ICM42688_OFFSET_USER1           0x78
#define ICM42688_OFFSET_USER2           0x79
#define ICM42688_OFFSET_USER3           0x7A
#define ICM42688_OFFSET_USER4           0x7B
#define ICM42688_OFFSET_USER5           0x7C
#define ICM42688_OFFSET_USER6           0x7D
#define ICM42688_OFFSET_USER7           0x7E
#define ICM42688_OFFSET_USER8           0x7F

#define ICM42688_STEP_DET_INT           1<<5
#define ICM42688_STEP_CNT_OVF_INT       1<<4
#define ICM42688_TILT_DET_INT           1<<3
#define ICM42688_WAKE_INT               1<<2
#define ICM42688_SLEEP_INT              1<<1
#define ICM42688_TAP_DET_INT            1

#define ICM42688_SMD_INT                1<<3
#define ICM42688_WOM_Z_INT              1<<2
#define ICM42688_WOM_Y_INT              1<<1
#define ICM42688_WOM_X_INT              1

#define ICM42688_STATUS_WALK 1
#define ICM42688_STATUS_RUN 2

// class DFRobot_ICM42688

  #define ERR_OK             0      ///< No error
  #define ERR_DATA_BUS      -1      ///< Data bus error
  #define ERR_IC_VERSION    -2      ///< The chip version not match
  
  #define GYRO     0
  #define ACCEL    1
  #define ALL      5



  #define TMST_DEFAULT_CONFIG_START  0x23
  #define TMST_VALUE_DIS             0<<4
  #define TMST_VALUE_EN              1<<4
  #define TMST_RES_EN_DIS            0<<3 
  #define TMST_RES_EN                1<<3
  #define TMST_FSYNC_EN              1<<1
  #define TMST_FSYNC_DIS             0<<1
  #define TMST_DELTA_EN              0<<2
  #define TMST_DELTA_DIS             1<<2
  #define TMST_EN                    1
  #define TMST_DIS                   0

  #define X_AXIS   0
  #define Y_AXIS   2
  #define Z_AXIS   4

  #define X_AXIS_WOM   1
  #define Y_AXIS_WOM   2
  #define Z_AXIS_WOM   4

  #define ODR_32KHZ         1
  #define ODR_16KHZ         2
  #define ODR_8KHZ          3
  #define ODR_4KHZ          4
  #define ODR_2KHZ          5
  #define ODR_1KHZ          6
  #define ODR_200HZ         7
  #define ODR_100HZ         8
  #define ODR_50HZ          9
  #define ODR_25KHZ         10
  #define ODR_12_5KHZ       11
  #define ODR_6_25KHZ       12
  #define ODR_3_125HZ       13
  #define ODR_1_5625HZ      14
  #define ODR_500HZ         15

  #define FSR_0             0
  #define FSR_1             1
  #define FSR_2             2
  #define FSR_3             3
  #define FSR_4             4
  #define FSR_5             5
  #define FSR_6             6
  #define FSR_7             7

  #define LP_MODE_ONLY_ACCEL  2
  #define LN_MODE  3
  #define STANDBY_MODE_ONLY_GYRO 1 
  #define OFF_MODE   0

  #define TAP_SINGLE 8
  #define TAP_DOUBLE 16

typedef struct {
    uint8_t gyroODR;
    uint8_t gyroFsSel;
} GyroConfig0;

typedef struct {
    uint8_t accelODR;
    uint8_t accelFsSel;
} AccelConfig0;

typedef struct {
    uint8_t tempDis;
    uint8_t gyroMode;
    uint8_t accelMode;
} sPWRMgmt0_t;



void I2C_WriteReg(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, uint8_t length);
void I2C_ReadReg(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, uint8_t length);


void ICM42688_init();
void DFRobot_ICM42688_begin(uint8_t i2cAddr);
bool DFRobot_ICM42688_setODRAndFSR(uint8_t who, uint8_t ODR, uint8_t FSR);
void DFRobot_ICM42688_startTempMeasure();
void DFRobot_ICM42688_startGyroMeasure(uint8_t mode);
void DFRobot_ICM42688_startAccelMeasure(uint8_t mode);
float DFRobot_ICM42688_getTemperature();
float DFRobot_ICM42688_getAccelDataX();
float DFRobot_ICM42688_getAccelDataY();
float DFRobot_ICM42688_getAccelDataZ();
float DFRobot_ICM42688_getGyroDataX();
float DFRobot_ICM42688_getGyroDataY();
float DFRobot_ICM42688_getGyroDataZ();

#endif
