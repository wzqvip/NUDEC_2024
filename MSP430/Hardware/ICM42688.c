#include "ICM42688.h"
#include "IOI2C.h"

static uint8_t _deviceAddr;
static GyroConfig0 gyroConfig0;
static AccelConfig0 accelConfig0;
static sPWRMgmt0_t PWRMgmt0;
static float _gyroRange;
static float _accelRange;
static bool FIFOMode = false;


uint8_t _r, _g, _b;
uint8_t _mode;
uint8_t _tapNum ;
uint8_t _tapAxis;
uint8_t _tapDir ;
int16_t _accelX;
int16_t _accelY;
int16_t _accelZ;
int16_t _gyroX;
int16_t _gyroZ;
int16_t _gyroY;
int8_t _temp;
int8_t _INTPin;


void ICM42688_init() {
    uint8_t ret;

    DFRobot_ICM42688_begin(DFRobot_ICM42688_I2C_H_ADDR);

    while ((ret = DFRobot_ICM42688_setODRAndFSR(GYRO, ODR_1KHZ, FSR_0)) == false) {
        // if (ret == 255) {
        //     printf("bus data access error");
        // } else {
        //     printf("Chip versions do not match:");
        //     printf("ICM42688 version is %d", ret);
        // }
        printf("ICM42688 set gyro ODR and FSR failed");
        delay_ms(1000);
    }
    printf("ICM42688 begin success!!!");

    if (!DFRobot_ICM42688_setODRAndFSR(GYRO, ODR_1KHZ, FSR_0)) {
        printf("Gyroscope configuration success");
    } else {
        printf("Gyroscope configuration failed");
    }

    if (!DFRobot_ICM42688_setODRAndFSR(ACCEL, ODR_500HZ, FSR_0)) {
        printf("Accelerometer configuration success");
    } else {
        printf("Accelerometer configuration failed");
    }

    DFRobot_ICM42688_startTempMeasure();
    DFRobot_ICM42688_startGyroMeasure(LN_MODE);
    DFRobot_ICM42688_startAccelMeasure(LN_MODE);
}

void DFRobot_ICM42688_begin(uint8_t i2cAddr) {
    _deviceAddr = i2cAddr;
}

bool DFRobot_ICM42688_setODRAndFSR(uint8_t who, uint8_t ODR, uint8_t FSR) {
    bool ret = true;
    uint8_t bank = 0;
    i2cWrite(_deviceAddr, ICM42688_REG_BANK_SEL, 1, &bank);
    if (who == GYRO) {
        if (ODR > ODR_12_5KHZ || FSR > FSR_7) {
            ret = false;
        } else {
            gyroConfig0.gyroODR = ODR;
            gyroConfig0.gyroFsSel = FSR;
            i2cWrite(_deviceAddr, ICM42688_GYRO_CONFIG0, 1, (uint8_t *)&gyroConfig0);
            switch (FSR) {
                case FSR_0:
                    _gyroRange = 4000 / 65535.0;
                    break;
                case FSR_1:
                    _gyroRange = 2000 / 65535.0;
                    break;
                case FSR_2:
                    _gyroRange = 1000 / 65535.0;
                    break;
                case FSR_3:
                    _gyroRange = 500 / 65535.0;
                    break;
                case FSR_4:
                    _gyroRange = 250 / 65535.0;
                    break;
                case FSR_5:
                    _gyroRange = 125 / 65535.0;
                    break;
                case FSR_6:
                    _gyroRange = 62.5 / 65535.0;
                    break;
                case FSR_7:
                    _gyroRange = 31.25 / 65535.0;
                    break;
            }
        }
    } else if (who == ACCEL) {
        if (ODR > ODR_500HZ || FSR > FSR_3) {
            ret = false;
        } else {
            accelConfig0.accelODR = ODR;
            accelConfig0.accelFsSel = FSR;
            i2cWrite(_deviceAddr, ICM42688_ACCEL_CONFIG0, 1, (uint8_t *)&accelConfig0);
            switch (FSR) {
                case FSR_0:
                    _accelRange = 0.488f;
                    break;
                case FSR_1:
                    _accelRange = 0.244f;
                    break;
                case FSR_2:
                    _accelRange = 0.122f;
                    break;
                case FSR_3:
                    _accelRange = 0.061f;
                    break;
            }
        }
    }
    return ret;
}

void DFRobot_ICM42688_startTempMeasure() {
    PWRMgmt0.tempDis = 0;
    uint8_t bank = 0;
    i2cWrite(_deviceAddr, ICM42688_REG_BANK_SEL, 1, &bank);
    i2cWrite(_deviceAddr, ICM42688_PWR_MGMT0, 1, (uint8_t *)&PWRMgmt0);
}

void DFRobot_ICM42688_startGyroMeasure(uint8_t mode) {
    PWRMgmt0.gyroMode = mode;
    uint8_t bank = 0;
    i2cWrite(_deviceAddr, ICM42688_REG_BANK_SEL, 1, &bank);
    i2cWrite(_deviceAddr, ICM42688_PWR_MGMT0, 1, (uint8_t *)&PWRMgmt0);
}

void DFRobot_ICM42688_startAccelMeasure(uint8_t mode) {
    PWRMgmt0.accelMode = mode;
    uint8_t bank = 0;
    i2cWrite(_deviceAddr, ICM42688_REG_BANK_SEL, 1, &bank);
    i2cWrite(_deviceAddr, ICM42688_PWR_MGMT0, 1, (uint8_t *)&PWRMgmt0);
}

float DFRobot_ICM42688_getTemperature() {
    float value;
    if (FIFOMode) {
        value = (_temp / 2.07) + 25;
    } else {
        uint8_t data[2];
        int16_t value2;
        i2cRead(_deviceAddr, ICM42688_TEMP_DATA1, 2, data);
        value2 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value2 / 132.48 + 25;
    }
    return value;
}

float DFRobot_ICM42688_getAccelDataX() {
    float value;
    if (FIFOMode) {
        value = _accelX;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_ACCEL_DATA_X1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _accelRange;
}

float DFRobot_ICM42688_getAccelDataY() {
    float value;
    if (FIFOMode) {
        value = _accelY;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_ACCEL_DATA_Y1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _accelRange;
}

float DFRobot_ICM42688_getAccelDataZ() {
    float value;
    if (FIFOMode) {
        value = _accelZ;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_ACCEL_DATA_Z1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _accelRange;
}

float DFRobot_ICM42688_getGyroDataX() {
    float value;
    if (FIFOMode) {
        value = _gyroX;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_GYRO_DATA_X1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _gyroRange;
}

float DFRobot_ICM42688_getGyroDataY() {
    float value;
    if (FIFOMode) {
        value = _gyroY;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_GYRO_DATA_Y1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _gyroRange;
}

float DFRobot_ICM42688_getGyroDataZ() {
    float value;
    if (FIFOMode) {
        value = _gyroZ;
    } else {
        uint8_t data[2];
        i2cRead(_deviceAddr, ICM42688_GYRO_DATA_Z1, 2, data);
        int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        value = value1;
    }
    return value * _gyroRange;
}