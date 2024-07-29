#ifndef __Sever_H_
#define __Sever_H_

#include "A_include.h"


#define Servo_Delta 150           //舵机左右转动的差值，与舵机型号，拉杆和舵机臂长有关
#define Servo_Center_Mid 0      //舵机直行中值，
#define Servo_Left_Max   (Servo_Center_Mid+Servo_Delta)      //舵机左转极限值
#define Servo_Right_Min  (Servo_Center_Mid-Servo_Delta)      //舵机右转极限值，此值跟舵机放置方式有关，立式


void Sever_Init(void);
void Servo_Ctrl (int duty);

#endif

