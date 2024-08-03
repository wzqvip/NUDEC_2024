#ifndef __Sever_H_
#define __Sever_H_

#include "A_include.h"


#define Servo_Delta 150           //�������ת���Ĳ�ֵ�������ͺţ����˺Ͷ���۳��й�
#define Servo_Center_Mid 0      //���ֱ����ֵ��
#define Servo_Left_Max   (Servo_Center_Mid+Servo_Delta)      //�����ת����ֵ
#define Servo_Right_Min  (Servo_Center_Mid-Servo_Delta)      //�����ת����ֵ����ֵ��������÷�ʽ�йأ���ʽ


void Sever_Init(void);
void Servo_Ctrl (int duty);

#endif

