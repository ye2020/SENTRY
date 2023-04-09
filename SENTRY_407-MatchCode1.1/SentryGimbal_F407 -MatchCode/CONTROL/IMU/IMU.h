#ifndef __IMU_H
#define __IMU_H
#include "main.h"

/*
SendBuff[0] = 0xfe;
SendBuff[1] = send_1 >> 8;
SendBuff[2] = send_1;
SendBuff[3] = send_2 >> 8;
SendBuff[4] = send_2;
SendBuff[5] = send_3 >> 8;
SendBuff[6] = send_3;
SendBuff[7] = send_4 >> 8;
SendBuff[8] = send_4;
SendBuff[9] = send_5 >> 8;
SendBuff[10] = send_5;
SendBuff[11] = send_6 >> 8;
SendBuff[12] = send_6;
SendBuff[13] = 0xee;
*/


typedef struct
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�

    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�

    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�

    float yaw_angle;
    float pitch_angle;
    float roll_angle;
} IMU_Data;


extern IMU_Data IMU_t;


void IMU_control_init(void);

void IMU_Data_Deal(void);
void BMI160_Zero_Correct(void);
void Gyro_usart_iwdg(void);

#endif
