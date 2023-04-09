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
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，

    float Temp;     //转换成实际的温度，单位为摄氏度

    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度

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
