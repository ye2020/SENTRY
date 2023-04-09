#ifndef __MAIN_H
#define __MAIN_H



//ϵͳͷ�ļ�
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

//SYSTEM
//#include "SysInit.h"//��ʼ��ͷ�ļ�
#include "sys.h"
#include "delay.h"

//HARDWARE
//#include "can.h"
//#include "iwdg.h"
//#include "key.h"
#include "led.h"
//#include "photoelectric.h"
//#include "pid.h"
//#include "time.h"
//#include "usart.h"
//#include "pwm.h"

//REFEREE

//CONTROL
//#include "CAN_1_Receive.h"
//#include "CAN_2_Receive.h"
//#include "chassis_behaviour.h"
//#include "gimbal_behaviour.h"
//#include "IMU.h"
#include "filter.h"
#include "maths.h"
//#include "RemoteControl.h"
//#include "rmmotor.h"

//Task
//#include "Task_Start.h"
//#include "Task_Remote.h"
//#include "Task_Chassis.h"
//#include "Task_Gimbal.h"
//#include "Task_Fire.h"
//#include "Task_Detect.h"
//#include "Task_Safecheck.h"
//#include "Task_Test.h"



#define RC_NVIC   0  //ң����   
#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5


/*ģ�鹤������*/
//#define watch_dog                //�������Ź�
#define gimbal_work              //��̨����
#define chassis_work             //���̹���
#define fire_work               //�䵯ģʽ���� (��Ħ����)
//#define power_limit             //������������
//#define double_gimbal           //ʹ��˫��̨
//#define super_capacitor         //ʹ�ó�������
//#define chassis_exclusive_use   //�������ʹ��5

#define IMU_BMI160  0             //�����ǹ��� 1Ϊʹ��		�����������ô��ڶ� ����Ϊ��λ��ͨ�� Ҫʹ����������������ڣ�

#define watch_dog		0							//���Ź����� 1Ϊʹ��

#define yaw_angle_limit 0				//yaw ��Ƕ�����			0 -> ������  1 -> ����

#define pitch_angle_position		0	// p��ʹ�ñ�����            0 -> ��ʹ��  1 -> ʹ��
/* ���ذ����� */
#define board_gimbal       //board_gimbal//board_chassis     //��������̨�廹�ǵ��̰�


/*****�������˵���̨��ֵ(����õ�������У׼��Ŀǰ2020��������P���õ���������ʼ��)******/
#define Pitch_Middle_Angle  200 //�ڱ�50   -8  -82     42  -32  74
#define Pitch_UP_Angle      190  //	

/*****�����Ƿ����������*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P����ٶ�
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y����ٶ�       
#define YAW_POSTION_OUTPUT_FLAG   (1)
#define YAW_ANGLE_FLAG            (1)  //������λ�ö�Y��Ƕȵ�Ӱ��
#define YAW_SPEED_OUTPUT_FLAG     (-1)  //���ٶȻ�Y����ٶȷ���
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


///*****������ͨ�������ѡ��PID����*********/
//#define test_short_focus  1      //�̽����
//#define test_long_focus   0      //�������
//#define test_industry     0      //��ҵ���


/************��� ������*���ٱ� ***************/
#define YAW_RATIO      (5*19)         //Yaw��
#define PITCH_RATIO		 (1.5*19)       //Pitch��
#define PITCH_GR			 (1.5)					//pitch������
#define Sec_YAW_RATIO  (3*1)          //��Yaw��


/* ���̵���ƶ��ٶ��趨 */
#define M3508_MAX_OUTPUT_CURRENT  5000   //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006������������

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508����������
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508���������

/*************���ٵ��������������������������**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21


//����ϵͳ�汾Ԥ����
#define		JUDGE_SZ		21
#define		JUDGE_ACE		20
#define		JUDGE_VERSION	JUDGE_SZ



#endif
