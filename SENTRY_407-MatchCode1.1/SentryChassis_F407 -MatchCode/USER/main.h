#ifndef __MAIN_H
#define __MAIN_H


//ϵͳͷ�ļ�
#include "stm32f4xx.h" 
#include "stm32f4xx_rng.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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
//#include "RefereeDeal.h"

//OPERATION
#include "filter.h"
#include "maths.h"
//#include "rmmotor.h"

//CONTROL
//#include "CAN_1_Receive.h"
//#include "CAN_2_Receive.h"
//#include "chassis_behaviour.h"
//#include "RemoteControl.h"
//#include "Capacitor_control.h"
//#include "Double_Gimbal.h"

//Task
//#include "Task_Start.h"
//#include "Task_Chassis.h"
//#include "Task_Fire.h"
//#include "Task_Detect.h"
//#include "Task_Safecheck.h"
//#include "Task_Test.h"




#define RC_NVIC       0  //ң����   
#define CAN1_NVIC     4
#define CAN2_NVIC     4
#define TIM3_NVIC     5
#define TIM6_NVIC     4
#define SPI5_RX_NVIC  5
#define MPU_INT_NVIC  5


/*ģ�鹤������*/
//#define watch_dog                //�������Ź�
#define gimbal_work              //��̨����
#define chassis_work             //���̹���
#define fire_work               //�䵯ģʽ���� (��������)
//#define power_limit             //������������
//#define double_gimbal           //ʹ��˫��̨
//#define TEST_MODE                //����ģʽ


//#define IMU_BMI160  1             //�����ǹ��� 1Ϊʹ��
#define chassis_using		 0    //����ʹ��   0 -> ��ʹ��   1 -> ʹ��

/* ���ذ����� */
#define board_chassis       //board_gimbal//board_chassis     //��������̨�廹�ǵ��̰�


/*****�������˵���̨��ֵ(����õ�������У׼��Ŀǰ2020��������P���õ���������ʼ��)******/
#define Pitch_Middle_Angle  190  //����1:6052   ����2:1530
#define Pitch_UP_Angle      190  //����1:6052   ����2:1530	


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
#define CHASSIS_RATIO  (1*19)					//���̵�����ٱ�
#define Sec_YAW_RATIO  (3*1)          //��Yaw��

/* ���̵���ƶ��ٶ��趨 */ 
#define M3508_MAX_OUTPUT_CURRENT  5000   //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006������������

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508����������
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508���������

/*************���ٵ��������������������������**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21

/*****�������ң���ٶ�����******/ 
#define MOUSE_YAW_SPEED        0.021    //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 		0.08     //���pitch���ٶ�����0.13
#define RC_YAW_SPEED          0.0026   //ң����yaw���ٶ�����
#define RC_PITCH_SPEED        0.0026   //ң����pitch���ٶ����� 0.0026


//����ϵͳ�汾Ԥ����
#define		JUDGE_SZ		21
#define		JUDGE_ACE		20
#define		JUDGE_VERSION	JUDGE_SZ


#endif
