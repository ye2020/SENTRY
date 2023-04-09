#ifndef __MAIN_H
#define __MAIN_H



//系统头文件
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

//SYSTEM
//#include "SysInit.h"//初始化头文件
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



#define RC_NVIC   0  //遥控器   
#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5


/*模块工作属性*/
//#define watch_dog                //启动看门狗
#define gimbal_work              //云台工作
#define chassis_work             //底盘工作
#define fire_work               //射弹模式开启 (开摩擦轮)
//#define power_limit             //启动功率限制
//#define double_gimbal           //使用双云台
//#define super_capacitor         //使用超级电容
//#define chassis_exclusive_use   //底盘裸机使用5

#define IMU_BMI160  0             //陀螺仪工作 1为使用		（陀螺仪所用串口二 已用为上位机通信 要使用陀螺仪需更换串口）

#define watch_dog		0							//看门狗工作 1为使用

#define yaw_angle_limit 0				//yaw 轴角度限制			0 -> 不限制  1 -> 限制

#define pitch_angle_position		0	// p轴使用编码器            0 -> 不使用  1 -> 使用
/* 主控板类型 */
#define board_gimbal       //board_gimbal//board_chassis     //板子是云台板还是底盘板


/*****各机器人的云台中值(如果用到编码器校准，目前2020赛季步兵P轴用到编码器初始化)******/
#define Pitch_Middle_Angle  200 //哨兵50   -8  -82     42  -32  74
#define Pitch_UP_Angle      190  //	

/*****陀螺仪方向参数配置*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P轴角速度
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y轴角速度       
#define YAW_POSTION_OUTPUT_FLAG   (1)
#define YAW_ANGLE_FLAG            (1)  //陀螺仪位置对Y轴角度的影响
#define YAW_SPEED_OUTPUT_FLAG     (-1)  //纯速度环Y电机速度方向
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


///*****测试普通自瞄相机选择PID参数*********/
//#define test_short_focus  1      //短焦相机
//#define test_long_focus   0      //长焦相机
//#define test_industry     0      //工业相机


/************电机 传动比*减速比 ***************/
#define YAW_RATIO      (5*19)         //Yaw轴
#define PITCH_RATIO		 (1.5*19)       //Pitch轴
#define PITCH_GR			 (1.5)					//pitch传动比
#define Sec_YAW_RATIO  (3*1)          //副Yaw轴


/* 底盘电机移动速度设定 */
#define M3508_MAX_OUTPUT_CURRENT  5000   //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006电机最大电流输出

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508最大电流输入
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508最大电流输出

/*************减速电机启动电流补偿（快速启动）**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21


//裁判系统版本预编译
#define		JUDGE_SZ		21
#define		JUDGE_ACE		20
#define		JUDGE_VERSION	JUDGE_SZ



#endif
