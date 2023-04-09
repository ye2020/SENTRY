#ifndef __MAIN_H
#define __MAIN_H


//系统头文件
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




#define RC_NVIC       0  //遥控器   
#define CAN1_NVIC     4
#define CAN2_NVIC     4
#define TIM3_NVIC     5
#define TIM6_NVIC     4
#define SPI5_RX_NVIC  5
#define MPU_INT_NVIC  5


/*模块工作属性*/
//#define watch_dog                //启动看门狗
#define gimbal_work              //云台工作
#define chassis_work             //底盘工作
#define fire_work               //射弹模式开启 (开拨弹轮)
//#define power_limit             //启动功率限制
//#define double_gimbal           //使用双云台
//#define TEST_MODE                //测试模式


//#define IMU_BMI160  1             //陀螺仪工作 1为使用
#define chassis_using		 0    //底盘使用   0 -> 不使用   1 -> 使用

/* 主控板类型 */
#define board_chassis       //board_gimbal//board_chassis     //板子是云台板还是底盘板


/*****各机器人的云台中值(如果用到编码器校准，目前2020赛季步兵P轴用到编码器初始化)******/
#define Pitch_Middle_Angle  190  //步兵1:6052   步兵2:1530
#define Pitch_UP_Angle      190  //步兵1:6052   步兵2:1530	


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
#define CHASSIS_RATIO  (1*19)					//底盘电机减速比
#define Sec_YAW_RATIO  (3*1)          //副Yaw轴

/* 底盘电机移动速度设定 */ 
#define M3508_MAX_OUTPUT_CURRENT  5000   //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006电机最大电流输出

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508最大电流输入
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508最大电流输出

/*************减速电机启动电流补偿（快速启动）**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21

/*****键盘鼠标遥控速度设置******/ 
#define MOUSE_YAW_SPEED        0.021    //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED 		0.08     //鼠标pitch轴速度增益0.13
#define RC_YAW_SPEED          0.0026   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026   //遥控器pitch轴速度增益 0.0026


//裁判系统版本预编译
#define		JUDGE_SZ		21
#define		JUDGE_ACE		20
#define		JUDGE_VERSION	JUDGE_SZ


#endif
