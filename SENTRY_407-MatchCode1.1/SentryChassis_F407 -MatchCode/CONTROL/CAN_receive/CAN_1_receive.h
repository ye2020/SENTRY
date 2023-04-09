/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1接收和发送电机数据
  ******************************************************************************
  */

#ifndef __CAN_1_RECEIVE_H
#define __CAN_1_RECEIVE_H
#include "main.h"
#include "RemoteControl.h"
//#define CHASSIS_CAN CAN2
//#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M3508_MOTOR1_ID = 0x201,
    CAN_M3508_MOTOR2_ID = 0x202,
    CAN_M3508_MOTOR3_ID = 0x203,
    CAN_M3508_MOTOR4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t position;
    int16_t  speed;
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_position;

		int16_t  real_round;
    int16_t speed_filt;
    int16_t first_Flag;
    int32_t actual_Position;
		int16_t output;
} motor_measure_t;


//返回yaw电机变量地址，通过指针方式获取原始数据
extern motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Fire_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);


/* CAN1底盘板发送到底盘电机 */
extern void CAN1_Chassis_SetMsg(int16_t ESC_201 , int16_t ESC_202);

#endif
