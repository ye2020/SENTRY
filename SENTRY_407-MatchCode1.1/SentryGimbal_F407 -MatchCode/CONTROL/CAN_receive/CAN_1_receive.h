/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1接收和发送电机数据
  ******************************************************************************
  */

#ifndef __CAN_1_RECEIVE_H
#define __CAN_1_RECEIVE_H
#include "main.h"

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

    CAN_TRIGGER_MOTORA_ID = 0x205,
    CAN_TRIGGER_MOTORB_ID = 0x206,
    CAN_PIT_MOTOR_ID = 0x207,
    CAN_YAW_MOTOR_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_ENEMY_COLOR_ID = 0x3ff,
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t position;
    int16_t speed;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_position;


    int16_t angle;
    int16_t speed_filt;
    int16_t first_Flag;
    int32_t yaw_angle;
    int32_t pitch_angle;
    int32_t actual_Position;  //真实位置
} motor_measure_t;


/*--------------------变量-----------------------*/
/* 底盘板CAN1发送到云台Y电机和供弹电机 | Y轴电机是205，P轴电机是206，供弹电机是207（P轴电机0x206没用到） */
extern void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern  motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *Get_Fire_MotorA_Measure_Point(void);
extern const motor_measure_t *Get_Fire_MotorB_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i);

#endif
