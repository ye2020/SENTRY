/**
  ******************************************************************************
  * @file       chassis_behaviour.c/h
  * @brief      底盘状态机。
  ******************************************************************************
  */
#ifndef __CHASSIS_BEHAVIOUR_H
#define __CHASSIS_BEHAVIOUR_H
#include "main.h"
#include "Task_Chassis.h"


//typedef enum
//{
//  CHASSIS_ZERO_FORCE,                  //底盘无力
//  CHASSIS_NO_MOVE,                     //底盘保持不动
//  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常步兵底盘跟随云台
//  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度
//  CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随角度，角度是开环的，但前后左右是有速度环
//  CHASSIS_OPEN                         //遥控器的值乘以比例直接发送到can总线上
//} chassis_behaviour_e;


extern void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f);

extern void Chassis_Stop(chassis_control_t *Chassis_Stop_f);
extern Fire_WorkStatus_e Return_Fire_Mode(void);  //发弹模式
extern Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void);  //摩擦轮模式
#endif



