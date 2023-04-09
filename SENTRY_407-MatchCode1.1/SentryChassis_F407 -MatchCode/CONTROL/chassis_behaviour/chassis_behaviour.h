/**
  ******************************************************************************
  * @file       chassis_behaviour.c/h
  * @brief      ����״̬����
  ******************************************************************************
  */
#ifndef __CHASSIS_BEHAVIOUR_H
#define __CHASSIS_BEHAVIOUR_H
#include "main.h"
#include "Task_Chassis.h"


//typedef enum
//{
//  CHASSIS_ZERO_FORCE,                  //��������
//  CHASSIS_NO_MOVE,                     //���̱��ֲ���
//  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //�����������̸�����̨
//  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ�������е�������������µ��̵�yaw��pitch��roll�Ƕ�
//  CHASSIS_NO_FOLLOW_YAW,               //���̲�����Ƕȣ��Ƕ��ǿ����ģ���ǰ�����������ٶȻ�
//  CHASSIS_OPEN                         //ң������ֵ���Ա���ֱ�ӷ��͵�can������
//} chassis_behaviour_e;


extern void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f);

extern void Chassis_Stop(chassis_control_t *Chassis_Stop_f);
extern Fire_WorkStatus_e Return_Fire_Mode(void);  //����ģʽ
extern Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void);  //Ħ����ģʽ
#endif



