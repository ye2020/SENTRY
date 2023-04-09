#ifndef __GIMBAL_BEHAVIOUR_H
#define __GIMBAL_BEHAVIOUR_H
#include "main.h"
#include "Task_Gimbal.h"
#include "Task_Fire.h"

#define PITCH_UP   0
#define PITCH_DOWN 1

#define RIGHT   0
#define LEFT 1

#define PITCH_UP_SPEED   500
#define PITCH_DOWN_SPEED -600
#define YAW_SPEED        500


#define PITCH_TOP_ANGLE     30
#define PITCH_BOTTOM_ANGLE  10

#define ENEMY_DISAPPEAR_TIMES  400		//��⵽������ʧ�Ĵ���

typedef enum
{
	Enemy_Appear=0,		//���ֵ���
	Enemy_Disappear=1,			//������ʧ
}VisionStatus_E;		//���Զ�����ģʽ����ʹ��

typedef enum
{
	AUTOMODE,			    //�Զ�ģʽ
	REMOTEMODE,				//ң��ģʽ
	FIREMODE,				//���ģʽ
}WorkStatus_E;

extern void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f);
extern void Gimbal_Stop(gimbal_control_t *gimbal_stop_f);  //��̨����
extern Fire_WorkStatus_e Return_Fire_Mode(void);  //����ģʽ
extern Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void);  //Ħ����ģʽ
#endif
