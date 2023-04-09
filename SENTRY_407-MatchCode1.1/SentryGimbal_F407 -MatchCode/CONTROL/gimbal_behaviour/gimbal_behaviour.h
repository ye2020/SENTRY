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

#define ENEMY_DISAPPEAR_TIMES  400		//检测到敌人消失的次数

typedef enum
{
	Enemy_Appear=0,		//发现敌人
	Enemy_Disappear=1,			//敌人消失
}VisionStatus_E;		//在自动控制模式里面使用

typedef enum
{
	AUTOMODE,			    //自动模式
	REMOTEMODE,				//遥控模式
	FIREMODE,				//火控模式
}WorkStatus_E;

extern void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f);
extern void Gimbal_Stop(gimbal_control_t *gimbal_stop_f);  //云台无力
extern Fire_WorkStatus_e Return_Fire_Mode(void);  //发弹模式
extern Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void);  //摩擦轮模式
#endif
