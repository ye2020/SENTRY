/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      通过CAN2进行板间通讯
  ******************************************************************************
  */

#ifndef __CAN_2_RECEIVE_H
#define __CAN_2_RECEIVE_H
#include "main.h"
#include "CAN_1_Receive.h"
#include "pid.h"
#include "RemoteControl.h"



typedef enum
{
	Enemy_Appear=0,		//发现敌人
	Enemy_Disappear=1,			//敌人消失
}VisionStatus_E;		//在自动控制模式里面使用


typedef struct
{
    int8_t photoelectric_zero;  //Y轴中值光电标志

    int8_t Gimbal_supply_flag;//补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置  3:归中中   4：补给模式结束)
    int8_t Gimbal_all_flag;//发送给底盘的初始化成功标志

    float angle;              //云台当前角度
    float last_angle;         //云台保存角度

    int16_t output;

} gimbal_yaw_receive_t;


/* CAN2 底盘板遥控器数据发送 */
extern void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can2_RC_send);
/* CAN2 底盘板键盘数据发送 */
extern void CAN2_Chassis_MK_SetMsg(const RC_ctrl_t *can2_MK_send);
/* CAN2 底盘板yaw位置数据发送 */
extern void CAN2_Chassis_yaw_Position_SetMsg(const motor_measure_t *yaw_send);
/* CAN2 云台板发送 */
extern void CAN2_gimbal_SetMsg(u8 INITIALIZE_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 GIMBAL_SUPPLY_FLAG, int16_t Sec_chassis_angle);
/*  can2接收云台识别的敌人状态 */
static void CAN2_gimbal_receive(CanRxMsg *rx_message);

VisionStatus_E  get_Enemy_status(void);

/*  发送敌人颜色 、射速 、热量上限、枪口热量给云台 */
void CAN2_Enemy_color_SetMsg(int16_t ESC_208 , int16_t bullet_speed , uint16_t shooter_cooling_limit ,  uint16_t  shooter_cooling_heat,uint8_t hurt_status);


#endif
