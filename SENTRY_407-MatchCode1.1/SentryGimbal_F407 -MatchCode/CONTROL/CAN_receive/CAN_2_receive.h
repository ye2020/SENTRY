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
extern void CAN2_Chassis_RC_SetMsg(void);
/* CAN2 底盘板键盘数据发送 */
extern void CAN2_Chassis_MK_SetMsg(void);
/* CAN2 底盘板yaw位置数据发送 */
extern void CAN2_Chassis_yaw_Position_SetMsg(void);
/* CAN2 云台板发送 */
extern void CAN2_gimbal_SetMsg(u8 Initialize_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 Supply_flag, int16_t Sec_chassis_angle);
/* CAN2 yaw控制 */
extern void CAN2_yaw_Setmsg(int16_t ESC_208);
//返回副yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void);

extern void CAN2_Enemy_status(int16_t ESC_208);
// 返回敌人颜色
uint8_t  get_Enemy_status(void);
// 返回从底盘获取的射速
int16_t  get_bullet_speed_from_chassis(void);
//枪口热量上限
uint16_t get_shooter_cooling_limit(void);
//枪口热量
uint16_t get_shooter_cooling_heat(void);
//受伤状态
uint8_t get_hurt_status(void);


#endif
