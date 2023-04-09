/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      ͨ��CAN2���а��ͨѶ
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
	Enemy_Appear=0,		//���ֵ���
	Enemy_Disappear=1,			//������ʧ
}VisionStatus_E;		//���Զ�����ģʽ����ʹ��


typedef struct
{
    int8_t photoelectric_zero;  //Y����ֵ����־

    int8_t Gimbal_supply_flag;//����״̬��־λ(0:����״̬�����벹��״̬  1:��̨90��ת����  2:����ָ��λ��  3:������   4������ģʽ����)
    int8_t Gimbal_all_flag;//���͸����̵ĳ�ʼ���ɹ���־

    float angle;              //��̨��ǰ�Ƕ�
    float last_angle;         //��̨����Ƕ�

    int16_t output;

} gimbal_yaw_receive_t;


/* CAN2 ���̰�ң�������ݷ��� */
extern void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can2_RC_send);
/* CAN2 ���̰�������ݷ��� */
extern void CAN2_Chassis_MK_SetMsg(const RC_ctrl_t *can2_MK_send);
/* CAN2 ���̰�yawλ�����ݷ��� */
extern void CAN2_Chassis_yaw_Position_SetMsg(const motor_measure_t *yaw_send);
/* CAN2 ��̨�巢�� */
extern void CAN2_gimbal_SetMsg(u8 INITIALIZE_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 GIMBAL_SUPPLY_FLAG, int16_t Sec_chassis_angle);
/*  can2������̨ʶ��ĵ���״̬ */
static void CAN2_gimbal_receive(CanRxMsg *rx_message);

VisionStatus_E  get_Enemy_status(void);

/*  ���͵�����ɫ ������ ���������ޡ�ǹ����������̨ */
void CAN2_Enemy_color_SetMsg(int16_t ESC_208 , int16_t bullet_speed , uint16_t shooter_cooling_limit ,  uint16_t  shooter_cooling_heat,uint8_t hurt_status);


#endif
