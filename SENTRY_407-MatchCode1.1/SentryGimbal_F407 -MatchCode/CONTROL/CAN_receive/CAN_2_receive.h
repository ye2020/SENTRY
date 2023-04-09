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
extern void CAN2_Chassis_RC_SetMsg(void);
/* CAN2 ���̰�������ݷ��� */
extern void CAN2_Chassis_MK_SetMsg(void);
/* CAN2 ���̰�yawλ�����ݷ��� */
extern void CAN2_Chassis_yaw_Position_SetMsg(void);
/* CAN2 ��̨�巢�� */
extern void CAN2_gimbal_SetMsg(u8 Initialize_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 Supply_flag, int16_t Sec_chassis_angle);
/* CAN2 yaw���� */
extern void CAN2_yaw_Setmsg(int16_t ESC_208);
//���ظ�yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void);

extern void CAN2_Enemy_status(int16_t ESC_208);
// ���ص�����ɫ
uint8_t  get_Enemy_status(void);
// ���شӵ��̻�ȡ������
int16_t  get_bullet_speed_from_chassis(void);
//ǹ����������
uint16_t get_shooter_cooling_limit(void);
//ǹ������
uint16_t get_shooter_cooling_heat(void);
//����״̬
uint8_t get_hurt_status(void);


#endif
