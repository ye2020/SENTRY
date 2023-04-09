/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1���պͷ��͵������
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

//rm���ͳһ���ݽṹ��
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


//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Fire_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);


/* CAN1���̰巢�͵����̵�� */
extern void CAN1_Chassis_SetMsg(int16_t ESC_201 , int16_t ESC_202);

#endif
