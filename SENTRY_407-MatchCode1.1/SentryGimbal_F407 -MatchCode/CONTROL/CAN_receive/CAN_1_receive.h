/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1���պͷ��͵������
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

//rm���ͳһ���ݽṹ��
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
    int32_t actual_Position;  //��ʵλ��
} motor_measure_t;


/*--------------------����-----------------------*/
/* ���̰�CAN1���͵���̨Y����͹������ | Y������205��P������206�����������207��P����0x206û�õ��� */
extern void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern  motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *Get_Fire_MotorA_Measure_Point(void);
extern const motor_measure_t *Get_Fire_MotorB_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i);

#endif
