/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      ͨ��CAN2���а��ͨѶ
  ******************************************************************************
  */

#include "CAN_2_Receive.h"
#include "CAN_1_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Gimbal.h"

/*
   ���ּ�д��RC��remote contral   ң����
             MK: mouse key        ����
*/

/*--------------------����-----------------------*/
//����yaw��3508�������
//��������̨�������
static motor_measure_t motor_yaw;

uint8_t  Enemy_color;				//������ɫ
static  int16_t  bullet_speed_to_pc;//ǹ������
static 	uint16_t hot_max;						//ǹ����������
static  uint16_t hot_current;				//ǹ������
static  uint8_t  hurt_status;				//����״̬


//���ظ�yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}

/*--------------------����-----------------------*/
//can���մ�����
#ifdef board_gimbal
    static void CAN2_gimbal_receive(CanRxMsg *rx_message);
#endif
#ifdef board_chassis
    static void CAN2_chassis_receive(CanRxMsg *rx_message);
#endif

extern RC_ctrl_t rc_ctrl;

//can2�����ж�
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);

        #ifdef board_gimbal
        CAN2_gimbal_receive(&rx2_message);
        #endif

        #ifdef board_chassis
        CAN2_chassis_receive(&rx2_message);
        #endif
    }
}



/**********************************************************************************/
/*************************************can2����*************************************/
/**********************************************************************************/
void CAN2_yaw_Setmsg(int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN2_TxMessage.StdId = 0x1ff;      //����820r���ñ�ʶ��
    CAN2_TxMessage.IDE = CAN_ID_STD;   //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN2_TxMessage.DLC = 8;            //ָ�����ݵĳ���

    CAN2_TxMessage.Data[0] = (unsigned char)(0);
    CAN2_TxMessage.Data[1] = (unsigned char)(0);
    CAN2_TxMessage.Data[2] = (unsigned char)(0);
    CAN2_TxMessage.Data[3] = (unsigned char)(0);
    CAN2_TxMessage.Data[4] = (unsigned char)(0);
    CAN2_TxMessage.Data[5] = (unsigned char)(0);
    CAN2_TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //������Ϣ
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //�ȴ����ͽ���
}


/*  ���͵���״̬������ */
void CAN2_Enemy_status(int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN2_TxMessage.StdId = 0x2ff;      //����820r���ñ�ʶ��
    CAN2_TxMessage.IDE = CAN_ID_STD;   //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN2_TxMessage.DLC = 8;            //ָ�����ݵĳ���

    CAN2_TxMessage.Data[0] = (unsigned char)(0);
    CAN2_TxMessage.Data[1] = (unsigned char)(0);
    CAN2_TxMessage.Data[2] = (unsigned char)(0);
    CAN2_TxMessage.Data[3] = (unsigned char)(0);
    CAN2_TxMessage.Data[4] = (unsigned char)(0);
    CAN2_TxMessage.Data[5] = (unsigned char)(0);
    CAN2_TxMessage.Data[6] = (unsigned char)(0);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //������Ϣ
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //�ȴ����ͽ���


}

/**********************************************************************************/
/*************************************can2����*************************************/
/**********************************************************************************/
static void CAN2_gimbal_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
        case 0x400:
        {
//			Safecheck_dog.RC_Receive_Flag = 1;
            rc_ctrl.rc.ch[2] = ((rx_message->Data[0] << 8) | rx_message->Data[1]);
            rc_ctrl.rc.ch[3] = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
            rc_ctrl.rc.ch[4] = (rx_message->Data[4] << 8 | rx_message->Data[5]);
            rc_ctrl.rc.s1 = (rx_message->Data[6]);
            rc_ctrl.rc.s2 = (rx_message->Data[7]);

            break;
        }

        /*yaw���*/
        case CAN_YAW_MOTOR_ID:
        {
            motor_yaw.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
            Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //����yaw�������ʵ����ֵ
            motor_yaw.yaw_angle = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
            motor_yaw.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
            break;
        }

        /*  ������ɫ */

        case CAN_ENEMY_COLOR_ID:
        {
            Enemy_color 			 = ( rx_message->Data[7]);
            bullet_speed_to_pc = ((rx_message->Data[5] << 8) | rx_message->Data[6]);			//����
            hot_max						 = ((rx_message->Data[3] << 8) | rx_message->Data[4]);			//ǹ����������
            hot_current        = ((rx_message->Data[1] << 8) | rx_message->Data[2]);			//ǹ������
            hurt_status				 = ( rx_message->Data[0]);																	//����״̬
        }

        default:
            break;
    }
}

uint8_t  get_Enemy_status(void)
{
    return Enemy_color;
}

int16_t  get_bullet_speed_from_chassis(void)
{
    return bullet_speed_to_pc;
}

uint16_t get_shooter_cooling_limit(void)	//ǹ����������
{
    return hot_max;
}

uint16_t get_shooter_cooling_heat(void)
{
    return hot_current;
}

uint8_t get_hurt_status(void)
{
    return hurt_status;
}