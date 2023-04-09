/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1���պͷ��͵������
  ******************************************************************************
  */

#include "CAN_1_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Safecheck.h"
#include "Task_Chassis.h"

//������ݶ�ȡ
#define get_motor_M3508(ptr, rx_message)                                                           \
    {                                                                                          \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);         \
    }


/*
   ���ּ�д��RC��remote contral   ң��
*/


/*--------------------����-----------------------*/
//�������̵������
static motor_measure_t motor_chassis[4];
//���������������
static motor_measure_t motor_fire;
//����yaw��3508�������
static motor_measure_t motor_yaw;


/*--------------------����-----------------------*/
//can���մ�����
#ifdef board_chassis
    static void CAN1_chassis_receive(CanRxMsg *rx_message);
#endif


//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Fire_Motor_Measure_Point(void)
{
    return &motor_fire;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];  //(i & 0x03)
}

extern SafeTypeDef Safecheck;

//can1�����ж�
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);


        #ifdef board_chassis
        CAN1_chassis_receive(&rx1_message);
        #endif
    }
}


/**********************************************************************************/
/*************************************can1����*************************************/
/**********************************************************************************/
/* ���̰�CAN1���͵�����*/
void CAN1_Chassis_SetMsg(int16_t ESC_202 , int16_t ESC_203)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN1_TxMessage.StdId = CAN_CHASSIS_ALL_ID; //����820r���ñ�ʶ��
    CAN1_TxMessage.IDE = CAN_ID_STD;           //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN1_TxMessage.RTR = CAN_RTR_DATA;         //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN1_TxMessage.DLC = 8;                    // ������֡��Ϣ
		CAN1_TxMessage.Data[0] = 0;
		CAN1_TxMessage.Data[1] = 0;
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_203);
		CAN1_TxMessage.Data[6] = 0;
		CAN1_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //������Ϣ
    i = 0;

    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //�ȴ����ͽ���
        i++;
}





/**********************************************************************************/
/*************************************can1����*************************************/
/**********************************************************************************/
/* ���̰�CAN1���մ�����
   ���̰壺����4������ݷ�����yaw��3508���ݷ������������2006���ݷ�����*/
static void CAN1_chassis_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
        /*���̵��*/
        case 0x202:
        {
						Safecheck.CAN_Receive_Flag = 1;
            get_motor_M3508(&motor_chassis[1], rx_message);
            motor_chassis[1].position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
						Motor_Actual_Position(&motor_chassis[1],CHASSIS_RATIO,8192);
            break;
        }
        default:
            break;
    }
}

