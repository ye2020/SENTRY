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

/*
   ���ּ�д��RC��remote contral   ң����
             MK: mouse key        ����
*/

/*-------------------- ���� ---------------------*/


/*  can2���մ����� */
static void CAN2_gimbal_receive(CanRxMsg *rx_message);
/*  ���ص���״̬ */
VisionStatus_E  get_Enemy_status(void);

/*--------------------����-----------------------*/
//���ڽ���������̨��yaw������

/*--------------------����-----------------------*/
//can���մ�����

//can2�����ж�
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
			
				CAN2_gimbal_receive(&rx2_message);          		//�жϴ�����
				
				

    }
}



/**********************************************************************************/
/*************************************can2����*************************************/
/**********************************************************************************/


/* ���̰�CAN1ң�������ݷ��� */
void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can1_RC_send)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CAN2_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN2_TxMessage.StdId = 0x400;
    CAN2_TxMessage.IDE = CAN_ID_STD;
    CAN2_TxMessage.RTR = CAN_RTR_DATA;
    CAN2_TxMessage.DLC = 8;

    CAN2_TxMessage.Data[0] = (unsigned char)(can1_RC_send->rc.ch[2] >> 8);
    CAN2_TxMessage.Data[1] = (unsigned char)(can1_RC_send->rc.ch[2]);
    CAN2_TxMessage.Data[2] = (unsigned char)(can1_RC_send->rc.ch[3] >> 8);
    CAN2_TxMessage.Data[3] = (unsigned char)(can1_RC_send->rc.ch[3]);
    CAN2_TxMessage.Data[4] = (unsigned char)(can1_RC_send->rc.ch[4] >> 8);
    CAN2_TxMessage.Data[5] = (unsigned char)(can1_RC_send->rc.ch[4]);
    CAN2_TxMessage.Data[6] = (unsigned char)(can1_RC_send->rc.s1);
    CAN2_TxMessage.Data[7] = (unsigned char)(can1_RC_send->rc.s2);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage);
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //�ȴ����ͽ���
        i++;
}



/*  ���͵�����ɫ ������ ���������ޡ�ǹ������������״̬����̨ */
void CAN2_Enemy_color_SetMsg(int16_t ESC_208 , int16_t bullet_speed , uint16_t shooter_cooling_limit ,  uint16_t  shooter_cooling_heat,uint8_t hurt_status)
{
	  u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //����һ��������Ϣ�Ľṹ��
	
	  CAN2_TxMessage.StdId = 0x3ff;      //����820r���ñ�ʶ��
    CAN2_TxMessage.IDE = CAN_ID_STD;   //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
	  CAN2_TxMessage.DLC = 8;            //ָ�����ݵĳ���

		CAN2_TxMessage.Data[0] = (unsigned char)(hurt_status);
    CAN2_TxMessage.Data[1] = (unsigned char)(shooter_cooling_heat >> 8);
    CAN2_TxMessage.Data[2] = (unsigned char)(shooter_cooling_heat);
    CAN2_TxMessage.Data[3] = (unsigned char)(shooter_cooling_limit >> 8);
		CAN2_TxMessage.Data[4] = (unsigned char)(shooter_cooling_limit);
    CAN2_TxMessage.Data[5] = (unsigned char)(bullet_speed >> 8);
		CAN2_TxMessage.Data[6] = (unsigned char)(bullet_speed);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);
	
    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //������Ϣ
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //�ȴ����ͽ���
			
}

/**********************************************************************************/
/********************************* can2���մ��� ***********************************/
/**********************************************************************************/

static VisionStatus_E  Enemy_status = Enemy_Disappear;			//���˳���״̬

static void CAN2_gimbal_receive(CanRxMsg *rx_message)
{

		 switch (rx_message->StdId)
		 {
				case 0x2ff:
				{
					Enemy_status = (VisionStatus_E)(rx_message->Data[7]);
					
					break;
				}
					default:
          break;
		 }
		 
}

VisionStatus_E  get_Enemy_status(void)
{
	return Enemy_status;
}

