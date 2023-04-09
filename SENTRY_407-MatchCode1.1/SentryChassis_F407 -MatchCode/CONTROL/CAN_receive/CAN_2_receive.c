/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      通过CAN2进行板间通讯
  ******************************************************************************
  */

#include "CAN_2_Receive.h"
#include "CAN_1_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"

/*
   部分简写：RC：remote contral   遥控器
             MK: mouse key        键鼠
*/

/*-------------------- 声明 ---------------------*/


/*  can2接收处理函数 */
static void CAN2_gimbal_receive(CanRxMsg *rx_message);
/*  返回敌人状态 */
VisionStatus_E  get_Enemy_status(void);

/*--------------------变量-----------------------*/
//用于接收来自云台的yaw轴数据

/*--------------------函数-----------------------*/
//can接收处理函数

//can2接收中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
			
				CAN2_gimbal_receive(&rx2_message);          		//中断处理函数
				
				

    }
}



/**********************************************************************************/
/*************************************can2发送*************************************/
/**********************************************************************************/


/* 底盘板CAN1遥控器数据发送 */
void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can1_RC_send)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

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

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}



/*  发送敌人颜色 、射速 、热量上限、枪口热量、受伤状态给云台 */
void CAN2_Enemy_color_SetMsg(int16_t ESC_208 , int16_t bullet_speed , uint16_t shooter_cooling_limit ,  uint16_t  shooter_cooling_heat,uint8_t hurt_status)
{
	  u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体
	
	  CAN2_TxMessage.StdId = 0x3ff;      //根据820r设置标识符
    CAN2_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	  CAN2_TxMessage.DLC = 8;            //指定数据的长度

		CAN2_TxMessage.Data[0] = (unsigned char)(hurt_status);
    CAN2_TxMessage.Data[1] = (unsigned char)(shooter_cooling_heat >> 8);
    CAN2_TxMessage.Data[2] = (unsigned char)(shooter_cooling_heat);
    CAN2_TxMessage.Data[3] = (unsigned char)(shooter_cooling_limit >> 8);
		CAN2_TxMessage.Data[4] = (unsigned char)(shooter_cooling_limit);
    CAN2_TxMessage.Data[5] = (unsigned char)(bullet_speed >> 8);
		CAN2_TxMessage.Data[6] = (unsigned char)(bullet_speed);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);
	
    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
			
}

/**********************************************************************************/
/********************************* can2接收处理 ***********************************/
/**********************************************************************************/

static VisionStatus_E  Enemy_status = Enemy_Disappear;			//敌人出现状态

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

