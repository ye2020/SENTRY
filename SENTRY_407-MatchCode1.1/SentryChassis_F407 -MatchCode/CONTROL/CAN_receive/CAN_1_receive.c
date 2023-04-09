/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1接收和发送电机数据
  ******************************************************************************
  */

#include "CAN_1_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Safecheck.h"
#include "Task_Chassis.h"

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                                           \
    {                                                                                          \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);         \
    }


/*
   部分简写：RC：remote contral   遥控
*/


/*--------------------变量-----------------------*/
//申明底盘电机变量
static motor_measure_t motor_chassis[4];
//申明拨弹电机变量
static motor_measure_t motor_fire;
//申明yaw轴3508电机变量
static motor_measure_t motor_yaw;


/*--------------------函数-----------------------*/
//can接收处理函数
#ifdef board_chassis
    static void CAN1_chassis_receive(CanRxMsg *rx_message);
#endif


//返回yaw电机变量地址，通过指针方式获取原始数据
motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回拨弹电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Fire_Motor_Measure_Point(void)
{
    return &motor_fire;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];  //(i & 0x03)
}

extern SafeTypeDef Safecheck;

//can1接收中断
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
/*************************************can1发送*************************************/
/**********************************************************************************/
/* 底盘板CAN1发送到底盘*/
void CAN1_Chassis_SetMsg(int16_t ESC_202 , int16_t ESC_203)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = CAN_CHASSIS_ALL_ID; //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;           //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA;         //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;                    // 发送两帧信息
		CAN1_TxMessage.Data[0] = 0;
		CAN1_TxMessage.Data[1] = 0;
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_203);
		CAN1_TxMessage.Data[6] = 0;
		CAN1_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}





/**********************************************************************************/
/*************************************can1接收*************************************/
/**********************************************************************************/
/* 底盘板CAN1接收处理函数
   底盘板：底盘4电机数据反馈，yaw轴3508数据反馈，拨弹电机2006数据反馈，*/
static void CAN1_chassis_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
        /*底盘电机*/
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

