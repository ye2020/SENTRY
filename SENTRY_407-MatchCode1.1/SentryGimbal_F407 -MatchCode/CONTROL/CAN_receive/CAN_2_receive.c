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
#include "Task_Gimbal.h"

/*
   部分简写：RC：remote contral   遥控器
             MK: mouse key        键鼠
*/

/*--------------------变量-----------------------*/
//申明yaw轴3508电机变量
//申明副云台电机变量
static motor_measure_t motor_yaw;

uint8_t  Enemy_color;				//敌人颜色
static  int16_t  bullet_speed_to_pc;//枪口热量
static 	uint16_t hot_max;						//枪口热量上限
static  uint16_t hot_current;				//枪口热量
static  uint8_t  hurt_status;				//受伤状态


//返回副yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}

/*--------------------函数-----------------------*/
//can接收处理函数
#ifdef board_gimbal
    static void CAN2_gimbal_receive(CanRxMsg *rx_message);
#endif
#ifdef board_chassis
    static void CAN2_chassis_receive(CanRxMsg *rx_message);
#endif

extern RC_ctrl_t rc_ctrl;

//can2接收中断
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
/*************************************can2发送*************************************/
/**********************************************************************************/
void CAN2_yaw_Setmsg(int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

    CAN2_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN2_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN2_TxMessage.DLC = 8;            //指定数据的长度

    CAN2_TxMessage.Data[0] = (unsigned char)(0);
    CAN2_TxMessage.Data[1] = (unsigned char)(0);
    CAN2_TxMessage.Data[2] = (unsigned char)(0);
    CAN2_TxMessage.Data[3] = (unsigned char)(0);
    CAN2_TxMessage.Data[4] = (unsigned char)(0);
    CAN2_TxMessage.Data[5] = (unsigned char)(0);
    CAN2_TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}


/*  发送敌人状态给底盘 */
void CAN2_Enemy_status(int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

    CAN2_TxMessage.StdId = 0x2ff;      //根据820r设置标识符
    CAN2_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN2_TxMessage.DLC = 8;            //指定数据的长度

    CAN2_TxMessage.Data[0] = (unsigned char)(0);
    CAN2_TxMessage.Data[1] = (unsigned char)(0);
    CAN2_TxMessage.Data[2] = (unsigned char)(0);
    CAN2_TxMessage.Data[3] = (unsigned char)(0);
    CAN2_TxMessage.Data[4] = (unsigned char)(0);
    CAN2_TxMessage.Data[5] = (unsigned char)(0);
    CAN2_TxMessage.Data[6] = (unsigned char)(0);
    CAN2_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束


}

/**********************************************************************************/
/*************************************can2接收*************************************/
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

        /*yaw电机*/
        case CAN_YAW_MOTOR_ID:
        {
            motor_yaw.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
            Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
            motor_yaw.yaw_angle = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
            motor_yaw.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
            break;
        }

        /*  敌人颜色 */

        case CAN_ENEMY_COLOR_ID:
        {
            Enemy_color 			 = ( rx_message->Data[7]);
            bullet_speed_to_pc = ((rx_message->Data[5] << 8) | rx_message->Data[6]);			//弹速
            hot_max						 = ((rx_message->Data[3] << 8) | rx_message->Data[4]);			//枪口热量上限
            hot_current        = ((rx_message->Data[1] << 8) | rx_message->Data[2]);			//枪口热量
            hurt_status				 = ( rx_message->Data[0]);																	//受伤状态
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

uint16_t get_shooter_cooling_limit(void)	//枪口热量上限
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