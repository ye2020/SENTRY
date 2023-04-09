/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1接收和发送电机数据
  ******************************************************************************
  */

#include "CAN_1_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Gimbal.h"

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                                           \
    {                                                                                          \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);         \
    }


/*
   部分简写：RC：remote contral   遥控器
             MK: mouse key        键鼠
*/


/*--------------------变量-----------------------*/
//申明底盘电机变量
static motor_measure_t motor_chassis[4];
//申明拨弹电机变量
static motor_measure_t motor_fire_A;
static motor_measure_t motor_fire_B;
//申明pitch轴电机变量
motor_measure_t motor_pitch;


/*--------------------函数-----------------------*/
//can接收处理函数
#ifdef board_gimbal
    static void CAN1_gimbal_receive(CanRxMsg *rx_message);
#endif

/*--------------------变量-----------------------*/

//返回pitch电机变量地址，通过指针方式获取原始数据
motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pitch;
}
const motor_measure_t *Get_Fire_MotorA_Measure_Point(void)
{
    return &motor_fire_A;
}
//返回拨弹电机变量地址，通过指针方式获取原始数据
const motor_measure_t *Get_Fire_MotorB_Measure_Point(void)
{
    return &motor_fire_B;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];  //(i & 0x03)
}



//can1接收中断
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);

        #ifdef board_gimbal
        CAN1_gimbal_receive(&rx1_message);
        #endif
    }
}




/**********************************************************************************/
/*************************************can1发送*************************************/
/**********************************************************************************/
/* 底盘板CAN1发送到云台Y电机和供弹电机 | Y轴电机是208，P轴电机是207，供弹电机是207 208 */
void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            //指定数据的长度

    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_206);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_207);
    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}




/**********************************************************************************/
/*************************************can1接收*************************************/
/**********************************************************************************/
static void CAN1_gimbal_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
            #if (pitch_angle_position == 1)

        /*Pitch轴编码器位置反馈*/
        case 0x01:
        {
            motor_pitch.position = (rx_message->Data[6] << 24 | rx_message->Data[5] << 16 | rx_message->Data[4] << 8 | rx_message->Data[3]);
			if(motor_pitch.position>=0&&motor_pitch.position<=150)
		{
				motor_pitch.position +=884;
		}
		else
		    motor_pitch.position -= 140;
            motor_pitch.actual_Position = motor_pitch.position;   //实际值减去中间值
            motor_pitch.pitch_angle = -(motor_pitch.actual_Position * 360 / 1024 / PITCH_GR - Pitch_Middle_Angle);
            break;
        }

        #endif

        /*pitch轴电机*/
        case CAN_PIT_MOTOR_ID:
        {
            motor_pitch.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];

		#if (pitch_angle_position == 0)
						motor_pitch.position  = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
						Motor_Actual_Position (&motor_pitch,1.5*19,8192);
						motor_pitch.pitch_angle  =( (motor_pitch.actual_Position*360/(8192)/1.5/19) - 47);
		#endif
			break;
        }

        /*拨弹电机A*/
        case CAN_TRIGGER_MOTORA_ID:
        {
            motor_fire_A.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
            Motor_Actual_Position(&motor_fire_A, Sec_YAW_RATIO, 8192);
            motor_fire_A.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];

            break;
        }

        /*拨弹电机B*/
        case CAN_TRIGGER_MOTORB_ID:
        {
            motor_fire_B.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
            Motor_Actual_Position(&motor_fire_B, Sec_YAW_RATIO, 8192);
            motor_fire_B.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
            break;
        }

//        case 0x02:
//        {
////			Safecheck_dog.RC_Receive_Flag = 1;
//            motor_yaw.actual_Position = (rx_message->Data[0] << 24 | rx_message->Data[1] << 16 | rx_message->Data[2] << 8 | rx_message->Data[3]);
//            motor_yaw.speed = (rx_message->Data[4] << 8 | rx_message->Data[5]);
//            break;
//				}
        default:
            break;

    }
}

