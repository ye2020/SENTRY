/**
  ******************************************************************************
  * @file       RemoteControl.c/h
  * @brief      完成遥控器数据接收。
  ******************************************************************************
  */
#include "RemoteControl.h"
#include "maths.h"
#include "usart.h"
#include "iwdg.h"

/*
ch0为右边的左右
ch1为右边的上下
ch2为左边的左右
ch3为左边的上下
ch4为拨弹（左上角）
s1为左上
s2为右上
*/

/*****************************操作键位说明书**********************************************************/
/* 一、遥控模式：
         1.底盘跟随  ：左中右上
         2.扭腰模式  ：左下右上
         3.底盘小陀螺：左上右上
         4.打符模式  ：左上右中（底盘以云台坐标轴移动，不跟随）
         5.自瞄模式  ：左下右中
         6.键盘模式  ：左中右中
         7.发弹      ：在初始化完成且右开关打上或中情况下，左上角波轮拉最上发弹，回中停止射弹
         8.关机      ：右下
   二、键鼠模式：
         1.基本运动：WASD
         2.云台运动：鼠标
         3.发射：    鼠标左键单点单发，按住连发
         4.加速（电容放电）：    按住shift（逮虾户）
         5.扭腰模式： F  （按一下进入，再按一下返回）
         6.自瞄模式： 鼠标右键  （按住）
         7.补给模式：  G  （按一下云台往一边转，补完弹再按一下回来）
         8.高射速模式：C （按一次进入）（这个模式没用了，之前全部最大射速打）
         9.低射速模式：V （按一次进入）
         10.退弹模式： Z  （按住）
         11.炮台模式： Ctrl （按住，只能控制云台，底盘不动）
         12.打符模式： X （一次进入）
         13.小陀螺模式：R（按一次进入）
                                                                                                    */
/****************************************************************************************************/


#define rc_deadline_limit(input, output, dealine)    \
{                                                    \
	if ((input) > (dealine) || (input) < -(dealine)) \
	{                                                \
		(output) = (input);                          \
	}                                                \
	else                                             \
	{                                                \
		(output) = 0;                                \
	}                                                \
}


//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

static uint8_t Usart1_Rx[USART1_RX_LEN]; //定义接收缓冲
int RC_FLAG;

RC_ctrl_t rc_ctrl;
//static RC_ctrl_t rc_ctrl;

//遥控器初始化
void remote_control_init(void)
{
	Usart1_Init(Usart1_Rx, USART1_RX_LEN);
}

//返回遥控器控制变量，通过指针传递方式传递信息
const RC_ctrl_t *Get_Remote_Control_Point(void)
{
    return &rc_ctrl;
}



//判断遥控器数据是否出错
void RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (int16_t_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (rc_ctrl.rc.s1 == RC_SW_ERROR)
    {
        goto rc_error;
    }
    if (rc_ctrl.rc.s2 == RC_SW_ERROR)
    {
        goto rc_error;
    }
    return ;

rc_error:
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
//	rc_ctrl.rc.s1 = RC_SW_ERROR;  //出现错误为0
//	rc_ctrl.rc.s2 = RC_SW_ERROR;  //出现错误为0
//	rc_ctrl.rc.s1 = RC_SW_MID;   //中
//	rc_ctrl.rc.s2 = RC_SW_DOWN;  //下
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
	
//	//重启遥控器
//	delay_ms(2);
//	RC_restart(USART1_RX_LEN);  //18
//	delay_ms(2);
}


/* 摇杆量清零 */
void Remote_reload(void)
{
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
}


/* 遥控器重启 */
void RC_restart(uint16_t dma_buf_num)
{
	USART_Cmd(USART1, DISABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);  //设置对应的 DMA 数据流传输的数据量大小

	USART_ClearFlag(USART1, USART_FLAG_IDLE);

	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}



static void RC_Deal(RC_ctrl_t *rc_ctrl, volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)  return;
	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                    //!< Switch left
    rc_ctrl->rc.s2 = ((sbus_buf[5] >> 4) & 0x0003);                         //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = ((int16_t)sbus_buf[16]|((int16_t)sbus_buf[17]<<8))&0x07FF;//遥控左上角拨轮

	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;  // -通道中值1024
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;  // （ -660 ~ 660 ）
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
	rc_deadline_limit(rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[0], 10);  //死区限制
	rc_deadline_limit(rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[1], 10);  //死区限制
	rc_deadline_limit(rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[2], 10);  //死区限制
	rc_deadline_limit(rc_ctrl->rc.ch[3], rc_ctrl->rc.ch[3], 10);  //死区限制
}



void DMA2_Stream5_IRQHandler(void)		//遥控数据的接收
{
	//判断是否为DMA发送完成中断
	if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		LEDE0 = 0; 
		
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		
		//将接收到的数据保存到Usart1_Rx中
		RC_Deal(&rc_ctrl, Usart1_Rx);
		
#ifdef watch_dog
		RC_FLAG++;
		if(500 == RC_FLAG)
		{
			IWDG_Feed();
			RC_FLAG = 0;
		}
#endif

	}
}


/**
  * @brief          返回s1的数值
  * @param[in]      none
  * @retval         返回s1的数值
  * @attention      
  */
char Return_RC_S1num(void)
{
	return (rc_ctrl.rc.s1);
}

/**
  * @brief          返回s2的数值
  * @param[in]      none
  * @retval         返回s2的数值
  * @attention      
  */
char Return_RC_S2num(void)
{
	return (rc_ctrl.rc.s2);
}





