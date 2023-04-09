/**
  ******************************************************************************
  * @file       RemoteControl.c/h
  * @brief      完成遥控器接受和数据处理。
  ******************************************************************************
  */
#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H
#include "main.h"


//定义接收缓冲
#define SBUS_RX_BUF_NUM   18


//遥控文档内容-BEGIN
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN            ((uint16_t)364 )			//通道最小值
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024)			//通道中间值
#define RC_CH_VALUE_MAX            ((uint16_t)1684)			//通道最大值
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR                ((uint16_t)0)  //出现严重错误
#define RC_SW1_UP                   ((uint16_t)1)
#define RC_SW1_MID                  ((uint16_t)3)
#define RC_SW1_DOWN                 ((uint16_t)2)

#define RC_SW2_UP                   ((uint16_t)1)
#define RC_SW2_MID                  ((uint16_t)3)
#define RC_SW2_DOWN                 ((uint16_t)2)

/* ----------------------- Data Struct ------------------------------------- */


typedef __packed struct //遥控统一结构体
{
	__packed struct
	{
					int16_t ch[5];
					char s1;
					char s2;
	} rc;
	__packed struct
	{
					int16_t x;
					int16_t y;
					int16_t z;
					uint8_t press_l;
					uint8_t press_r;
	} mouse;
	__packed struct
	{
					uint16_t v;
	} key;

} RC_ctrl_t;


//遥控器初始化
extern void remote_control_init(void);
//获取遥控器指针
extern const RC_ctrl_t *Get_Remote_Control_Point(void);
//遥控数据处理
extern void RC_Deal(RC_ctrl_t *rc_ctrl, volatile const uint8_t *sbus_buf);
//判断遥控器数据是否异常
extern void RC_data_is_error(void);
//遥控器重启
extern void RC_restart(uint16_t dma_buf_num);
//遥控器数值清零
extern void Remote_reload(void);

extern char Return_RC_S1num(void);
extern char Return_RC_S2num(void);

#endif









