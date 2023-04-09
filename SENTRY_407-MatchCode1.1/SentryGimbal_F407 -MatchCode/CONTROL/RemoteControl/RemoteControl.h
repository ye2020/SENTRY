/**
  ******************************************************************************
  * @file       RemoteControl.c/h
  * @brief      ���ң�������ܺ����ݴ���
  ******************************************************************************
  */
#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H
#include "main.h"


//������ջ���
#define SBUS_RX_BUF_NUM   18


//ң���ĵ�����-BEGIN
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN            ((uint16_t)364 )			//ͨ����Сֵ
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024)			//ͨ���м�ֵ
#define RC_CH_VALUE_MAX            ((uint16_t)1684)			//ͨ�����ֵ
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR                ((uint16_t)0)  //�������ش���
#define RC_SW1_UP                   ((uint16_t)1)
#define RC_SW1_MID                  ((uint16_t)3)
#define RC_SW1_DOWN                 ((uint16_t)2)

#define RC_SW2_UP                   ((uint16_t)1)
#define RC_SW2_MID                  ((uint16_t)3)
#define RC_SW2_DOWN                 ((uint16_t)2)

/* ----------------------- Data Struct ------------------------------------- */


typedef __packed struct //ң��ͳһ�ṹ��
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


//ң������ʼ��
extern void remote_control_init(void);
//��ȡң����ָ��
extern const RC_ctrl_t *Get_Remote_Control_Point(void);
//ң�����ݴ���
extern void RC_Deal(RC_ctrl_t *rc_ctrl, volatile const uint8_t *sbus_buf);
//�ж�ң���������Ƿ��쳣
extern void RC_data_is_error(void);
//ң��������
extern void RC_restart(uint16_t dma_buf_num);
//ң������ֵ����
extern void Remote_reload(void);

extern char Return_RC_S1num(void);
extern char Return_RC_S2num(void);

#endif









