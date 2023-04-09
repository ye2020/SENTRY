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


/* ң���ĵ�����-BEGIN */
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN            ((uint16_t)364 )			//ͨ����Сֵ
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024)			//ͨ���м�ֵ
#define RC_CH_VALUE_MAX            ((uint16_t)1684)			//ͨ�����ֵ
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR                ((uint16_t)0) //�������ش���
#define RC_SW1_UP                   ((uint16_t)1)
#define RC_SW1_MID                  ((uint16_t)3)
#define RC_SW1_DOWN                 ((uint16_t)2)

#define RC_SW2_UP                   ((uint16_t)1)
#define RC_SW2_MID                  ((uint16_t)3)
#define RC_SW2_DOWN                 ((uint16_t)2)


/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W       ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S       ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D       ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R       ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F       ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G       ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z       ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X       ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C       ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V       ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B       ((uint16_t)1 << 15)
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
extern void Remote_Control_Init(void);
//��ȡң����ָ��
extern const RC_ctrl_t *get_remote_control_point(void);
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









