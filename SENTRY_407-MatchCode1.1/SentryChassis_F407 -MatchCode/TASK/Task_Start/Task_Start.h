/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      ��ʼ����
  ******************************************************************************
  */

#ifndef __TASK_START_H
#define __TASK_START_H
#include "maths.h"
#include "filter.h"
#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h" 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


/*��������״̬*/
//typedef enum 
//{
//	INITIALIZE,   //��ʼ��״̬
//	POWEROFF,     //����״̬
//	
//	WORKING,      //�ֶ�״̬
//	
//	AUTOATTACK,   //����״̬
//	AUTOBUFF,     //���״̬
//	REPLENISHMEN, //����״̬
//	DOUBLE_GIMBAL,//˫��̨״̬������̨���飬����̨������ 
//	
//}WorkStatus;



void StartTask(void);

#endif
