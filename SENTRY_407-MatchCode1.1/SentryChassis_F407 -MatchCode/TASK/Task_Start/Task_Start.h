/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
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


/*整机工作状态*/
//typedef enum 
//{
//	INITIALIZE,   //初始化状态
//	POWEROFF,     //待机状态
//	
//	WORKING,      //手动状态
//	
//	AUTOATTACK,   //自瞄状态
//	AUTOBUFF,     //打符状态
//	REPLENISHMEN, //补给状态
//	DOUBLE_GIMBAL,//双云台状态（主云台自瞄，副云台操作） 
//	
//}WorkStatus;



void StartTask(void);

#endif
