/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
  ******************************************************************************
  */
  
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Gimbal.h"
#include "Task_Fire.h"
#include "Task_Safecheck.h"



/* 云台控制任务 */
#define GIMBAL_TASK_PRIO  1
#define GIMBAL_STK_SIZE   200
static TaskHandle_t GIMBALTask_Handler;

/* 火控任务 */
#define FIRE_TASK_PRIO  3
#define FIRE_STK_SIZE   50
static TaskHandle_t FireTask_Handler;

/* 心跳程序 */
#define Detect_TASK_PRIO  5
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;

/* 安全监测程序 */
#define SAFE_TASK_PRIO  2
#define SAFE_STK_SIZE   80
static TaskHandle_t SafecheckTask_Handler;

/* 开始任务 */
#define START_TASK_PRIO  1
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;



void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区
			
	xTaskCreate((TaskFunction_t)GIMBAL_Task,         //完成云台控制任务
                (const char *)"GIMBAL_Task",         //任务名称
                (uint16_t)GIMBAL_STK_SIZE,           //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)GIMBAL_TASK_PRIO,       //任务优先级
                (TaskHandle_t *)&GIMBALTask_Handler);//任务句柄
	
	xTaskCreate((TaskFunction_t)DETECT_Task,         //一个普通心跳程序
                (const char *)"DETECT_Task",         //任务名称
                (uint16_t)Detect_STK_SIZE,           //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数 
                (UBaseType_t)Detect_TASK_PRIO,       //任务优先级
                (TaskHandle_t *)&DetectTask_Handler);//任务句柄
				
	xTaskCreate((TaskFunction_t)FIRE_Task,              //创建火控任务	
                (const char *)"FIRE_Task",              //任务名称
                (uint16_t)FIRE_STK_SIZE,                //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)FIRE_TASK_PRIO,            //任务优先级
                (TaskHandle_t *)&FireTask_Handler);     //任务句柄
	
	xTaskCreate((TaskFunction_t)SAFECHECK_Task,         //创建安全检查任务	
                (const char *)"SAFECHECK_Task",         //任务名称
                (uint16_t)SAFE_STK_SIZE,                //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)SAFE_TASK_PRIO,            //任务优先级
                (TaskHandle_t *)&SafecheckTask_Handler);//任务句柄
	
	vTaskDelete(StartTask_Handler);  //删除开始任务
	taskEXIT_CRITICAL();             //退出临界区
}



void StartTask()
{

	xTaskCreate((TaskFunction_t)START_Task,          //任务函数
                (const char *)"START_Task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
	
}






