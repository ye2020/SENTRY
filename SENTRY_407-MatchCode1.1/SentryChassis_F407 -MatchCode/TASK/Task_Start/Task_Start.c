/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
  ******************************************************************************
  */
  
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Fire.h"
#include "Task_Safecheck.h"
#include "Task_Test.h"
#include "Task_Remote.h"
#include "rmmotor.h"


/* 任务优先级数值越小，任务优先级越低 */

/* 底盘控制任务 */
#define Chassis_TASK_PRIO  5
#define Chassis_STK_SIZE   200
TaskHandle_t ChassisTask_Handler;

/* 火控任务 */
#define FIRE_TASK_PRIO  4
#define FIRE_STK_SIZE   50
static TaskHandle_t FireTask_Handler;

/* 心跳程序 */
#define Detect_TASK_PRIO  2
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;


#ifdef TEST_MODE
/* 电机测试程序 */
#define TEST_TASK_PRIO  2
#define TEST_STK_SIZE   80
static TaskHandle_t TestTask_Handler;
#endif

/* 遥控数据处理任务 */
#define REMOTE_TASK_PRIO  4
#define REMOTE_STK_SIZE   200
static TaskHandle_t RemoteTask_Handler;

/* 开始任务 */
#define START_TASK_PRIO  5
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;


void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区
	
	//一个普通心跳程序, 优先级2
	xTaskCreate((TaskFunction_t)DETECT_Task,         //一个普通心跳程序
                (const char *)"DETECT_Task",         //任务名称
                (uint16_t)Detect_STK_SIZE,           //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数 
                (UBaseType_t)Detect_TASK_PRIO,       //任务优先级
                (TaskHandle_t *)&DetectTask_Handler);//任务句柄
	
	//完成底盘控制任务, 优先级5
	xTaskCreate((TaskFunction_t)CHASSIS_Task,         
                (const char *)"CHASSIS_Task",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
	
	//创建火控任务, 优先级4
	xTaskCreate((TaskFunction_t)FIRE_Task,              	
                (const char *)"FIRE_Task",
                (uint16_t)FIRE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FIRE_TASK_PRIO,
                (TaskHandle_t *)&FireTask_Handler);
	
#ifdef TEST_MODE
	//创建电机测试任务	
	xTaskCreate((TaskFunction_t)TEST_Task,              
                (const char *)"TEST_Task",
                (uint16_t)TEST_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TEST_TASK_PRIO,
                (TaskHandle_t *)&TestTask_Handler);
#endif
				
	//创建遥控数据处理任务, 优先级4
	xTaskCreate((TaskFunction_t)REMOTE_Task,         	
                (const char *)"REMOTE_Task",
                (uint16_t)REMOTE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REMOTE_TASK_PRIO,
                (TaskHandle_t *)&RemoteTask_Handler);
	
	vTaskDelete(StartTask_Handler);  //删除开始任务
	taskEXIT_CRITICAL();             //退出临界区
}


/* 开始任务 */
void StartTask()
{	
	xTaskCreate((TaskFunction_t)START_Task,
                (const char *)"START_Task",
                (uint16_t)START_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIO,
                (TaskHandle_t *)&StartTask_Handler);
}






