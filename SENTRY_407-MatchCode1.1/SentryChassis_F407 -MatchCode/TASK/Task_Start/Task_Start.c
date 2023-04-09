/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      ��ʼ����
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


/* �������ȼ���ֵԽС���������ȼ�Խ�� */

/* ���̿������� */
#define Chassis_TASK_PRIO  5
#define Chassis_STK_SIZE   200
TaskHandle_t ChassisTask_Handler;

/* ������� */
#define FIRE_TASK_PRIO  4
#define FIRE_STK_SIZE   50
static TaskHandle_t FireTask_Handler;

/* �������� */
#define Detect_TASK_PRIO  2
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;


#ifdef TEST_MODE
/* ������Գ��� */
#define TEST_TASK_PRIO  2
#define TEST_STK_SIZE   80
static TaskHandle_t TestTask_Handler;
#endif

/* ң�����ݴ������� */
#define REMOTE_TASK_PRIO  4
#define REMOTE_STK_SIZE   200
static TaskHandle_t RemoteTask_Handler;

/* ��ʼ���� */
#define START_TASK_PRIO  5
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;


void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���
	
	//һ����ͨ��������, ���ȼ�2
	xTaskCreate((TaskFunction_t)DETECT_Task,         //һ����ͨ��������
                (const char *)"DETECT_Task",         //��������
                (uint16_t)Detect_STK_SIZE,           //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ��� 
                (UBaseType_t)Detect_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t *)&DetectTask_Handler);//������
	
	//��ɵ��̿�������, ���ȼ�5
	xTaskCreate((TaskFunction_t)CHASSIS_Task,         
                (const char *)"CHASSIS_Task",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
	
	//�����������, ���ȼ�4
	xTaskCreate((TaskFunction_t)FIRE_Task,              	
                (const char *)"FIRE_Task",
                (uint16_t)FIRE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FIRE_TASK_PRIO,
                (TaskHandle_t *)&FireTask_Handler);
	
#ifdef TEST_MODE
	//���������������	
	xTaskCreate((TaskFunction_t)TEST_Task,              
                (const char *)"TEST_Task",
                (uint16_t)TEST_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TEST_TASK_PRIO,
                (TaskHandle_t *)&TestTask_Handler);
#endif
				
	//����ң�����ݴ�������, ���ȼ�4
	xTaskCreate((TaskFunction_t)REMOTE_Task,         	
                (const char *)"REMOTE_Task",
                (uint16_t)REMOTE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REMOTE_TASK_PRIO,
                (TaskHandle_t *)&RemoteTask_Handler);
	
	vTaskDelete(StartTask_Handler);  //ɾ����ʼ����
	taskEXIT_CRITICAL();             //�˳��ٽ���
}


/* ��ʼ���� */
void StartTask()
{	
	xTaskCreate((TaskFunction_t)START_Task,
                (const char *)"START_Task",
                (uint16_t)START_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIO,
                (TaskHandle_t *)&StartTask_Handler);
}






