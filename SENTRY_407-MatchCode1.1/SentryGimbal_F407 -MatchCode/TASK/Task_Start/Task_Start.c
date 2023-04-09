/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      ��ʼ����
  ******************************************************************************
  */
  
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Gimbal.h"
#include "Task_Fire.h"
#include "Task_Safecheck.h"



/* ��̨�������� */
#define GIMBAL_TASK_PRIO  1
#define GIMBAL_STK_SIZE   200
static TaskHandle_t GIMBALTask_Handler;

/* ������� */
#define FIRE_TASK_PRIO  3
#define FIRE_STK_SIZE   50
static TaskHandle_t FireTask_Handler;

/* �������� */
#define Detect_TASK_PRIO  5
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;

/* ��ȫ������ */
#define SAFE_TASK_PRIO  2
#define SAFE_STK_SIZE   80
static TaskHandle_t SafecheckTask_Handler;

/* ��ʼ���� */
#define START_TASK_PRIO  1
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;



void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���
			
	xTaskCreate((TaskFunction_t)GIMBAL_Task,         //�����̨��������
                (const char *)"GIMBAL_Task",         //��������
                (uint16_t)GIMBAL_STK_SIZE,           //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)GIMBAL_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t *)&GIMBALTask_Handler);//������
	
	xTaskCreate((TaskFunction_t)DETECT_Task,         //һ����ͨ��������
                (const char *)"DETECT_Task",         //��������
                (uint16_t)Detect_STK_SIZE,           //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ��� 
                (UBaseType_t)Detect_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t *)&DetectTask_Handler);//������
				
	xTaskCreate((TaskFunction_t)FIRE_Task,              //�����������	
                (const char *)"FIRE_Task",              //��������
                (uint16_t)FIRE_STK_SIZE,                //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)FIRE_TASK_PRIO,            //�������ȼ�
                (TaskHandle_t *)&FireTask_Handler);     //������
	
	xTaskCreate((TaskFunction_t)SAFECHECK_Task,         //������ȫ�������	
                (const char *)"SAFECHECK_Task",         //��������
                (uint16_t)SAFE_STK_SIZE,                //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)SAFE_TASK_PRIO,            //�������ȼ�
                (TaskHandle_t *)&SafecheckTask_Handler);//������
	
	vTaskDelete(StartTask_Handler);  //ɾ����ʼ����
	taskEXIT_CRITICAL();             //�˳��ٽ���
}



void StartTask()
{

	xTaskCreate((TaskFunction_t)START_Task,          //������
                (const char *)"START_Task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
	
}






