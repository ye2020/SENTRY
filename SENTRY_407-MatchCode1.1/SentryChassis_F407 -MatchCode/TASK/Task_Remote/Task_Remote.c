#include "Task_Remote.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "RemoteControl.h"
#include "pid.h"
#include "rmmotor.h"
#include "led.h"

extern SemaphoreHandle_t Remote_BinarySem;

void REMOTE_Task(void *pvParameters)
{
	vTaskDelay(5);
	
    while (1)
    {

		
		//��ȡ��ֵ�ź���
		//xSemaphoreTake(Remote_BinarySem, portMAX_DELAY); //��ֵ�ź���������ȴ�ʱ��
		
        vTaskDelay(200);
    }
}

