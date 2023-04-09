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

		
		//获取二值信号量
		//xSemaphoreTake(Remote_BinarySem, portMAX_DELAY); //二值信号量句柄、等待时间
		
        vTaskDelay(200);
    }
}

