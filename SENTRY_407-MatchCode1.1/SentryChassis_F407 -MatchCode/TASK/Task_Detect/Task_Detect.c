#include "Task_Detect.h"
#include "led.h"

void DETECT_Task(void *pvParameters)
{
    LEDB7 = 0;

    //����һ��ʱ��
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        LEDB7 = !LEDB7;
        vTaskDelay(DETECT_CONTROL_TIME);
    }
}
