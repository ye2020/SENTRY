#include "Task_Detect.h"
#include "led.h"
#include "RemoteControl.h"
#include "iwdg.h"

const RC_ctrl_t * RC_data;


void DETECT_Task(void *pvParameters)
{

		RC_data =  Get_Remote_Control_Point();
    //¿ÕÏÐÒ»¶ÎÊ±¼ä
    vTaskDelay(DETECT_TASK_INIT_TIME);

	
    while (1)
    {
				
						 if((RC_data->rc.ch[0] >= -660 && RC_data->rc.ch[0] <= 660) && 
							  (RC_data->rc.ch[1] >= -660 && RC_data->rc.ch[1] <= 660)	&&
							  (RC_data->rc.ch[2] >= -660 && RC_data->rc.ch[2] <= 660) &&
							  (RC_data->rc.ch[3] >= -660 && RC_data->rc.ch[3] <= 660) &&
						  	(RC_data->rc.ch[4] >= -660 && RC_data->rc.ch[4] <= 660))	IWDG_Feed();        LEDB7 = 0;

				
        vTaskDelay(DETECT_CONTROL_TIME / portTICK_RATE_MS);
    }
}
