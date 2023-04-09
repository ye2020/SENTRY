/*************安全监测*******************
内容：1.遥控器安全监测（现象：断连，数据异常
                      处理：强行变为待机状态）
      2.can断连检测（现象：断连，数据异常
                     处理：云台板：恢复待机状态
						   底盘板：恢复待机状态）
备注：
*****************************************/
#include "Task_Safecheck.h"
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Safecheck.h"
#include "RemoteControl.h"

extern TaskHandle_t ChassisTask_Handler;


SafeTypeDef Safecheck;

/*=======安全监测任务======*/
void SAFECHECK_Task(void *pvParameters)
{

}


