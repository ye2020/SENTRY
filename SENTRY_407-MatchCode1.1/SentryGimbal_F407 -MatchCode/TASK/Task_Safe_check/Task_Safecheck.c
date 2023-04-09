/*************安全监测*******************
内容：1.遥控器安全监测（现象：断连，数据异常
                      处理：强行变为待机状态）
      2.can2断连检测（现象：断连，数据异常
                     处理：云台板：恢复待机状态
						   底盘板：恢复待机状态）
备注：can看门狗还没加
*****************************************/
#include "Task_Safecheck.h"
#include "Task_Detect.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"
#include "RemoteControl.h"


//================参数定义
int safecheck_heart = 0; //安全检测任务心跳

//================结构体定义
DogTypeDef Safecheck_dog = {0, 0, 0}; //相关结构体声明
DogTypeDef Dog = {0, 1, 1};

//================内部函数定义
static void RC_Check(void);
static void Dog_Feed(void);

/*=======安全监测任务======*/
void SAFECHECK_Task(void *pvParameters)
{

    while (1)
    {
#ifdef watch_dog

        LEDE4 = safecheck_heart;	//安全检测任务心跳
		
//        //遥控断连检测
//        RC_Check();
//        //喂狗
//        Dog_Feed();
#endif

        //检测周期
        vTaskDelay(CHECK_CONTROL_TIME_MS / portTICK_RATE_MS);  // portTICK_RATE_MS 用于将以心跳为单位的时间值转化为以毫秒为单位的时间值。
    }
}

/*=================检测遥控连接========================*/
static void RC_Check(void)
{
	
}

/*=======================喂狗函数=======================*/
static void Dog_Feed(void)
{
    if (Safecheck_dog.RC_Connected == 1)
    {
        IWDG_ReloadCounter(); //重装载
    }
}
