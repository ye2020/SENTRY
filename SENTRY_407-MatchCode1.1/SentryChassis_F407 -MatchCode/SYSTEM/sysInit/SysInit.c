#include "SysInit.h"

//HARDWARE
#include "can.h"
#include "key.h"
#include "led.h"
#include "pid.h"
#include "pwm.h"
#include "adc.h"
#include "rng.h"
#include "time.h"
#include "iwdg.h"
#include "usart.h"
#include "sensor.h"
#include "photoelectric.h"

//REFEREE
#include "RefereeDeal.h"

//OPERATION
#include "filter.h"
#include "maths.h"
#include "rmmotor.h"

//CONTROL
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "chassis_behaviour.h"
#include "RemoteControl.h"
#include "Capacitor_control.h"
#include "Double_Gimbal.h"

//Task
#include "Task_Start.h"
#include "Task_Chassis.h"
#include "Task_Fire.h"
#include "Task_Detect.h"
#include "Task_Safecheck.h"
#include "Task_Test.h"

/**
  * @brief          硬件层初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
void System_Init(void)
{
    taskENTER_CRITICAL();  //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区

    delay_ms(10);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置系统中断优先级分组 4

	
    /* 串口初始化 */
    Remote_Control_Init();      //串口一初始化（遥控器初始化）
    //Capacitance_Control_Init(); //串口二初始化（与超级电容控制板通讯）
    referee_system_init();      //串口六初始化（裁判系统）

	
    /* GPIO初始化 */
    LED_Init();               //板载LED初始化
		Laser_GPIO_Init();				//两端漫反射传感器



    /* CAN接口初始化 */
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

		
		
		/*随机数初始化*/
		RNG_Init();
		
		
		/*数模转换初始化*/
		Adc_Init();
		
    /* 看门狗初始化 */
//    IWDG_Init(IWDG_Prescaler_64, 625);   //1s


    /*初始化完成状态*/
    LEDB7 = 1;

    delay_ms(100);

    taskEXIT_CRITICAL();             //退出临界区
}



/**
  * @brief          失控保护
  * @param[in]      none
  * @retval         none
  * @attention
  */
void SYSTEM_OutCtrlProtect(void)
{
    //SYSTEM_Reset();      //系统恢复至重启状态
    Remote_reload();      //遥控数据恢复至默认状态
    Laser_OFF();          //激光关
    Chassis_Task_OFF(2);  //底盘关
    //Magazine_StopCtrl();  //舵机停止转动
    Fire_Task_OFF();      //火控任务关闭
    SuperCap_OFF();       //电容关闭充放电
}


