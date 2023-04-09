#include "SysInit.h"

//HARDWARE
#include "can.h"
#include "iwdg.h"
#include "key.h"
#include "led.h"
#include "photoelectric.h"
#include "pid.h"
#include "time.h"
#include "usart.h"
#include "pwm.h"
#include "usart2.h"
#include "upper_machine.h"


//REFEREE

//CONTROL
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "gimbal_behaviour.h"
#include "IMU.h"
#include "filter.h"
#include "maths.h"
#include "RemoteControl.h"
#include "rmmotor.h"

//Task
#include "Task_Start.h"
#include "Task_Gimbal.h"
#include "Task_Fire.h"
#include "Task_Detect.h"
#include "Task_Safecheck.h"

void System_Init(void)
{
    delay_ms(100);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组 4

//	delay_init(180);  //延时函数初始化

    /*GPIO初始化*/
//	LED_Init();               //板载LED初始化
    Laser_Init();             //激光初始化

    /* 串口初始化 */
    remote_control_init(); //串口一初始化（遥控器初始化）
	
    #if IMU_BMI160
    IMU_control_init(); //串口二初始化（接收陀螺仪数据）	
    #endif
	
    automatic_aiming_init();  //串口三初始化（视觉通讯）
//	Usart6_Init();            //裁判系统

	/*  蓝牙串口调试 */
		upper_machine_communication();
		
    /* GPIO初始化 */
    LED_Init();   //板载LED初始化

    /* 光电初始化 */
    Yaw_Zero_GPIO_Init();     //Y轴中值光电初始化 PA3
    Yaw_Two_Zero_GPIO_Init(); //副Y轴中值光电初始化 PA2

    /* CAN接口初始化 */
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

    /*PWM初始化*/
    Friction_Init();  //摩擦轮初始化

    #ifdef watch_dog
    /* 看门狗初始化 */
    IWDG_Init(IWDG_Prescaler_64, 320); //半秒多溢出   4， 300
    #endif

    /*初始化完成状态*/
//    LEDB7 = 1;

    delay_ms(100);
}


