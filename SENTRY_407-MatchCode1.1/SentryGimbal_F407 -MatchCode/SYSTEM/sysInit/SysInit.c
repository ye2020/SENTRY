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

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //����ϵͳ�ж����ȼ����� 4

//	delay_init(180);  //��ʱ������ʼ��

    /*GPIO��ʼ��*/
//	LED_Init();               //����LED��ʼ��
    Laser_Init();             //�����ʼ��

    /* ���ڳ�ʼ�� */
    remote_control_init(); //����һ��ʼ����ң������ʼ����
	
    #if IMU_BMI160
    IMU_control_init(); //���ڶ���ʼ�����������������ݣ�	
    #endif
	
    automatic_aiming_init();  //��������ʼ�����Ӿ�ͨѶ��
//	Usart6_Init();            //����ϵͳ

	/*  �������ڵ��� */
		upper_machine_communication();
		
    /* GPIO��ʼ�� */
    LED_Init();   //����LED��ʼ��

    /* ����ʼ�� */
    Yaw_Zero_GPIO_Init();     //Y����ֵ����ʼ�� PA3
    Yaw_Two_Zero_GPIO_Init(); //��Y����ֵ����ʼ�� PA2

    /* CAN�ӿڳ�ʼ�� */
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

    /*PWM��ʼ��*/
    Friction_Init();  //Ħ���ֳ�ʼ��

    #ifdef watch_dog
    /* ���Ź���ʼ�� */
    IWDG_Init(IWDG_Prescaler_64, 320); //��������   4�� 300
    #endif

    /*��ʼ�����״̬*/
//    LEDB7 = 1;

    delay_ms(100);
}


