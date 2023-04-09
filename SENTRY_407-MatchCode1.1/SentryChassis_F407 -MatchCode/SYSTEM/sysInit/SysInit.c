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
  * @brief          Ӳ�����ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
void System_Init(void)
{
    taskENTER_CRITICAL();  //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���

    delay_ms(10);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //����ϵͳ�ж����ȼ����� 4

	
    /* ���ڳ�ʼ�� */
    Remote_Control_Init();      //����һ��ʼ����ң������ʼ����
    //Capacitance_Control_Init(); //���ڶ���ʼ�����볬�����ݿ��ư�ͨѶ��
    referee_system_init();      //��������ʼ��������ϵͳ��

	
    /* GPIO��ʼ�� */
    LED_Init();               //����LED��ʼ��
		Laser_GPIO_Init();				//���������䴫����



    /* CAN�ӿڳ�ʼ�� */
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

		
		
		/*�������ʼ��*/
		RNG_Init();
		
		
		/*��ģת����ʼ��*/
		Adc_Init();
		
    /* ���Ź���ʼ�� */
//    IWDG_Init(IWDG_Prescaler_64, 625);   //1s


    /*��ʼ�����״̬*/
    LEDB7 = 1;

    delay_ms(100);

    taskEXIT_CRITICAL();             //�˳��ٽ���
}



/**
  * @brief          ʧ�ر���
  * @param[in]      none
  * @retval         none
  * @attention
  */
void SYSTEM_OutCtrlProtect(void)
{
    //SYSTEM_Reset();      //ϵͳ�ָ�������״̬
    Remote_reload();      //ң�����ݻָ���Ĭ��״̬
    Laser_OFF();          //�����
    Chassis_Task_OFF(2);  //���̹�
    //Magazine_StopCtrl();  //���ֹͣת��
    Fire_Task_OFF();      //�������ر�
    SuperCap_OFF();       //���ݹرճ�ŵ�
}


