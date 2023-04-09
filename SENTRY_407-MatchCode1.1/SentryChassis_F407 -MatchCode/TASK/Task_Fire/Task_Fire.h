#ifndef __FIRETASK_H
#define __FIRETASK_H
#include "main.h"
#include "pid.h"
#include "RemoteControl.h"
#include "CAN_1_Receive.h"


/*OS�������������Լ�����ʱ��*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

/*Ħ����*/
#define PWM_Shoot_Left   TIM4->CCR1  //PD12
#define PWM_Shoot_Right  TIM4->CCR2	 //PD13

/**********��������ٶȻ�pid��������**********/
#define FIRE_S_P 12.0f   //�������M2006�ٶȻ�
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

/**********����ϵͳ�ٶ��趨**********/
//�㼶��15m/s   һ����18m/s  ������22m/s  ������30m/s
#define FRICTION_THREE_SHOOT_SPEED     1226       //30Ħ���ָ���pwm
#define FRICTION_TWO_SHOOT_SPEED       1193       //22Ħ���ָ���pwm
#define FRICTION_ONE_SHOOT_SPEED       1172       //18Ħ���ָ���pwm
#define FRICTION_ZERO_SHOOT_SPEED      1165       //15Ħ���ֵ���pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define FRICTION_SHOOT_STOP            1000       //0Ħ����ֹͣpwm
#define LOADING_SPEED                  (-3000)    //��������ٶ�




/*����ģʽ*/
typedef enum
{
    FIRE,          //����
    AUTO_FIRE,     //�Զ�����
    STOP_FIRE,     //ֹͣ����
    BACK,          //�˵�
    FIRE_ERROR,

} Fire_WorkStatus_e;

/*Ħ����ģʽ*/
typedef enum
{
    LOW_SPEED,
    HIGH_SPEED,
    STOP_SHOOT,
    SHOOT_ERROR,

} Shoot_WorkStatus_e;

/*��ؽṹ��*/
typedef struct Fire_param
{
    const motor_measure_t *fire_motor_measure;
    const RC_ctrl_t *fire_RC;   //��������ʹ�õ�ң����ָ��
    PidTypeDef fire_s_pid;      //����2006���pid

    Fire_WorkStatus_e fire_workstatus;   //�䵯ģʽ
    Shoot_WorkStatus_e Friction_workstatus;  //Ħ����ģʽ

    int16_t GD_output;          //����������
    uint8_t shoot_speed;        //��ǰ��������
    uint8_t dead_mode;          //����ģʽ����

} Fire_task_t;



///*ȫ������*/
//extern Fire_task_t Fire_param;


extern const Fire_task_t *get_Fire_control_point(void);
extern void FIRE_Task(void *pvParameters);
/**
  * @brief          ���ϵͳ�ر�
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Fire_Task_OFF(void);

#endif
