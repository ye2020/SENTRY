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
#define PWM_Shoot_Upper_Left   TIM3->CCR1  //PA6
#define PWM_Shoot_Lower_Left   TIM3->CCR2	//PA7
#define PWM_Shoot_Upper_Right  TIM5->CCR3  //PA2
#define PWM_Shoot_Lower_Right  TIM5->CCR4	 //PA3



/**********��������ٶȻ�pid��������**********/
#define FIRE_S_P 12.0f   //�������M2006�ٶȻ�
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

#define FIRE_P_P 50.0f
#define FIRE_P_I 0.1f  //0 TODO
#define FIRE_P_D 0.1f

/**********����ϵͳ�ٶ��趨**********/
//�㼶��15m/s   һ����18m/s  ������22m/s  ������30m/s
#define FRICTION_THREE_SHOOT_SPEED    1070  //	1400				//(2000 - 1215)//   24 m/s// (2000 - 1205) //30Ħ���ָ���pwm			1205 -> 22.5m/s     (1243)
#define FRICTION_TWO_SHOOT_SPEED       	1020					//(2000 - 1193)     //22Ħ���ָ���pwm			
#define FRICTION_ONE_SHOOT_SPEED      1000 //	1298					//(2000 - 1160)//(2000 - 1187)       //18Ħ���ָ���pwm		ԭ1173  13.5m/s			��1187��
#define FRICTION_ZERO_SHOOT_SPEED     1000	//1260					 //(2000 - 1172)       //15Ħ���ֵ���pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define FRICTION_SHOOT_STOP            	800						//	(2000 - 1000)       //0Ħ����ֹͣpwm
#define LOADING_SPEED         (-500)//(-3300)    //��������ٶ�  //5.2 2000����Ϊ2900


#define	LOADING_STOP_TIMES  700



/* �������״̬*/
typedef enum
{
	motor_A_blocking_mode,		// �������A��תģʽ
	motor_B_blocking_mode,		// �������B��תģʽ
	normal_mode,			// ����ģʽ

} Loading_motor_e;



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
    const motor_measure_t *fire_motorA_measure;
		const motor_measure_t *fire_motorB_measure;
    const RC_ctrl_t *fire_RC;   //��������ʹ�õ�ң����ָ��
    PidTypeDef fire_s_pid;      //����2006���pid
		PidTypeDef fire_p_pid;			//����2006���λ��pid

	
		Loading_motor_e Loading_motorStatus; //�������״̬
    Fire_WorkStatus_e fire_workstatus;   //�䵯ģʽ
    Shoot_WorkStatus_e Friction_workstatus;  //Ħ����ģʽ

    int16_t GDA_output;          //����������
		int16_t GDB_output;          //����������
    uint8_t shoot_speed;        //��ǰ��������
    uint8_t dead_mode;          //����ģʽ����
		uint16_t hot_max;						//ǹ����������
		uint16_t hot_current;				//ǹ������
		float GD_speed_gain;				//�������� ����ֵ

} Fire_task_t;



///*ȫ������*/
//extern Fire_task_t Fire_param;

static void snail_motor_pwm(uint32_t snial_pwm);			

extern const Fire_task_t *get_Fire_control_point(void);
extern void FIRE_Task(void *pvParameters);

#endif
