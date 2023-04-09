/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      ��ɵ��̿�������
  ******************************************************************************
  */
#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H
#include "main.h"
#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "Task_Fire.h"
#include "maths.h"
#include "pid.h"


/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

//���̲���ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define CHASSIS_TEST_MODE   1

//���̹������� Ҫ�þ���Ϊ1
#define power_limit  1

/**********************���̵��pid����**********************/
#define CHASSIS_MOTOR1_PID_Kp    8.0f
#define CHASSIS_MOTOR1_PID_Ki    2.0f
#define CHASSIS_MOTOR1_PID_Kd    0.0f

#define CHASSIS_MOTOR2_PID_Kp    8.0f
#define CHASSIS_MOTOR2_PID_Ki    2.0f
#define CHASSIS_MOTOR2_PID_Kd    0.0f

#define CHASSIS_MOTOR3_PID_Kp    8.0f
#define CHASSIS_MOTOR3_PID_Ki    0.0f
#define CHASSIS_MOTOR3_PID_Kd    0.2f

#define CHASSIS_MOTOR4_PID_Kp    8.0f
#define CHASSIS_MOTOR4_PID_Ki    0.0f
#define CHASSIS_MOTOR4_PID_Kd    0.2f

#define CHASSIS_LOCATION_PID_P  100.0f
#define CHASSIS_LOCATION_PID_I  10.0f
#define CHASSIS_LOCATION_PID_D  10.8f

#define CHASSIS_ROTATE_FOLLOW_P  10.0f   //���̾�ֹ����PID   8.0
#define CHASSIS_ROTATE_FOLLOW_I  0.0f   //0.01
#define CHASSIS_ROTATE_FOLLOW_D  1.8f   //5.02   10.02

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���  |  0.26f  |  0.0097f

/**********************�������Ʋ���**********************/
#define PowerLimit_Param  6.5f         //���������������
#define PowerLimit_Thres  190.0f        //����������ֵ�����幦��ʣ������50.0f

/**********************�˶����ٶ�����**********************/
#define STRAIGHT_ACCELERAD        3.5f      //ֱ�е��̼��ٶ�����
#define TRANSLATION_ACCELERAD     5.5f      //ƽ�Ƶ��̼��ٶ�����
#define ROTATING_ACCELERAD        19.0f     //��ת���̼��ٶ�����

#define CHASSIS_ROTATION_SPEED    600      //С���ݵ���ת�ٶ�  2000
#define CHASSIS_ROTATION_MOVE_SPEED  600   //С�����ƶ�ʱΪ��ֹ�켣ʧ���ת��   1700
#define CHASSIS_TWIST_SPEED       600      //Ť���ٶ�  1600
#define CHASSIS_AUTO_SPPED				7000			 //�Զ��ٶ�  3000					
#define CHASSIS_BLOCKING_SPPED		6700 //6700//	900			 //��λ�ٶ�  7000
#define CHASSIS_AUTO_MAX_SPEED		4800			 //�Զ�����  4800
#define CHASSIS_AUTO_SLOW_SPPED				0			// �������� �ٶȼ���		5.17   3000->0  5.25


#define NORMAL_FORWARD_BACK_SPEED 	300.0f   //������ֱͨ���ٶ�
#define NORMAL_LEFT_RIGHT_SPEED   	300.0f   //������ͨƽ���ٶ�

#define HIGH_FORWARD_BACK_SPEED 	600.0f     //���̼���ֱ���ٶ�
#define HIGH_LEFT_RIGHT_SPEED   	600.0f     //���̼���ƽ���ٶ�

#define SPEED_TIME_FLAG				430	//510			//����ʱ��
#define SPEED_MODE						  	4//���̱��٣�1 -> �����٣� 2 -> ����  ; 3 -> ���� + ����1 ;4 -> ����2
/*����ģʽ*/
typedef enum
{
    CHASSIS_STOP,       //ֹͣ
    CHASSIS_INITIALIZE, //��ʼ����
    CHASSIS_STANDBY,    //����

    //CHASSIS_WORKING,   //����

    CHASSIS_FOLLOW,    //����
    CHASSIS_NO_FOLLOW, //������

    CHASSIS_TWIST_WAIST, //Ť��
    CHASSIS_ROTATION,    //С����
    CHASSIS_BATTERY,     //��̨ģʽ

    CHASSIS_REMOTECONTROL,//ң��ģʽ
    CHASSIS_AUTO,				   //�Զ�ģʽ
    CHASSIS_BLOCKING,      //��λģʽ
    CHASSIS_FIXATION			// ���̶̹�ģʽ
} Chassis_mode_e;


/*������ɫ*/
typedef enum
{
    Enemy_color_red = 0,    //������ɫΪ��ɫ
    Enemy_color_blue		//������ɫΪ��ɫ

} Enemy_color_e;
/*���̰����ṹ��*/
typedef struct
{
    float KEY_W;
    float KEY_S;
    float KEY_A;
    float KEY_D;
    u8 KEY_SHIFT;
    u8 KEY_CTRL;
    u8 KEY_Q;   //��������
    u8 KEY_E;   //��������
    u8 KEY_R;   //С����
    u8 KEY_F;   //Ť��
    u8 KEY_G;   //����ģʽ
    u8 KEY_Z;   //�˵�
    u8 KEY_X;   //
    u8 KEY_C;   //������
    u8 KEY_V;   //������
    u8 KEY_B;   //
    u8 Mouse_L; //����
    u8 Mouse_R; //�Զ���׼
    u16 Mouse_X; //��̨����
    u16 Mouse_Y; //��̨����
} Key_Pressed_Sigh_t;

/*���̵������*/
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 position;
    fp32 last_position;
    int16_t output;
} Motor_t;

/*���̹�������*/
typedef struct Power_Limit
{
    float Real_Power[3]; //2���� 1�ϴ� 0���ϴ�
    float RemainPower[3];
    int32_t SumOutValue;
    int32_t LimitOutValue;
    float scale;
} PowerLimit_t;

/*�����������ݽṹ��*/
typedef struct
{
    const RC_ctrl_t *chassis_RC;             //����ʹ�õ�ң����ָ��
    const gimbal_yaw_receive_t *gimbal_re_data; //��̨�崦�����ݣ�������can2��������̰壬���̰��������yaw����
    const Fire_task_t *Fire_task_control;
    motor_measure_t *yaw_motor_measure;      //can1ֱ�ӽ��յ�yaw������
    motor_measure_t *chassis_motor_measure;
    Key_Pressed_Sigh_t  Key_Press;

    PowerLimit_t Chassis_PowerLimit; //���̹������ƽṹ��
    Motor_t chassis_motor[4];        //���̵������(�������ͳһ�ṹ��ָ��)
    PidTypeDef chassis_speed_pid[4]; //�����ƶ�pid
    PidTypeDef chassis_location_pid; //����λ�û�pid
    PidTypeDef chassis_rotate_pid;   //��תpid

    Chassis_mode_e chassis_mode; //���̿���״̬��

    first_order_filter_type_t LowFilt_chassis_vx; //��ͨ�˲���
    first_order_filter_type_t LowFilt_chassis_vy; //��ͨ�˲���

//	fp32 speed_x;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
//	fp32 speed_y;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
//	fp32 speed_z;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
    fp32 speed_x_set; //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
    fp32 speed_y_set; //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
    fp32 speed_z_set; //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

    fp32 chassis_gimbal_angel; //��������̨�ĽǶ�

    uint8_t SuperCap_discharge_flag;  //�������ݷŵ��־λ
    uint8_t Chassis_keyboad;

    u8 sign;								//ǰ���߱�־
    u8 sign_last;						//����ǰ���߱�־

//	fp32 max_speed_x;   //ǰ����������ٶ� ��λm/s
//	fp32 min_speed_x;   //ǰ��������С�ٶ� ��λm/s
//	fp32 max_speed_y;   //���ҷ�������ٶ� ��λm/s
//	fp32 min_speed_y;   //���ҷ�����С�ٶ� ��λm/s

//	fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
//	fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
//	fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_control_t;


//����������
extern void CHASSIS_Task(void *pvParameters);
//���̿���������
extern void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2);

extern u8 Return_Chassis_Mode(void);

void Chassis_Task_OFF(u8 options);

uint8_t  automatic_Enemy_color(void);


#endif
