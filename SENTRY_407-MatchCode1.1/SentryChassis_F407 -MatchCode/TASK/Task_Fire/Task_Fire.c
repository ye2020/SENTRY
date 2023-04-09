/*==================================================*
*�ļ����ܣ�1.������������ϵͳ������
*         2.����ϵͳ�Ŀ��ƺͷ���Ħ���ֵĿ���
*         3.�������ƣ����ȼ���
*
*��ע��ԓ�ļ���û�������ϵͳ����������
*
*����ģʽ��1.������ģʽ�����õ������������ȼ��е������������Ħ���֣�����ȷ�ȸߡ���糵ʱ����Զ��ʱ�õ���
           2.������ģʽ��Ĭ���������������15m/s�ٶȣ���֤����Ƶ������ȷ�ȵ͡�2.5m��Χ�ڽ�ս�ã�
           3.��Ѫģʽ  ���ر��������ƣ���Ѫ����ȡ�������������Ƶ���п��ٴ���������������������ã�ע�����ٲ�Ҫ���ߣ���Ȼ������������

*==================================================*/

#include "Task_Fire.h"
#include "rmmotor.h"
#include "chassis_behaviour.h"
#include "LED.h"


int fire_heart = 0; //�����������


/*ȫ������*/
Fire_task_t Fire_param;


static void Fire_Control(void);    //����ϵͳ����
static void Fire_param_init(void); //������ʼ��


//���ػ�ؿ��Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const Fire_task_t *get_Fire_control_point(void)
{
    return &Fire_param;
}


/**�������**/
void FIRE_Task(void *pvParameters)
{
    //����ʱ��
    vTaskDelay(FIRE_TASK_INIT_TIME);

    Fire_param_init();

    while(1)
    {
        #ifdef fire_work
        fire_heart = !fire_heart; //�����������
        LEDE3 = fire_heart;

        //�������
        if (Return_Chassis_Mode() == 1 && (Return_RC_S1num() == RC_SW1_UP ||  Return_RC_S1num() == RC_SW1_MID))//1��3
        {
            Fire_Control();
            Laser_ON();
        }
        else //���������
        {
            PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_SHOOT_STOP;
            Fire_param.GD_output = 0;
            Fire_param.shoot_speed = 0;
            Laser_OFF();
        }

        #endif
        //��������
        vTaskDelay(FIRE_CONTROL_TIME_MS);
    }
}


/**
  * @brief          ������ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Fire_param_init(void)
{
    //��ȡ�������ָ��
    Fire_param.fire_motor_measure = get_Fire_Motor_Measure_Point();

    //�䵯ģʽĬ��Ϊͣ��
    Fire_param.fire_workstatus = STOP_FIRE;
    //Ħ����ģʽֹͣģʽ
    Fire_param.Friction_workstatus = STOP_SHOOT;

    pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);

    Fire_param.GD_output = 0;
    Fire_param.shoot_speed = 0;
}


/**
  * @brief          ����ϵͳ����
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Fire_Control(void)
{
    PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_ONE_SHOOT_SPEED;

    //��ȡĦ����ģʽ������
    Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();
    //��ȡ���ģʽ���Ƿ񿪻�
    Fire_param.fire_workstatus = Return_Fire_Mode();

    /* Ħ���ֿ��� */
    if (Fire_param.Friction_workstatus == LOW_SPEED)
    {
        PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_TWO_SHOOT_SPEED;
    }
    else if (Fire_param.Friction_workstatus == HIGH_SPEED)
    {
        PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_THREE_SHOOT_SPEED;
    }
    else if (Fire_param.Friction_workstatus == STOP_SHOOT)
    {
        PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_SHOOT_STOP;
    }
    else
    {
        PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_SHOOT_STOP;
    }

    /* ����������� GD=���� */
    if(Fire_param.fire_workstatus == STOP_FIRE) //ֹͣ����
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else if(Fire_param.fire_workstatus == FIRE) //����
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, -LOADING_SPEED, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else if(Fire_param.fire_workstatus == BACK) //�˵�
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, LOADING_SPEED, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
}


/**
  * @brief          ���ϵͳ�ر�
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Fire_Task_OFF(void)
{
    PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_SHOOT_STOP;
    Fire_param.GD_output = 0;
    Fire_param.shoot_speed = 0;
    Laser_OFF();
}

