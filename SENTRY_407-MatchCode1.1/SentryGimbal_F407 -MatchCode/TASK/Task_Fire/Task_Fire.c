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
#include "gimbal_behaviour.h"
#include "LED.h"


int fire_heart = 0; //�����������


/*ȫ������*/
Fire_task_t Fire_param;

/*  ���������ת���� */
static  u16 loading_A_stop_count = 0;
static  u16 loading_B_stop_count = 0;
/*  ���������ת���� */
 u16 loading_A_time = 0;
 u16 loading_B_time = 0;
/* ��������������ֵ */
static uint16_t  loading_A_speed = LOADING_SPEED;
static uint16_t  loading_B_speed = -LOADING_SPEED;

 

#define shoot_laser_on()    Laser_ON()      //���⿪���궨��
#define shoot_laser_off()   Laser_OFF()     //����رպ궨��



static void Fire_Control(void);    //����ϵͳ����
static void Fire_param_init(void); //������ʼ��
static void shoot_hot_limit(void); //��������


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
        LEDE3 = fire_heart;		 //�����������

        //�������																	
        if (Return_Gimbal_Mode() == 1 && (Return_RC_S2num() == RC_SW2_UP ||  Return_RC_S2num() == RC_SW2_MID))			//5.10�޸�
        {
            Fire_Control();
            shoot_laser_on();
        }
        else //���������
        {
						PWM_Shoot_Upper_Left  = FRICTION_SHOOT_STOP;
						PWM_Shoot_Lower_Left  = FRICTION_SHOOT_STOP;
						PWM_Shoot_Upper_Right = FRICTION_SHOOT_STOP;
						PWM_Shoot_Lower_Right = FRICTION_SHOOT_STOP;
            Fire_param.GDA_output = 0;
						Fire_param.GDB_output = 0;
            Fire_param.shoot_speed = 0;
            shoot_laser_off();
        }

        #endif
        //��������
        vTaskDelay(FIRE_CONTROL_TIME_MS);
    }
}

//int hot_current;
/*  �������� */
void shoot_hot_limit(void)
{
		uint16_t Residual_power_percentage;			
	
		Fire_param.hot_max 		 = get_shooter_cooling_limit(); //������ 1 �� 17mm ǹ����������
		Fire_param.hot_current = get_shooter_cooling_heat();	//1 �� 17mm ǹ������
//		hot_current = get_shooter_cooling_heat();
	
		Residual_power_percentage = (Fire_param.hot_max - Fire_param.hot_current);
		
		if(Residual_power_percentage > 0 && Residual_power_percentage < 60)
		{
			//����������
			Fire_param.GD_speed_gain = (float)(Residual_power_percentage/ 60.0f)*(Residual_power_percentage / 60.0f);
		}
		
		else 
		{
			//������������
			Fire_param.GD_speed_gain = 1.0f;
		}

}

static void snail_motor_pwm(uint32_t snial_pwm)
{
#if	(REVERES_LIGHT_COUPLING == 1)
	if ((snial_pwm >= 600) && (snail_pwm <= 1600))
		PWM_Shoot_Upper_Left  = PWM_Shoot_Lower_Left  = PWM_Shoot_Upper_Right = PWM_Shoot_Lower_Right = 2111 - snial_pwm;
#else 	
	if ((snial_pwm >= 600) && (snial_pwm) <= 1600)
		PWM_Shoot_Upper_Left  = PWM_Shoot_Lower_Left  = PWM_Shoot_Upper_Right = PWM_Shoot_Lower_Right = snial_pwm;
	
#endif	
		
	
}

/**������ʼ��**/
static void Fire_param_init(void)
{
		//��ȡ�������ָ��
    Fire_param.fire_motorA_measure = Get_Fire_MotorA_Measure_Point();
    Fire_param.fire_motorB_measure = Get_Fire_MotorB_Measure_Point();

    //�䵯ģʽĬ��Ϊͣ��
    Fire_param.fire_workstatus = STOP_FIRE;
    //Ħ����ģʽֹͣģʽ
    Fire_param.Friction_workstatus = STOP_SHOOT;

    pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);
		pid_init(&Fire_param.fire_p_pid, FIRE_P_P, FIRE_P_I, FIRE_P_D, 0, 0);

    Fire_param.GDA_output = 0;
	  Fire_param.GDB_output = 0;
    Fire_param.shoot_speed = 0;
	snail_motor_pwm(FRICTION_SHOOT_STOP);

}


/**����ϵͳ����**/
static void Fire_Control(void)
{		
		//��������
		shoot_hot_limit();
    //��ȡĦ����ģʽ������
    Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();				// 5.10 Ϊ�ⵥ����̨Ħ�����޸�
    //��ȡ���ģʽ���Ƿ񿪻�
    Fire_param.fire_workstatus = Return_Fire_Mode();

    /* Ħ���ֿ��� */
    if (Fire_param.Friction_workstatus == LOW_SPEED)
    {
//		snail_motor_pwm(FRICTION_ONE_SHOOT_SPEED);
				PWM_Shoot_Upper_Left  = FRICTION_ONE_SHOOT_SPEED;//4.23���Դ򵯹ص�һ��
				PWM_Shoot_Lower_Left  = FRICTION_ONE_SHOOT_SPEED;
				PWM_Shoot_Upper_Right = FRICTION_ONE_SHOOT_SPEED;
				PWM_Shoot_Lower_Right = FRICTION_ONE_SHOOT_SPEED;
    }
    else if (Fire_param.Friction_workstatus == HIGH_SPEED)
    {
//		snail_motor_pwm(FRICTION_THREE_SHOOT_SPEED);
				PWM_Shoot_Upper_Left  = FRICTION_THREE_SHOOT_SPEED;
				PWM_Shoot_Lower_Left  = FRICTION_THREE_SHOOT_SPEED;
				PWM_Shoot_Upper_Right = FRICTION_THREE_SHOOT_SPEED;
				PWM_Shoot_Lower_Right = FRICTION_THREE_SHOOT_SPEED;
    }
    else if (Fire_param.Friction_workstatus == STOP_SHOOT)
    {
		snail_motor_pwm(FRICTION_SHOOT_STOP);
//				PWM_Shoot_Upper_Left  = FRICTION_SHOOT_STOP;
//				PWM_Shoot_Lower_Left  = FRICTION_SHOOT_STOP;
//				PWM_Shoot_Upper_Right = FRICTION_SHOOT_STOP;
//				PWM_Shoot_Lower_Right = FRICTION_SHOOT_STOP;
    }
    else
    {
//		snail_motor_pwm(FRICTION_SHOOT_STOP);
				PWM_Shoot_Upper_Left  = FRICTION_SHOOT_STOP;
				PWM_Shoot_Lower_Left  = FRICTION_SHOOT_STOP;
				PWM_Shoot_Upper_Right = FRICTION_SHOOT_STOP;
				PWM_Shoot_Lower_Right = FRICTION_SHOOT_STOP;
    }

    /* ����������� GD=���� */
    if(Fire_param.fire_workstatus == STOP_FIRE) //ֹͣ����
    {
        Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
				Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
		
		
    else if(Fire_param.fire_workstatus == FIRE) //����
    {		
			/*  ���������ת�ٵ���һ��ֵ */
			if(Fire_param.fire_motorA_measure->speed >= (LOADING_SPEED + 700)&& Fire_param.fire_motorA_measure->speed <= (-(LOADING_SPEED + 700)))   
			{	
				/*  ��ת�����ת����ֵ */
				loading_A_stop_count++;
				
				/*  ��ת����ֵ����һ���� ����¼��ת���� +1 */
				if(loading_A_stop_count >= LOADING_STOP_TIMES)
				{
					loading_A_time++;
					loading_A_stop_count = 0;
				}
			}
			
			if(Fire_param.fire_motorB_measure->speed >= (LOADING_SPEED + 700)&& Fire_param.fire_motorB_measure->speed  <= (-(LOADING_SPEED + 700)))
			{
				loading_B_stop_count++;
				
				if(loading_B_stop_count >= LOADING_STOP_TIMES)
				{
					loading_B_time++;
					loading_B_stop_count = 0;
				}
			}
			
				if(loading_A_time % 2 != 0 )
						 loading_A_speed = (-LOADING_SPEED);
				else loading_A_speed =  LOADING_SPEED;
				
				if(loading_B_time % 2 != 0 )
						 loading_B_speed = (LOADING_SPEED);
				else loading_B_speed =  (-LOADING_SPEED);
				
	
	      Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_A_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
				Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_B_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
			
			}
					

    else if(Fire_param.fire_workstatus == BACK) //�˵�
    {
        Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, LOADING_SPEED, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
				Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, -LOADING_SPEED, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else
    {
        Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
				Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
}
