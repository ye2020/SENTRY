#include "gimbal_behaviour.h"
#include "Task_Gimbal.h"
#include "maths.h"
#include "rmmotor.h"
#include "filter.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "IMU.h"
#include "photoelectric.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "Task_Start.h"
#include "automatic_strike.h"
#include "Task_Fire.h"

//��̨��ʼ��
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f);
//��̨ģʽѡ��
static void Gimbal_Mode_Choose(gimbal_control_t *fir_gimbal_mode_choose_f);
//��̨�ֶ�ģʽ
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f);
//��̨�Զ���׼
static void Gimbal_Auto(gimbal_control_t *gimbal_auto_f);
//��̨�Զ�Ѳ��
static void Gimbal_Patrol(gimbal_control_t *gimbal_Auto_f);
//����ģʽ
static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f);
	
VisionStatus_E  Last_VisionStatus;		//������¼��һ�ε��Ӿ�״̬
VisionStatus_E  VisionStatus = Enemy_Disappear;

WorkStatus_E	GimbalStatus;
WorkStatus_E  Now_GimbalMode;							
WorkStatus_E  Last_GimbalMode;				//������¼��һ�ε���̨ģʽ

static u16 vision_status_count = 0;

//static
float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f; //��̨����ܿ���
u8 Friction_wheel_mode = STOP_SHOOT;
u8 Fire_mode = STOP_FIRE;


/**
  * @brief          ��ʼ������
  * @param[in]      fir_gimbal_behaviour_f
  * @retval         none
  * @attention
  */
void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f)
{		
	
    //!ģʽѡ��
    Gimbal_Mode_Choose(fir_gimbal_behaviour_f);
	
    //��Ϊ��ʼ��
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_INITIALIZE)  //��̨״̬Ϊ��ʼ��
    {
        Gimbal_Location_Init(fir_gimbal_behaviour_f); //��̨��ʼ��
    }
		
    //��Ϊͣ��ģʽ
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_STOP || fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_STANDBY)
    {
        Remote_reload();       //ҡ��������
        Gimbal_Stop(fir_gimbal_behaviour_f); //ֹͣ
    }
		
    //��Ϊң��ģʽremotecontrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_REMOTECONTROL)
    {
				Gimbal_RemoteControl(fir_gimbal_behaviour_f);
    }
				
    //��Ϊ����ģʽautomatic
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_AUTOATTACK)
    {
//				Gimbal_Auto(fir_gimbal_behaviour_f);
//			
				Gimbal_AutoControl(fir_gimbal_behaviour_f);
				Friction_wheel_mode = STOP_SHOOT;
				Fire_mode = STOP_FIRE;
    }
		
    //��ΪѲ��ģʽpatrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_PATROl)
    {
				Gimbal_Patrol(fir_gimbal_behaviour_f);
    }
		
    //��Ϊ����ģʽautocontrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_AUTOCONTROL)
    {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
    }
}

/*ң����ָ��
- �ұ�����Ϊ��Ҫ��λ��������ͣ�������м���ң�أ����������Զ�
- ��ң�ص�λ����ҡ�˿�����̨����ҡ�˿��Ƶ��̣���ҡ�˴���Ϊ������̨����Ħ���֣���ҡ�˴���Ϊ����̨��Ħ��������Ϊ�����٣���ҡ�˴���Ϊ����̨��Ħ��������Ϊ������
- ���Զ���λ����ߵ�λΪ���ϲ��Ǳ���ģʽ������Ϊ�Զ�������Ϊ��λ�����������������ԣ�����Ϊ����ģʽ*/

/**
  * @brief          ��̨ģʽѡ��
  * @param[in]      fir_gimbal_mode_choose_f
  * @retval         none
  * @attention
  */
static void Gimbal_Mode_Choose(gimbal_control_t *fir_gimbal_mode_choose_f)
{

    //�ҿ��ش��ϣ��Զ�����
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_UP)
    {
        //�ոտ����������ȴ��£�Ȼ����ܿ���
        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STOP)
        {
            Remote_reload();       //ҡ��������
            Gimbal_Stop(fir_gimbal_mode_choose_f);
        }

        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STANDBY) //֮ǰ��״̬Ϊ����
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_INITIALIZE; //״̬����Ϊ��ʼ��
            Remote_reload();      //ҡ��������
        }

        if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_DOWN)
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour =GIMBAL_STANDBY ; //Ѳ��ģʽ(��ʱΪ����)
        }
				if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_MID)
				{
						fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_AUTOATTACK;  //�Զ�����ģʽ
				}
				if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 ==  RC_SW1_UP)
				{
						fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_AUTOCONTROL;  //����ģʽ

            if(fir_gimbal_mode_choose_f->auto_c->auto_pitch_angle == 0.0f && fir_gimbal_mode_choose_f->auto_c->auto_yaw_angle == 0.0f)
            {
                vision_status_count++;
            }
            else
            {
                vision_status_count = 0;
                VisionStatus = Enemy_Appear;		//���˳���
							
            }

            if(vision_status_count >= ENEMY_DISAPPEAR_TIMES)
            {
                vision_status_count = ENEMY_DISAPPEAR_TIMES;
                VisionStatus = Enemy_Disappear;		//������ʧ
							

            }
				}
    }

    //�ҿ��ش��У�ң�ؿ���
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_MID)
    {
        //�ոտ����������ȴ����£�Ȼ����ܿ���
        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STOP)
        {
            Remote_reload();       //ҡ��������
            Gimbal_Stop(fir_gimbal_mode_choose_f);
        }

        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STANDBY) //֮ǰ��״̬Ϊ����
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_INITIALIZE; //״̬����Ϊ��ʼ��
            Remote_reload();      //ҡ��������
        }

        fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_REMOTECONTROL; //״̬����Ϊ�ֶ�

        if(fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_DOWN)
        {
            Friction_wheel_mode = STOP_SHOOT;
        }

        if(fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_MID)
        {
            Friction_wheel_mode = LOW_SPEED;
        }

        if(fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_UP)
        {
            Friction_wheel_mode = HIGH_SPEED;
        }
            
        
     }

    //�ҿ��ش��£�ֹͣ����
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_DOWN)
    {
        Gimbal_Stop(fir_gimbal_mode_choose_f); //��̨����
        Remote_reload();       //ҡ��������
        fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_STANDBY; //�������ģʽ
        fir_gimbal_mode_choose_f->Gimbal_all_flag = 0;   //��ʼ����־λ����
    }

    //�������ش���
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_ERROR)
    {
        fir_gimbal_mode_choose_f->Gimbal_all_flag = 0;  //��ʼ����־λ����
        Remote_reload();      //ҡ��������
    }
}



/**
  * @brief          ��̨��ʼ��
  * @param[in]      fir_gimbal_location_init_f
  * @retval         none
  * @attention
  */
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f)
{
    //��������ʼ���Ļ�ֱ�Ӽ��Ͼͺ��ˣ����õ�ʱ��ֱ��ע�͵�
//	  fir_gimbal_location_init_f->yaw_c.init_flag = 1;
//    fir_gimbal_location_init_f->pitch_c.init_flag = 1;
//    fir_gimbal_location_init_f->sec_gimbal_control.init_flag = 1;
    //����������ʼ���ɹ�
//    if (fir_gimbal_location_init_f->yaw_c.init_flag == 1 && fir_gimbal_location_init_f->pitch_c.init_flag == 1 && fir_gimbal_location_init_f->sec_gimbal_control.init_flag == 1)
//    {
    fir_gimbal_location_init_f->yaw_c.init_flag = 0;
    fir_gimbal_location_init_f->pitch_c.init_flag = 0;
    fir_gimbal_location_init_f->sec_gimbal_control.init_flag = 0;
    fir_gimbal_location_init_f->Gimbal_all_flag = 1;
    fir_gimbal_location_init_f->gimbal_behaviour = GIMBAL_WORKING;
//    }
}


/**
  * @brief          ��̨ң��ģʽ
  * @param[in]      gimbal_remotecontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f)
{

    Gimbal_ch2 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[2]) * RC_YAW_SPEED * 0.2f;  //Y��λ�û����ۼ�   RC_YAW_SPEED
    Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f); //ѭ���޷�
    Gimbal_ch3 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[3]) * RC_PITCH_SPEED * 0.09f;  //P��λ�û����ۼ�  RC_PITCH_SPEED

    //yaw�Ƕ�����     -180~180
//    Gimbal_ch2 = float_limit(Gimbal_ch2, YAW_ANGLE_LIMIT, -YAW_ANGLE_LIMIT);
    //pitch�Ƕ�����   0 ~ -85  (�ڱ��ϸ�����)
    Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);

    if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //������  ����
    }
    else if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] <= -500)
    {
        Fire_mode = BACK;       //������  ����
    }
    else
    {
        Fire_mode = STOP_FIRE;  //�����м�  ֹͣ����
    }


    Gimbal_Manual_Work(gimbal_remotecontrol_f, Gimbal_ch2, Gimbal_ch3);
}


/**
  * @brief          ��̨�Զ���׼
  * @param[in]      gimbal_auto_f
  * @retval         none
  * @attention
  */
static void Gimbal_Auto(gimbal_control_t *gimbal_auto_f)
{
    //Change_camera(1); //����ģʽ���л�����ͷ����Ϊ�̽����

    /*Ԥ����*/
//    gimbal_auto_f->yaw_c.angle = gimbal_auto_f->yaw_c.yaw_motor_measure->angle - gimbal_auto_f->yaw_c.last_angle; //��ȡ��̨�Զ����ʱ��������Y��Ƕ�
//	Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);  //��С���Դ���ǶȽ��п������˲�
		if( VisionStatus == Enemy_Disappear )
		{
				gimbal_auto_f->pitch_c.Auto_record_location = (gimbal_auto_f->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024);
		}

    /*��̨�Ӿ�������*/    //(y�ᣬp�ỹû�ÿ�����)
    gimbal_auto_f->auto_c->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 4.0f;
    gimbal_auto_f->auto_c->yaw_control_data = gimbal_auto_f->auto_c->auto_yaw_angle * 4.0f;

		gimbal_auto_f->auto_c->pitch_control_data = float_limit(gimbal_auto_f->auto_c->pitch_control_data, 5.0f*PITCH_ANGLE_LIMIT_UP , 25.0f*PITCH_ANGLE_LIMIT_DOWN);
		gimbal_auto_f->auto_c->yaw_control_data   = float_limit(gimbal_auto_f->auto_c->yaw_control_data , YAW_ANGLE_LIMIT , -YAW_ANGLE_LIMIT);
//			/* P���˲� */
//		Data_Accelerated_Control(&gimbal_auto_f->auto_c->pitch_control_data, 3.6f); ////б�¼��ٶ����ƺ���
//		gimbal_auto_f->auto_c->pitch_control_data = Sliding_Mean_Filter(&gimbal_auto_f->pitch_c.Slidmean_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data, 45); //��ֵ�����˲������ͺ�
//		gimbal_auto_f->auto_c->pitch_control_data = first_order_filter(&gimbal_auto_f->pitch_c.LowFilt_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data);       //һ�׵�ͨ�˲�
		
    Gimbal_Automatic_Work(gimbal_auto_f);
}



/**
  * @brief          ��̨����
  * @param[in]      gimbal_stop_f
  * @retval         none
  * @attention
  */
void Gimbal_Stop(gimbal_control_t *gimbal_stop_f)
{
    //������
    gimbal_stop_f->pitch_c.output = 0;
    gimbal_stop_f->yaw_c.output = 0;
    gimbal_stop_f->sec_gimbal_control.output = 0;

    //���������
    gimbal_stop_f->auto_c->pitch_control_data = 0;
    gimbal_stop_f->auto_c->yaw_control_data = 0;
	
		Friction_wheel_mode = STOP_SHOOT;  

    Gimbal_ch3 = 0.0f;
    Gimbal_ch2 = 0.0f;
}




/**
  * @brief          ��̨Ѳ��
  * @param[in]      *gimbal_Patro_f
  * @retval         none
  *///����yawͨ��4.24
static uint32_t yaw_direction = RIGHT ;
static uint32_t pitch_direction = PITCH_UP ;
float Auto_Yaw_Angle_Target = 0.0f;
float Auto_Pitch_Angle_Target = 0.0f;
static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f)
{
#if yaw_angle_limit	
		/*Y��*//*------------------  �ⲿ��ע�͵���yaw�����360�� -----------------------------*/
	
		if(gimbal_Patro_f->yaw_c.yaw_motor_measure->yaw_angle >= YAW_ANGLE_LIMIT )								
		{
				yaw_direction = LEFT;
		}
		else if(gimbal_Patro_f->yaw_c.yaw_motor_measure->yaw_angle <= -YAW_ANGLE_LIMIT)
		{
				yaw_direction = RIGHT;
		}
		
/*-----------------------------------------------------------------------------------------*/		
#endif		
		if(yaw_direction == LEFT)
		{
				Auto_Yaw_Angle_Target -=0.2f;   //������̨�Զ����ٶ�
		}
		if(yaw_direction == RIGHT)
		{
				Auto_Yaw_Angle_Target +=0.2f;
		}
		
		/*P��*/
		if(gimbal_Patro_f->pitch_c.pitch_motor_measure->pitch_angle >= PITCH_ANGLE_LIMIT_UP)
		{
				pitch_direction = PITCH_DOWN;
		}
		else if(gimbal_Patro_f->pitch_c.pitch_motor_measure->pitch_angle <= PITCH_ANGLE_LIMIT_DOWN)
		{
				pitch_direction = PITCH_UP;
		}
		
		if(pitch_direction == PITCH_UP)
		{
				Auto_Pitch_Angle_Target += 0.2f;
		}
		if(pitch_direction == PITCH_DOWN)
		{
				Auto_Pitch_Angle_Target -= 0.2f;
		}
		Gimbal_Manual_Work(gimbal_Patro_f , Auto_Yaw_Angle_Target , Auto_Pitch_Angle_Target);
}

/**
  * @brief          ����ģʽ(����+Ѳ��)
  * @param[in]      gimbal_autocontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f)
{

    if(  VisionStatus == Enemy_Disappear )
    {
        Gimbal_Patrol(gimbal_autocontrol_f);
				Fire_mode = STOP_FIRE;
				Last_VisionStatus = Enemy_Disappear;
    }

    if(  VisionStatus == Enemy_Appear )
    {
        Gimbal_Auto(gimbal_autocontrol_f);
				Fire_mode = FIRE;
				Last_VisionStatus = Enemy_Appear;
    }
		Friction_wheel_mode = HIGH_SPEED;
}

/**
  * @brief          ����Ħ����ģʽ
  * @param[in]      none
  * @retval         Friction_wheel_mode
  * @attention
  */
Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void)
{
//	LOW_SPEED,   //������
//	HIGH_SPEED,  //������
//	STOP_SHOOT,  //ֹͣ
    if (Friction_wheel_mode == LOW_SPEED)
        return LOW_SPEED;

    if (Friction_wheel_mode == HIGH_SPEED)
        return HIGH_SPEED;

    if (Friction_wheel_mode == STOP_SHOOT)
        return STOP_SHOOT;

    return SHOOT_ERROR;
}

/**
  * @brief          ���ط���ģʽ
  * @param[in]      none
  * @retval         Fire_mode
  * @attention
  */
Fire_WorkStatus_e Return_Fire_Mode(void)
{
//	FIRE,          //����
//	AUTO_FIRE,     //�Զ�����
//	STOP_FIRE,     //ֹͣ����
//	BACK,          //�˵�
    if (Fire_mode == FIRE)
        return FIRE;

    if (Fire_mode == AUTO_FIRE)
        return AUTO_FIRE;

    if (Fire_mode == STOP_FIRE)
        return STOP_FIRE;

    if (Fire_mode == BACK)
        return BACK;

    return FIRE_ERROR;
}



