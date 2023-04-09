/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      ����״̬����
******************************************************************************
*/
#include "chassis_behaviour.h"
#include "Task_Chassis.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "sensor.h"
#include "rng.h"
#include "rmmotor.h"
#include "CAN_2_receive.h"
#include "RefereeDeal.h"

static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f); 		 //����ģʽѡ��
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f); //����ң��
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f);							 		 //�����Զ�ģʽ
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f);					 //������λģʽ
static int16_t Yaw_Angle_Limit(int16_t M3508_data);														 //���̵�����̴���

static int32_t real_round;
/*-------������--------*/
static int32_t register1;//��������Ϊ���Զ���λ�µļ���
static int32_t register2;
static int32_t register3;
static int32_t register4;
/*-------������--------*/
static int32_t state;
static int32_t flag = 0;
float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //���̵���ܿ���
int8_t Vision_flag = 0;             //�Ӿ�ģʽ�л���־��0���رգ�1�����飬2���������أ�



u8 Friction_wheel_mode = STOP_SHOOT;
u8 Fire_mode = STOP_FIRE;
int chassis_cos_calculate_jscope = 0;
int chassis_sin_calculate_jscope = 0;


/**
  * @brief          ����ģʽ����
  * @param[in]      *chassis_pid_f���������ṹ��
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f)
{
    //!����ģʽѡ��
    chassis_mode_choose(chassis_behaviour_f);

    //*�Զ�ģʽ
    if (chassis_behaviour_f->chassis_mode == CHASSIS_AUTO)
    {
        Chassis_Auto(chassis_behaviour_f);
    }

    //*ң��ģʽ
    if (chassis_behaviour_f->chassis_mode == CHASSIS_REMOTECONTROL)
    {
        Chassis_RemoteControl(chassis_behaviour_f);
    }

    //*��λģʽ
    if (chassis_behaviour_f->chassis_mode == CHASSIS_BLOCKING)
    {
        Chassis_Blocking(chassis_behaviour_f);
    }

    //*ͣ��ģʽ
    if (chassis_behaviour_f->chassis_mode == CHASSIS_STOP)
    {
        Chassis_Stop(chassis_behaviour_f);
    }

    //*����ֵ����
    chassis_set_remote(chassis_behaviour_f, Chassis_ch0, Chassis_ch1, Chassis_ch2);
}

/*ң����ָ��
- �ұ�����Ϊ��Ҫ��λ��������ͣ�������м���ң�أ����������Զ�
- ��ң�ص�λ����ҡ�˿�����̨����ҡ�˿��Ƶ��̣���ҡ�˴���Ϊ������̨����Ħ���֣���ҡ�˴���Ϊ����̨��Ħ��������Ϊ�����٣���ҡ�˴���Ϊ����̨��Ħ��������Ϊ������
- ���Զ���λ����ߵ�λΪ���ϲ��Ǳ���ģʽ������Ϊ�Զ�������Ϊ��λ�����������������ԣ�����Ϊ����ģʽ*/

/**
  * @brief          ����ģʽѡ��
  * @param[in]      *chassis_mode_choose_f���������ṹ��
  * @retval         none
  */
static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f)
{

    /* �ҿ��ش��ϣ��������ģʽ */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_UP)
    {
        /* �ж�֮ǰ��״̬��ʲô */
        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STOP) //֮ǰ��״̬Ϊ����
        {
            Remote_reload(); //ҡ��������
            Chassis_Stop(chassis_mode_choose_f);
        }

        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STANDBY) //֮ǰ��״̬Ϊ����
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_INITIALIZE; //״̬����Ϊ��ʼ��
            Remote_reload();                                          //ҡ��������
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_UP)//ע�� ��������߲��˴���
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_AUTO;					//�����˶�
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_DOWN)  //ע�� ��������߲��˴���
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_REMOTECONTROL;
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_MID) //ע�� ��������߲��˴���
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_AUTO;//CHASSIS_BLOCKING;		6.2����
        }
    }

    /* �ҿ��ش��У�ң�ؿ��� */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_MID)
    {
        /* �ж�֮ǰ��״̬��ʲô */
        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STOP) //֮ǰ��״̬Ϊ����
        {
            Remote_reload(); //ҡ��������
            Chassis_Stop(chassis_mode_choose_f);
        }

        chassis_mode_choose_f->chassis_mode = CHASSIS_REMOTECONTROL;
    }

    /* �ҿ��ش��£�ֹͣ���� */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_DOWN)
    {
        chassis_mode_choose_f->chassis_mode = CHASSIS_STANDBY; //�������״̬
        Remote_reload();                                        //ҡ��������
    }

    /* �������ش��� */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_ERROR)
    {
        chassis_mode_choose_f->chassis_mode = CHASSIS_STANDBY; //�������ģʽ
        //Remote_reload();                                        //ҡ��������
    }
}


/**
  * @brief          ����ң��
  * @param[in]      *Chassis_Independent_f���������ṹ��
  * @retval         none
  */
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{

    Chassis_ch1 = Chassis_RemoteControl_f->chassis_RC->rc.ch[1];
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[1];

    if (Chassis_RemoteControl_f->chassis_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //������  ����
        Friction_wheel_mode = LOW_SPEED;
    }
    else if (Chassis_RemoteControl_f->chassis_RC->rc.ch[4] <= -500)
    {
        Fire_mode = FIRE;       //������  ����
        Friction_wheel_mode = LOW_SPEED; //Ħ����ģʽ����ģʽ
    }
    else
    {
        Fire_mode = STOP_FIRE;  //�����м�  ֹͣ����
        Friction_wheel_mode = LOW_SPEED; //Ħ����ģʽ����ģʽ
    }
}


/**
  * @brief          ��������
  * @param[in]      *Chassis_Stop_f���������ṹ��
  * @retval         none
  */
void Chassis_Stop(chassis_control_t *Chassis_Stop_f)
{
    //*������
    Chassis_Stop_f->chassis_motor[0].output = 0;
    Chassis_Stop_f->chassis_motor[1].output = 0;
    Chassis_Stop_f->chassis_motor[2].output = 0;
    Chassis_Stop_f->chassis_motor[3].output = 0;
    Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0;
    Fire_mode = STOP_FIRE;
    Friction_wheel_mode = STOP_SHOOT;
}

/**
  * @brief          �����Զ�
  * @param[in]      *Chassis_Auto_f���������ṹ��
  * @retval         none
  */

VisionStatus_E  Enemy_status = Enemy_Disappear;			//���˳���״̬

static uint16_t outpost_HP;													//ǰ��վѪ��
 uint8_t  Enemy_color;												//������
static u16 		  speed_status_count = 0;							// ����ʱ���ʱ
static u16 		  speed_status_count2 = 0;						// ����ʱ���ʱ
static int32_t  chassis_register1 = 5000;//5000;//360;						//�ٶȼ�¼ֵ
static int32_t 	chassis_register2 = 0;							//�ٶȼ�¼ֵ
static u16 			speed_status_flag = 0;							// ��Եת���ֹ���
static u16 			forward_flag ,back_flag = 0;					// ���ģʽ1 ǿ��ת���־λ
#if (SPEED_MODE == 4)
  u16 		  speed_status_count3 = 0;									// ����ģʽ��ʱ���ʱ
  u16 			chassis_flag = 0;													//����ģʽ����־λ
  u16  		speed_time_back_flag = SPEED_TIME_FLAG;			//����ʱ��1
  u16  		speed_time_forward_flag = SPEED_TIME_FLAG;	//����ʱ��1
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
 u16 			last_remain_HP = 600; 											//�ϴ�ʣ��Ѫ��
 u16 			changed_HP;																	// �仯Ѫ��		
 u16      speed_status_count4 = 0;									// ģʽ5����ֵ
#endif	

static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{
		 
		
//	static VisionStatus_E  Enemy_status = Enemy_Disappear;			//���˳���״̬
    Enemy_status = get_Enemy_status();

    if		 (automatic_Enemy_color() == Enemy_color_blue)		// ����Ϊ����
        outpost_HP = 	referee_red_outpost_HP();							// ��ȡ�췽ǰ��վѪ��

    else if(automatic_Enemy_color() == Enemy_color_red)			// �з�Ϊ�췽
        outpost_HP =  referee_blue_outpost_HP();						// ��ȡ����ǰ��վѪ��

    if(outpost_HP >= 10	)																		// ǰ��վѪ���ϸ�
        Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;
		
    else if(outpost_HP < 10)
    {
        if	(Enemy_status == Enemy_Disappear)													//����δ���֣������ٶ�����
        {
					
#if   (SPEED_MODE == 1)										
            Chassis_ch2 = CHASSIS_AUTO_SPPED;
#elif (SPEED_MODE == 2 || SPEED_MODE == 3 || SPEED_MODE == 4)
						
						speed_status_count ++;
					
					if ((Get_Laser_Forward() == HAVE_THING) || (Get_Laser_Back() == HAVE_THING))	// �˶�����Ե �� ��־λ��1 -> һ��ʱ�����˶���������㷨Ӱ��
					{
							speed_status_flag = 1;
						
						
							speed_status_count 	= 0;
							speed_status_count2 = 0; 
						
							#if (SPEED_MODE == 4)																																												// ��ֹƵ��ת���µ������
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif												
					}
					
					  if(speed_status_count > SPEED_TIME_FLAG)
					{	
						speed_status_flag = 0;																																// ��Եת���־λ
						
						if(speed_status_flag == 0 )
						{
							speed_status_count = 0;															  															//����ֵ����
							chassis_register1 = RNG_Get_RandomRange(4900,5200);//(4900,5000);											//����4000~7000�������(4900 5000)
							Chassis_ch2 = chassis_register1;																										//��ֵ�µ�����ٶ�ֵ
						}							
					}
						else
					{
						Chassis_ch2 = chassis_register1;												//�ٶ�ֵΪ��һ���������ֵ
					}
#endif					
#if (SPEED_MODE == 3)
					speed_status_count2 ++;
			
					if(speed_status_count2 > SPEED_TIME_FLAG)									// ��ʱʱ�䵽 �� ���������
					{
						speed_status_count2 =0;
						speed_status_flag = 0;																	
						chassis_register2 = RNG_Get_RandomRange(0,1);						// ����0~2�����
						
						if (chassis_register2 > 0)
						{
							forward_flag ++;
							back_flag = 0;
							if (forward_flag > 3 )
								chassis_register2 = 0;
							else
								chassis_register2 = 1;
						}
																																		// ���Ʊ����ʱ���� 3 ��������
						if (chassis_register2 <= 0)
						{
							back_flag ++;
							forward_flag =0;
							if(back_flag > 3)
								chassis_register2 = 1;
							else
								chassis_register2 = 0;
						}
						
					}
					
					
					if(speed_status_flag == 0)																// ��־λΪ1ʱ
				{
					if (chassis_register2 > 0)
					{	
						Chassis_Auto_f->sign = GOFORWARD;												// ���������1�� ǰ��
					}
					
					if (chassis_register2 <= 0)															
					{	
            Chassis_Auto_f->sign = GOBACK;														// С��1, ����
					}
		   	}	

#endif
#if (SPEED_MODE == 4)

				speed_status_count3 ++;
//				Chassis_ch2 = chassis_register1;													

        if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))			 
				{
					chassis_flag = 1; 																																		// ��ʱ���̺��˿����µ�����						
					speed_time_forward_flag = SPEED_TIME_FLAG;																						// ����������һ��ת̬�ļ�������
					speed_status_count3 = 0;
				}
				if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
				{
					chassis_flag = 2;
					speed_time_back_flag = SPEED_TIME_FLAG;
					speed_status_count3 = 0;
				}
				
			
			if(speed_status_flag == 0)																															// ��Եת��������
			{
				if (chassis_flag == 1)																															
				{
					if(speed_status_count3 > speed_time_back_flag && Chassis_Auto_f->sign == GOBACK)		// ��ʱ������ֵ���� -> ����ʼ��������
					{ 
						Chassis_Auto_f->sign = GOFORWARD;
						speed_status_count3 = 0;
						speed_time_back_flag += SPEED_TIME_FLAG;																					// ����ֵ�ﵽָ��ֵ -> ָ��ֵ����һ������
					}
				}
				else if(chassis_flag == 2)
				{
					if(speed_status_count3 > speed_time_forward_flag && Chassis_Auto_f->sign == GOFORWARD)
					{
						Chassis_Auto_f->sign = GOBACK;
						speed_status_count3 = 0;
						speed_time_forward_flag += SPEED_TIME_FLAG;
					}
					
				}
			}
				
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
				
				changed_HP = last_remain_HP - referee_remain_HP();										// �仯��Ѫ��
				last_remain_HP = referee_remain_HP();																	// �ϴ�Ѫ��
				speed_status_count4++;
				
				if(speed_status_count4 > 0xFFF0)																			
					speed_status_count4 = 0;
				
				if(changed_HP >= 10)
			{
					if(speed_status_count4 > SPEED_TIME_FLAG)
				{
						if(speed_status_flag == 0)																			// ��Եת��������
						{	
							(Chassis_Auto_f->sign == GOBACK)? (Chassis_Auto_f->sign = GOFORWARD) : (Chassis_Auto_f->sign = GOBACK);			// ������
							chassis_register1 = CHASSIS_BLOCKING_SPPED;
							
							speed_status_count = 0;
							speed_status_count2 = 0; 
							#if (SPEED_MODE == 4)																																												// ��ֹƵ��ת���µ������
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif
						}
				}	
			}
					 		
#endif				
				
            if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))
            {
                Chassis_Auto_f->sign = GOBACK;
            }

            if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
            {
                Chassis_Auto_f->sign = GOFORWARD;
            }
        }

				
				
        else if(Enemy_status == Enemy_Appear)												//���˳��֣������ٶȽ���
        {
            Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;

            if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))
            {
                Chassis_Auto_f->sign = GOBACK;
            }

            if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
            {
                Chassis_Auto_f->sign = GOFORWARD;
            }

        }
    }
}

/**
  * @brief          ���̶̹�
* @param[in]      *Chassis_Blocking_f:  �������ṹ��
  * @retval         none
  */
static void Chassis_Fixation(chassis_control_t * Chassis_Fixation_f)
{

}


/**
  * @brief          ������λ
* @param[in]      *Chassis_Blocking_f:  �������ṹ��
  * @retval         none
  */
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f)
{
    if ((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))
    {
        Chassis_Blocking_f->sign = GOBACK;
        Chassis_ch2 = CHASSIS_AUTO_SPPED;
    }

    if ((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
    {
        Chassis_Blocking_f->sign = GOFORWARD;
        Chassis_ch2 = CHASSIS_AUTO_SPPED;
    }

    if ((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == NO_THING))
    {
        {
            Chassis_ch2 = CHASSIS_BLOCKING_SPPED;
        }

//				uint16_t change_dir_count;
//				register1 = RNG_Get_RandomRange(5,10);
//				change_dir_count ++;
//				if (change_dir_count == register1)
//				{
//						if (Chassis_Blocking_f->sign == GOBACK)
//						{
//								Chassis_Blocking_f->sign = GOFORWARD;
//						}
//						if (Chassis_Blocking_f->sign == GOFORWARD)
//						{
//								Chassis_Blocking_f->sign = GOBACK;
//						}
//						change_dir_count = 0;
//				}
        //���ֵó����ת�ĽǶ� Ϊ�����Զ���λ������׼��
        if (flag == 1)
        {
            register1 = RNG_Get_RandomRange(5, 15);
            register2 = register1;

            if (register2 > 10)
            {
                Chassis_Blocking_f->sign = GOFORWARD;

                if (real_round == 0)
                {
                    state ++;
                    Chassis_ch2 = CHASSIS_AUTO_SPPED * 5.0f;					//5.0
                    register3 = RNG_Get_RandomRange(5, 15);
                    register4 = register3;

                    if (state == register4)
                    {
                        Chassis_Auto(Chassis_Blocking_f);
                        delay_ms(5000);
                        flag = 0;
                    }
                }
            }

            if (register2 < 10)
            {
                Chassis_Blocking_f->sign = GOBACK;

                if (real_round == 0)
                {
                    state ++;
                    Chassis_ch2 = CHASSIS_AUTO_SPPED * 5.0f;
                    register3 = RNG_Get_RandomRange(5, 15);
                    register4 = register3;

                    if (state == register4)
                    {
                        Chassis_Auto(Chassis_Blocking_f);
                        delay_ms(5000);
                        flag = 0;
                    }
                }
            }
        }

        if (flag == 0)
        {
            Chassis_Auto(Chassis_Blocking_f);
            delay_ms(5000);
            flag = 1;
        }

    }
}

/**
  * @brief          �ٽ��Ǵ���
  * @param[in]      none
  * @retval         Friction_wheel_mode
  */
int16_t Yaw_Angle_Limit(int16_t Err_data)
{
    if(Err_data < -4096)
    {
        Err_data += 8191;
    }
    else if(Err_data > 4096)
    {
        Err_data -= 8191;
    }

    return Err_data;
}


/**
  * @brief          ����Ħ����ģʽ
  * @param[in]      none
  * @retval         Friction_wheel_mode
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

