/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      ��ɵ��̿�������
  ******************************************************************************
  */
#include "Task_Chassis.h"
#include "Task_Safecheck.h"
#include "Task_Detect.h"
#include "sensor.h"
#include "iwdg.h"
#include "chassis_behaviour.h"
#include "photoelectric.h"
#include "Task_Fire.h"
#include "rmmotor.h"
#include "RefereeDeal.h"
#include "CRC.h"
#include "Capacitor_control.h"


/*--------------------����-----------------------*/
//���̿������� static
chassis_control_t chassis_control;



#if CHASSIS_TEST_MODE
    float lowfilt_ch1_jscope; //��ͨ�˲�vx��ӡʵ������
    float lowfilt_ch0_jscope; //��ͨ�˲�vy��ӡʵ������
    int 	id1_17mm_cooling_heat;	// ǹ��1����
    int 	id2_17mm_cooling_heat;	// ǹ��2����
#endif

extern SafeTypeDef Safecheck;
/*--------------------����-----------------------*/
static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f);                //���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��
static void chassis_controlwork(chassis_control_t *chassis_control_f);             //��Ҫ���ƺ��� ѭ��
static void chassis_init(chassis_control_t *chassis_move_init_f);                  //�������ݳ�ʼ��
static void chassis_data_update(chassis_control_t *chassis_data_update_f);         //�������ݸ���
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f);  //����ң��ģʽѡ��
static void chassis_pid_calc(chassis_control_t *chassis_pid_f);                    //����pid����
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2); //���ٶ����ƴ���

#if (power_limit == 1)
    static void Chassis_power_limit(chassis_control_t *chassis_power_limit); //���̹������ƺ���
#endif
#if CHASSIS_TEST_MODE
    static void chassis_jscope_print_curve(void); /* jscope��ӡ���� */
#endif



/**
  * @brief          ���̿���������
  * @param[in]      none
  * @retval         none
  */
void CHASSIS_Task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //�������ݳ�ʼ��
    chassis_init(&chassis_control);

    while (1)
    {
        referee_read_data();
        //*���̿���
        chassis_controlwork(&chassis_control);

        //*���͵��̹��ʡ����̻����������ͷ��±�־λ���������ݰ�
        SuperCap_Send(chassis_control.SuperCap_discharge_flag);

        //*can2���̷����ݵ���̨
        Chassis_to_Gimbal(&chassis_control);

        #if chassis_using
        LEDE1 = 0; //������������

        CAN1_Chassis_SetMsg(chassis_control.chassis_motor[1].output,
                            chassis_control.chassis_motor[2].output);					 //*can1���͵���ֵ
        #endif
        vTaskDelay(CHASSIS_CONTROL_TIME);
    }
}



/**
  * @brief          ���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��
  * @param[in]      *chassis_setmsg_f���������ṹ��
  * @retval         none
  */
static int send_sign = 0; //˳����ң��ֵ
static uint8_t Enemy_color ; //������ɫ
static int16_t can_bullet_speed;

static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f)
{
    /*���̷�ң�����ݵ���̨*/
    if ((chassis_setmsg_f->chassis_mode != CHASSIS_STANDBY) || (chassis_setmsg_f->chassis_mode != CHASSIS_STOP)) //����ˢ������
    {
        if (send_sign == 0)
        {
            CAN2_Chassis_RC_SetMsg(chassis_setmsg_f->chassis_RC); //����Ϊң����ģʽ
            send_sign = 1;
        }

        if (send_sign == 1)
        {
//						CAN1_Chassis_Yaw_SetMsg(chassis_setmsg_f->yaw_motor_measure);
            send_sign = 0;
        }
    }



    can_bullet_speed = (int)bullet_speed();				//����
    Enemy_color =	automatic_Enemy_color();				//������ɫ
    CAN2_Enemy_color_SetMsg(Enemy_color,
                            can_bullet_speed,
                            referee_shooter_id1_17mm_cooling_limit(),		//������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
                            referee_shooter_id1_17mm_cooling_heat(),	 		//1 �� 17mm ǹ������
                            referee_hurt_type());

}


/**
  * @brief          �������ݳ�ʼ��
  * @param[in]      *chassis_move_init_f���������ṹ��
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init_f)
{
    uint8_t i;


    /*--------------------��ȡָ��--------------------*/
    {
        //����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
        chassis_move_init_f->chassis_RC = get_remote_control_point();

        //��ȡ����ָ��
        for (i = 0; i < 4; i++)
        {
            chassis_move_init_f->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        }

        //������̨��can2�����������ݽṹ��ָ��
        //��ȡcan1��yaw���ָ��
        chassis_move_init_f->yaw_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();

        chassis_move_init_f->Fire_task_control = get_Fire_control_point();
    }

    /*--------------------���ң������ֵ�Ƿ���ȷ--------------------*/
    RC_data_is_error();

    /*--------------------��ʼ����ͨ�˲�--------------------*/
    {
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
    }

    /*--------------------��ʼ������pid--------------------*/
    {
        //�����ƶ�pid
        pid_init(&chassis_move_init_f->chassis_speed_pid[0], CHASSIS_MOTOR1_PID_Kp, CHASSIS_MOTOR1_PID_Ki, CHASSIS_MOTOR1_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[1], CHASSIS_MOTOR2_PID_Kp, CHASSIS_MOTOR2_PID_Ki, CHASSIS_MOTOR2_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[2], CHASSIS_MOTOR3_PID_Kp, CHASSIS_MOTOR3_PID_Ki, CHASSIS_MOTOR3_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[3], CHASSIS_MOTOR4_PID_Kp, CHASSIS_MOTOR4_PID_Ki, CHASSIS_MOTOR4_PID_Kd, 0, 0);
        //����λ�û�pid
        pid_init(&chassis_move_init_f->chassis_location_pid, CHASSIS_LOCATION_PID_P, CHASSIS_LOCATION_PID_I, CHASSIS_LOCATION_PID_D, 0, 0);
        //������ת����pid
        pid_init(&chassis_move_init_f->chassis_rotate_pid, CHASSIS_ROTATE_FOLLOW_P, CHASSIS_ROTATE_FOLLOW_I, CHASSIS_ROTATE_FOLLOW_D, 0, 0);
    }

    //���̿���״̬Ϊֹͣ (ע������ڻ�ȡָ���Ժ�)
    chassis_move_init_f->chassis_mode = CHASSIS_STOP;
    //chassis_move_init_f->last_chassis_mode = CHASSIS_STOP;

    chassis_move_init_f->sign = GOFORWARD;

    //���ν����������
    chassis_data_update(chassis_move_init_f);
}


/**
  * @brief          ������Ҫ������ƺ���
  * @param[in]      *chassis_control_f���������ṹ��
  * @retval         none
  */
static void chassis_controlwork(chassis_control_t *chassis_control_f)
{


    //���ң������ֵ�Ƿ���ȷ
    RC_data_is_error();

    //�������ݸ���
    chassis_data_update(chassis_control_f);

    //ң����ģʽ״̬����
    chassis_remote_mode_choose(chassis_control_f);

    //���̿���PID����
    chassis_pid_calc(chassis_control_f);

    #if power_limit //��������
    Chassis_power_limit(chassis_control_f);
    #endif
}



/**
  * @brief          �������ݸ���
  * @param[in]      *chassis_data_update_f���������ṹ��
  * @retval         none
  * @attention
  */
static void chassis_data_update(chassis_control_t *chassis_data_update_f)
{
    u8 i;

    //����ٶȸ���
    for (i = 0; i < 4; i++)
    {
        chassis_data_update_f->chassis_motor[i].speed = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->speed;
        chassis_data_update_f->chassis_motor[i].position = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->position;
        chassis_data_update_f->chassis_motor[i].accel = chassis_data_update_f->chassis_speed_pid[i].Derror[0] * 500.0f;
    }
}


/**
  * @brief          ����ң��ģʽѡ��
  * @param[in]      *chassis_mode_choose_f���������ṹ��
  * @retval         none
  */
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    //�ж���ʲôģʽ
    chassis_behaviour_mode_set(chassis_mode_choose_f);
}



/**
  * @brief          ���̿���������
  * @param[in]      *chassis_set_f���������ṹ��
  *                 ch0��������ch0����ֵ
  *                 ch1��������ch1����ֵ
  *                 ch2��������ch2����ֵ
  * @retval         none
  */
void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2)
{
    /*������б�º�������*/
    Chassis_accelerated_Control(&ch0, &ch1, &ch2);

    //һ�׵�ͨ�˲�����
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

    /*����ң��ģʽ*/
    if (chassis_set_f->chassis_mode == CHASSIS_REMOTECONTROL)
    {
        chassis_set_f->speed_z_set = 5.0f * ch0;
        chassis_set_f->chassis_motor[1].speed_set = chassis_set_f->speed_z_set;
    }

    if (chassis_set_f->chassis_mode == CHASSIS_STANDBY)
    {
        chassis_set_f->speed_z_set = 0.0f * ch0;
        chassis_set_f->chassis_motor[1].speed_set = chassis_set_f->speed_z_set;

    }

    /*�����Զ�ģʽ*/
    if  (chassis_set_f->chassis_mode == CHASSIS_AUTO || chassis_set_f->chassis_mode == CHASSIS_BLOCKING)
    {
        chassis_set_f->speed_z_set = ch2;

        if (chassis_set_f->sign == GOFORWARD)
        {
            chassis_set_f->chassis_motor[1].speed_set = chassis_set_f->speed_z_set;
//						if (chassis_set_f->chassis_motor[1].speed > CHASSIS_AUTO_MAX_SPEED)
//						{
//								chassis_set_f->chassis_motor[1].speed = CHASSIS_AUTO_MAX_SPEED;
//						}
        }

        if (chassis_set_f->sign == GOBACK)
        {
            chassis_set_f->chassis_motor[1].speed_set = -chassis_set_f->speed_z_set;
//						if (chassis_set_f->chassis_motor[1].speed < -CHASSIS_AUTO_MAX_SPEED)
//						{
//								chassis_set_f->chassis_motor[1].speed = -CHASSIS_AUTO_MAX_SPEED;
//						}
        }

    }

    #if CHASSIS_TEST_MODE
    chassis_jscope_print_curve(); // jscope��ӡ����
    #endif
}



/**
  * @brief          ���̿���PID����
  * @param[in]      *chassis_pid_f���������ṹ��
  * @retval         none
  */
static void chassis_pid_calc(chassis_control_t *chassis_pid_f)
{
    //��̨ģʽ������λ�û�����
    if (chassis_pid_f->chassis_mode == CHASSIS_BATTERY)
    {
        /*���PIDλ�ñջ�����*/
        chassis_pid_f->chassis_motor[0].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[0]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[0].speed, 0, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
        chassis_pid_f->chassis_motor[1].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[1]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[1].speed, 0, M3508_MAX_OUTPUT_CURRENT); //���16000���� MAX_MOTOR_CAN_OUTPUT
        chassis_pid_f->chassis_motor[2].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[2]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[2].speed, 0, M3508_MAX_OUTPUT_CURRENT);
        chassis_pid_f->chassis_motor[3].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[3]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[3].speed, 0, M3508_MAX_OUTPUT_CURRENT);
    }
    else
    {
        /*���PID�ٶȱջ�����*/
        chassis_pid_f->chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[0]), chassis_pid_f->chassis_motor[0].speed_set, chassis_pid_f->chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
        chassis_pid_f->chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[1]), chassis_pid_f->chassis_motor[1].speed_set, chassis_pid_f->chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT); //���16000���� MAX_MOTOR_CAN_OUTPUT
        chassis_pid_f->chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[2]), chassis_pid_f->chassis_motor[2].speed_set, chassis_pid_f->chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
        chassis_pid_f->chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[3]), chassis_pid_f->chassis_motor[3].speed_set, chassis_pid_f->chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);
    }
}



/**
  * @brief          ���̼��ٶ�����б�º���
  * @param[in]      *ch0��
  *                 *ch1��
  *                 *ch2��
  * @retval         none
  */
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2)
{
    static int16_t last_ch[3] = {0, 0, 0};
    int16_t temp[3];

    temp[0] = *ch0 - last_ch[0];
    temp[1] = *ch1 - last_ch[1];
    temp[2] = *ch2 - last_ch[2];

    if (chassis_control.chassis_RC->rc.s2 == RC_SW2_UP) //�Զ�ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (chassis_control.chassis_mode == CHASSIS_TWIST_WAIST || chassis_control.chassis_mode == CHASSIS_ROTATION) //Ť��ģʽ�²�����ת���ٶ�����
        {
            if (float_abs(temp[2]) > ROTATING_ACCELERAD)
                *ch2 = last_ch[2] + temp[2] / float_abs(temp[2]) * ROTATING_ACCELERAD;
        }
    }

    if (chassis_control.chassis_RC->rc.s2 == RC_SW2_MID) //ң��ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (chassis_control.chassis_mode == CHASSIS_TWIST_WAIST) //Ť��ģʽ�²�����ת���ٶ�����
        {
            if (float_abs(temp[2]) > ROTATING_ACCELERAD)
                *ch2 = last_ch[2] + temp[2] / float_abs(temp[2]) * ROTATING_ACCELERAD;
        }
    }

    last_ch[0] = *ch0;
    last_ch[1] = *ch1;
    last_ch[2] = *ch2;
}



/*
*���ܣ����̷�ֹ�˶��켣ʧ��
*���룺�����ĸ��������ָ��
*�������������ĸ����
*/
/**
  * @brief          ���̷�ֹ�˶��켣ʧ��
  * @param[in]      *v0��
  *                 *v1��
  *                 *v2��
  *                 *v3��
  *                 SPEED_GAIN��
  * @retval         none
  */
static void Keep_Driv_Track(int16_t *v0, int16_t *v1, int16_t *v2, int16_t *v3, int16_t SPEED_GAIN)
{
    static int16_t max_v = 0;
    static float scale = 1.0f;

    max_v = max_abs(*v0, max_abs(*v1, max_abs(*v2, *v3))); // ȡ�ٶ���ֵ��������

    if (SPEED_GAIN == 0)
    {
        *v0 = *v1 = *v2 = *v3 = 0;
    }
    else if (max_v > (SPEED_GAIN * 660))
    {
        scale = max_v / (SPEED_GAIN * 660);
        *v0 = (int16_t)((*v0) / scale);
        *v1 = (int16_t)((*v1) / scale);
        *v2 = (int16_t)((*v2) / scale);
        *v3 = (int16_t)((*v3) / scale);
    }
}



/* �������� */
#if (power_limit == 1)
/*****************************************************************���̹�������**************************************************************************************/
////����������ز�����ʼ��
///**
//  * @brief          ����������ز�����ʼ��
//  * @param[in]      *chassis_power_init_f���������ṹ��
//  * @retval         none
//  */
//static void Power_Init(chassis_control_t *chassis_power_init)
//{
//    chassis_power_init->Chassis_PowerLimit.Real_Power[2] = REFEREE.PowerHeat.chassis_power;         //ʵʱ����
//    chassis_power_init->Chassis_PowerLimit.RemainPower[2] = REFEREE.PowerHeat.chassis_power_buffer; //���ʻ���

//    chassis_power_init->Chassis_PowerLimit.Real_Power[1] = chassis_power_init->Chassis_PowerLimit.Real_Power[2];
//    chassis_power_init->Chassis_PowerLimit.Real_Power[0] = chassis_power_init->Chassis_PowerLimit.Real_Power[1];

//    chassis_power_init->Chassis_PowerLimit.RemainPower[1] = chassis_power_init->Chassis_PowerLimit.RemainPower[2];
//    chassis_power_init->Chassis_PowerLimit.RemainPower[0] = chassis_power_init->Chassis_PowerLimit.RemainPower[1];

//    REFEREE.PowerHeat.error = 0;
//    chassis_power_init->Chassis_PowerLimit.RemainPower[2] = chassis_power_init->Chassis_PowerLimit.RemainPower[1] = chassis_power_init->Chassis_PowerLimit.RemainPower[0] = 60; //���幦��
//}

/**
  * @brief          ���̹�������
  * @param[in]      *chassis_power_limit_f���������ṹ��
  * @retval         none
  */
static void Chassis_power_limit(chassis_control_t *chassis_power_limit)
{
    chassis_power_limit->Chassis_PowerLimit.RemainPower[2] = referee_chassis_power_buffer(); //���ʻ���

    {
        if (chassis_power_limit->Chassis_PowerLimit.RemainPower[2] < PowerLimit_Thres && chassis_power_limit->Chassis_PowerLimit.RemainPower[2] > 5) //����0��ֹ����ϵͳ���ݴ����
        {
            chassis_power_limit->Chassis_PowerLimit.SumOutValue =  abs(chassis_power_limit->chassis_motor[1].output) ;						//������������Ժ�
            chassis_power_limit->Chassis_PowerLimit.LimitOutValue = chassis_power_limit->Chassis_PowerLimit.RemainPower[2] * chassis_power_limit->Chassis_PowerLimit.RemainPower[2] * PowerLimit_Param; //���幦��ƽ���ͳ˹�������ϵ��
            //�����ֵ���б�������
            chassis_power_limit->chassis_motor[1].output = chassis_power_limit->Chassis_PowerLimit.LimitOutValue * chassis_power_limit->chassis_motor[1].output / chassis_power_limit->Chassis_PowerLimit.SumOutValue;

        }
        else if (chassis_power_limit->Chassis_PowerLimit.RemainPower[2] <= 5) // && chassis_power_limit->Chassis_PowerLimit.RemainPower[2]!=0
        {
            chassis_power_limit->chassis_motor[0].output = 0;
            chassis_power_limit->chassis_motor[1].output = 0;
            chassis_power_limit->chassis_motor[2].output = 0;
            chassis_power_limit->chassis_motor[3].output = 0;
        }
    }

    /*��ֹ�����˶��켣ʧ�洦��*/
    Keep_Driv_Track(&chassis_power_limit->chassis_motor[0].output, &chassis_power_limit->chassis_motor[1].output, &chassis_power_limit->chassis_motor[2].output, &chassis_power_limit->chassis_motor[3].output, 14);
}
#endif



/**
  * @brief          ���ص���״̬
  * @param[in]      none
  * @retval
  */
u8 Return_Chassis_Mode(void)
{
    if (chassis_control.chassis_mode != CHASSIS_STOP && chassis_control.chassis_mode != CHASSIS_INITIALIZE && chassis_control.chassis_mode != CHASSIS_STANDBY)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          ����ȫ���ر�
  * @param[in]      options:  0:������̿�����  1:��ֹ����   2:��ֹ���̼�yaw��Ͳ�����
  * @retval         none
  * @attention
  */
void Chassis_Task_OFF(u8 options)
{
    //������̿�����
    chassis_control.speed_x_set = 0;
    chassis_control.speed_y_set = 0;
    chassis_control.speed_z_set = 0;
    chassis_control.chassis_motor[0].speed_set = 0;
    chassis_control.chassis_motor[1].speed_set = 0;
    chassis_control.chassis_motor[2].speed_set = 0;
    chassis_control.chassis_motor[3].speed_set = 0;
    chassis_control.chassis_motor[0].output = 0;
    chassis_control.chassis_motor[1].output = 0;
    chassis_control.chassis_motor[2].output = 0;
    chassis_control.chassis_motor[3].output = 0;

    if (options)
    {
        CAN1_Chassis_SetMsg(0, 0);

        if (options == 2)
        {
//            CAN1_Chassis_Gimbal_Fire(0, 0, 0);
        }
    }
}


static uint8_t robot_id;


// ������id ��ȡ�з���ɫ				7 -> �췽�ڱ��� 0 -> �����ڱ�
uint8_t  automatic_Enemy_color(void)
{

    robot_id = referee_robot_id();

    if(robot_id == 7)
        Enemy_color = Enemy_color_blue;

    else if(robot_id == 107)
        Enemy_color = Enemy_color_red;

}





#if CHASSIS_TEST_MODE
/* jscope��ӡ���� */
static void chassis_jscope_print_curve(void)
{
    lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
    lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
    id1_17mm_cooling_heat = referee_shooter_id1_17mm_cooling_heat();
    id2_17mm_cooling_heat = referee_shooter_id2_17mm_cooling_heat();

}
#endif
