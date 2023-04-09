/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      完成底盘控制任务。
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


/*--------------------变量-----------------------*/
//底盘控制数据 static
chassis_control_t chassis_control;



#if CHASSIS_TEST_MODE
    float lowfilt_ch1_jscope; //低通滤波vx打印实际曲线
    float lowfilt_ch0_jscope; //低通滤波vy打印实际曲线
    int 	id1_17mm_cooling_heat;	// 枪口1热量
    int 	id2_17mm_cooling_heat;	// 枪口2热量
#endif

extern SafeTypeDef Safecheck;
/*--------------------函数-----------------------*/
static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f);                //底盘发送数据到云台（yaw轴电机换算码盘值，yaw轴电机速度值，遥控值）
static void chassis_controlwork(chassis_control_t *chassis_control_f);             //主要控制函数 循环
static void chassis_init(chassis_control_t *chassis_move_init_f);                  //底盘数据初始化
static void chassis_data_update(chassis_control_t *chassis_data_update_f);         //底盘数据更新
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f);  //底盘遥控模式选择
static void chassis_pid_calc(chassis_control_t *chassis_pid_f);                    //底盘pid计算
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2); //加速度限制处理

#if (power_limit == 1)
    static void Chassis_power_limit(chassis_control_t *chassis_power_limit); //底盘功率限制函数
#endif
#if CHASSIS_TEST_MODE
    static void chassis_jscope_print_curve(void); /* jscope打印曲线 */
#endif



/**
  * @brief          底盘控制主函数
  * @param[in]      none
  * @retval         none
  */
void CHASSIS_Task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘数据初始化
    chassis_init(&chassis_control);

    while (1)
    {
        referee_read_data();
        //*底盘控制
        chassis_controlwork(&chassis_control);

        //*发送底盘功率、底盘缓冲能量、和飞坡标志位到超级电容板
        SuperCap_Send(chassis_control.SuperCap_discharge_flag);

        //*can2底盘发数据到云台
        Chassis_to_Gimbal(&chassis_control);

        #if chassis_using
        LEDE1 = 0; //底盘任务心跳

        CAN1_Chassis_SetMsg(chassis_control.chassis_motor[1].output,
                            chassis_control.chassis_motor[2].output);					 //*can1发送电流值
        #endif
        vTaskDelay(CHASSIS_CONTROL_TIME);
    }
}



/**
  * @brief          底盘发送数据到云台（yaw轴电机换算码盘值，yaw轴电机速度值，遥控值）
  * @param[in]      *chassis_setmsg_f：底盘主结构体
  * @retval         none
  */
static int send_sign = 0; //顺序发送遥控值
static uint8_t Enemy_color ; //敌人颜色
static int16_t can_bullet_speed;

static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f)
{
    /*底盘发遥控数据到云台*/
    if ((chassis_setmsg_f->chassis_mode != CHASSIS_STANDBY) || (chassis_setmsg_f->chassis_mode != CHASSIS_STOP)) //待机刷新数据
    {
        if (send_sign == 0)
        {
            CAN2_Chassis_RC_SetMsg(chassis_setmsg_f->chassis_RC); //否则为遥控器模式
            send_sign = 1;
        }

        if (send_sign == 1)
        {
//						CAN1_Chassis_Yaw_SetMsg(chassis_setmsg_f->yaw_motor_measure);
            send_sign = 0;
        }
    }



    can_bullet_speed = (int)bullet_speed();				//射速
    Enemy_color =	automatic_Enemy_color();				//敌人颜色
    CAN2_Enemy_color_SetMsg(Enemy_color,
                            can_bullet_speed,
                            referee_shooter_id1_17mm_cooling_limit(),		//机器人 1 号 17mm 枪口上限速度 单位 m/s
                            referee_shooter_id1_17mm_cooling_heat(),	 		//1 号 17mm 枪口热量
                            referee_hurt_type());

}


/**
  * @brief          底盘数据初始化
  * @param[in]      *chassis_move_init_f：底盘主结构体
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init_f)
{
    uint8_t i;


    /*--------------------获取指针--------------------*/
    {
        //返回遥控器控制变量，通过指针传递方式传递信息
        chassis_move_init_f->chassis_RC = get_remote_control_point();

        //获取底盘指针
        for (i = 0; i < 4; i++)
        {
            chassis_move_init_f->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        }

        //接收云台板can2传过来的数据结构体指针
        //获取can1的yaw轴的指针
        chassis_move_init_f->yaw_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();

        chassis_move_init_f->Fire_task_control = get_Fire_control_point();
    }

    /*--------------------检查遥控器数值是否正确--------------------*/
    RC_data_is_error();

    /*--------------------初始化低通滤波--------------------*/
    {
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
    }

    /*--------------------初始化底盘pid--------------------*/
    {
        //底盘移动pid
        pid_init(&chassis_move_init_f->chassis_speed_pid[0], CHASSIS_MOTOR1_PID_Kp, CHASSIS_MOTOR1_PID_Ki, CHASSIS_MOTOR1_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[1], CHASSIS_MOTOR2_PID_Kp, CHASSIS_MOTOR2_PID_Ki, CHASSIS_MOTOR2_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[2], CHASSIS_MOTOR3_PID_Kp, CHASSIS_MOTOR3_PID_Ki, CHASSIS_MOTOR3_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[3], CHASSIS_MOTOR4_PID_Kp, CHASSIS_MOTOR4_PID_Ki, CHASSIS_MOTOR4_PID_Kd, 0, 0);
        //底盘位置环pid
        pid_init(&chassis_move_init_f->chassis_location_pid, CHASSIS_LOCATION_PID_P, CHASSIS_LOCATION_PID_I, CHASSIS_LOCATION_PID_D, 0, 0);
        //底盘旋转跟随pid
        pid_init(&chassis_move_init_f->chassis_rotate_pid, CHASSIS_ROTATE_FOLLOW_P, CHASSIS_ROTATE_FOLLOW_I, CHASSIS_ROTATE_FOLLOW_D, 0, 0);
    }

    //底盘开机状态为停止 (注意必须在获取指针以后)
    chassis_move_init_f->chassis_mode = CHASSIS_STOP;
    //chassis_move_init_f->last_chassis_mode = CHASSIS_STOP;

    chassis_move_init_f->sign = GOFORWARD;

    //初次进入更新数据
    chassis_data_update(chassis_move_init_f);
}


/**
  * @brief          底盘主要任务控制函数
  * @param[in]      *chassis_control_f：底盘主结构体
  * @retval         none
  */
static void chassis_controlwork(chassis_control_t *chassis_control_f)
{


    //检查遥控器数值是否正确
    RC_data_is_error();

    //底盘数据更新
    chassis_data_update(chassis_control_f);

    //遥控器模式状态设置
    chassis_remote_mode_choose(chassis_control_f);

    //底盘控制PID计算
    chassis_pid_calc(chassis_control_f);

    #if power_limit //功率限制
    Chassis_power_limit(chassis_control_f);
    #endif
}



/**
  * @brief          底盘数据更新
  * @param[in]      *chassis_data_update_f：底盘主结构体
  * @retval         none
  * @attention
  */
static void chassis_data_update(chassis_control_t *chassis_data_update_f)
{
    u8 i;

    //电机速度更新
    for (i = 0; i < 4; i++)
    {
        chassis_data_update_f->chassis_motor[i].speed = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->speed;
        chassis_data_update_f->chassis_motor[i].position = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->position;
        chassis_data_update_f->chassis_motor[i].accel = chassis_data_update_f->chassis_speed_pid[i].Derror[0] * 500.0f;
    }
}


/**
  * @brief          底盘遥控模式选择
  * @param[in]      *chassis_mode_choose_f：底盘主结构体
  * @retval         none
  */
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    //判断是什么模式
    chassis_behaviour_mode_set(chassis_mode_choose_f);
}



/**
  * @brief          底盘控制量设置
  * @param[in]      *chassis_set_f：底盘主结构体
  *                 ch0：处理后的ch0的数值
  *                 ch1：处理后的ch1的数值
  *                 ch2：处理后的ch2的数值
  * @retval         none
  */
void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2)
{
    /*输入量斜坡函数处理*/
    Chassis_accelerated_Control(&ch0, &ch1, &ch2);

    //一阶低通滤波计算
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

    /*底盘遥控模式*/
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

    /*底盘自动模式*/
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
    chassis_jscope_print_curve(); // jscope打印曲线
    #endif
}



/**
  * @brief          底盘控制PID计算
  * @param[in]      *chassis_pid_f：底盘主结构体
  * @retval         none
  */
static void chassis_pid_calc(chassis_control_t *chassis_pid_f)
{
    //炮台模式（底盘位置环锁死
    if (chassis_pid_f->chassis_mode == CHASSIS_BATTERY)
    {
        /*电机PID位置闭环处理*/
        chassis_pid_f->chassis_motor[0].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[0]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[0].speed, 0, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
        chassis_pid_f->chassis_motor[1].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[1]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[1].speed, 0, M3508_MAX_OUTPUT_CURRENT); //最大16000左右 MAX_MOTOR_CAN_OUTPUT
        chassis_pid_f->chassis_motor[2].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[2]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[2].speed, 0, M3508_MAX_OUTPUT_CURRENT);
        chassis_pid_f->chassis_motor[3].output = Motor_Position_Speed_Control(&(chassis_pid_f->chassis_speed_pid[3]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[3].speed, 0, M3508_MAX_OUTPUT_CURRENT);
    }
    else
    {
        /*电机PID速度闭环处理*/
        chassis_pid_f->chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[0]), chassis_pid_f->chassis_motor[0].speed_set, chassis_pid_f->chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
        chassis_pid_f->chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[1]), chassis_pid_f->chassis_motor[1].speed_set, chassis_pid_f->chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT); //最大16000左右 MAX_MOTOR_CAN_OUTPUT
        chassis_pid_f->chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[2]), chassis_pid_f->chassis_motor[2].speed_set, chassis_pid_f->chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
        chassis_pid_f->chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[3]), chassis_pid_f->chassis_motor[3].speed_set, chassis_pid_f->chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);
    }
}



/**
  * @brief          底盘加速度限制斜坡函数
  * @param[in]      *ch0：
  *                 *ch1：
  *                 *ch2：
  * @retval         none
  */
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2)
{
    static int16_t last_ch[3] = {0, 0, 0};
    int16_t temp[3];

    temp[0] = *ch0 - last_ch[0];
    temp[1] = *ch1 - last_ch[1];
    temp[2] = *ch2 - last_ch[2];

    if (chassis_control.chassis_RC->rc.s2 == RC_SW2_UP) //自动模式
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (chassis_control.chassis_mode == CHASSIS_TWIST_WAIST || chassis_control.chassis_mode == CHASSIS_ROTATION) //扭腰模式下才用旋转加速度限制
        {
            if (float_abs(temp[2]) > ROTATING_ACCELERAD)
                *ch2 = last_ch[2] + temp[2] / float_abs(temp[2]) * ROTATING_ACCELERAD;
        }
    }

    if (chassis_control.chassis_RC->rc.s2 == RC_SW2_MID) //遥控模式
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (chassis_control.chassis_mode == CHASSIS_TWIST_WAIST) //扭腰模式下才用旋转加速度限制
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
*功能：底盘防止运动轨迹失真
*传入：底盘四个轮子输出指针
*传出：处理后的四个输出
*/
/**
  * @brief          底盘防止运动轨迹失真
  * @param[in]      *v0：
  *                 *v1：
  *                 *v2：
  *                 *v3：
  *                 SPEED_GAIN：
  * @retval         none
  */
static void Keep_Driv_Track(int16_t *v0, int16_t *v1, int16_t *v2, int16_t *v3, int16_t SPEED_GAIN)
{
    static int16_t max_v = 0;
    static float scale = 1.0f;

    max_v = max_abs(*v0, max_abs(*v1, max_abs(*v2, *v3))); // 取速度数值最大的轮子

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



/* 功率限制 */
#if (power_limit == 1)
/*****************************************************************底盘功率限制**************************************************************************************/
////功率限制相关参数初始化
///**
//  * @brief          功率限制相关参数初始化
//  * @param[in]      *chassis_power_init_f：底盘主结构体
//  * @retval         none
//  */
//static void Power_Init(chassis_control_t *chassis_power_init)
//{
//    chassis_power_init->Chassis_PowerLimit.Real_Power[2] = REFEREE.PowerHeat.chassis_power;         //实时功率
//    chassis_power_init->Chassis_PowerLimit.RemainPower[2] = REFEREE.PowerHeat.chassis_power_buffer; //功率缓冲

//    chassis_power_init->Chassis_PowerLimit.Real_Power[1] = chassis_power_init->Chassis_PowerLimit.Real_Power[2];
//    chassis_power_init->Chassis_PowerLimit.Real_Power[0] = chassis_power_init->Chassis_PowerLimit.Real_Power[1];

//    chassis_power_init->Chassis_PowerLimit.RemainPower[1] = chassis_power_init->Chassis_PowerLimit.RemainPower[2];
//    chassis_power_init->Chassis_PowerLimit.RemainPower[0] = chassis_power_init->Chassis_PowerLimit.RemainPower[1];

//    REFEREE.PowerHeat.error = 0;
//    chassis_power_init->Chassis_PowerLimit.RemainPower[2] = chassis_power_init->Chassis_PowerLimit.RemainPower[1] = chassis_power_init->Chassis_PowerLimit.RemainPower[0] = 60; //缓冲功率
//}

/**
  * @brief          底盘功率限制
  * @param[in]      *chassis_power_limit_f：底盘主结构体
  * @retval         none
  */
static void Chassis_power_limit(chassis_control_t *chassis_power_limit)
{
    chassis_power_limit->Chassis_PowerLimit.RemainPower[2] = referee_chassis_power_buffer(); //功率缓冲

    {
        if (chassis_power_limit->Chassis_PowerLimit.RemainPower[2] < PowerLimit_Thres && chassis_power_limit->Chassis_PowerLimit.RemainPower[2] > 5) //不等0防止裁判系统数据传输空
        {
            chassis_power_limit->Chassis_PowerLimit.SumOutValue =  abs(chassis_power_limit->chassis_motor[1].output) ;						//输入数据求绝对和
            chassis_power_limit->Chassis_PowerLimit.LimitOutValue = chassis_power_limit->Chassis_PowerLimit.RemainPower[2] * chassis_power_limit->Chassis_PowerLimit.RemainPower[2] * PowerLimit_Param; //缓冲功率平方和乘功率限制系数
            //将输出值进行比例分配
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

    /*防止底盘运动轨迹失真处理*/
    Keep_Driv_Track(&chassis_power_limit->chassis_motor[0].output, &chassis_power_limit->chassis_motor[1].output, &chassis_power_limit->chassis_motor[2].output, &chassis_power_limit->chassis_motor[3].output, 14);
}
#endif



/**
  * @brief          返回底盘状态
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
  * @brief          底盘全部关闭
  * @param[in]      options:  0:清除底盘控制量  1:禁止底盘   2:禁止底盘加yaw轴和拨弹轮
  * @retval         none
  * @attention
  */
void Chassis_Task_OFF(u8 options)
{
    //清除底盘控制量
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


// 处理己方id 获取敌方颜色				7 -> 红方哨兵； 0 -> 蓝方哨兵
uint8_t  automatic_Enemy_color(void)
{

    robot_id = referee_robot_id();

    if(robot_id == 7)
        Enemy_color = Enemy_color_blue;

    else if(robot_id == 107)
        Enemy_color = Enemy_color_red;

}





#if CHASSIS_TEST_MODE
/* jscope打印曲线 */
static void chassis_jscope_print_curve(void)
{
    lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
    lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
    id1_17mm_cooling_heat = referee_shooter_id1_17mm_cooling_heat();
    id2_17mm_cooling_heat = referee_shooter_id2_17mm_cooling_heat();

}
#endif
