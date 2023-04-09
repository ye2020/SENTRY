/*==================================================*
*文件功能：1.控制整个发弹系统的任务
*         2.供弹系统的控制和发射摩擦轮的控制
*         3.热量限制（按等级）
*
*备注：該文件还没加入裁判系统的热量限制
*
*发射模式：1.高射速模式（即用到了射速三个等级中的最高射速来给摩擦轮）（精确度高。打风车时或者远程时用到）
           2.低射速模式（默认用最低射速限制15m/s速度，保证高射频）（精确度低。2.5m范围内近战用）
           3.换血模式  （关闭热量限制，用血量换取热量，以最高射频进行快速打击，用在最后近距离打基地用，注意射速不要拉高，不然打两发就死）

*==================================================*/

#include "Task_Fire.h"
#include "rmmotor.h"
#include "chassis_behaviour.h"
#include "LED.h"


int fire_heart = 0; //火控任务心跳


/*全局声明*/
Fire_task_t Fire_param;


static void Fire_Control(void);    //发弹系统控制
static void Fire_param_init(void); //参数初始化


//返回火控控制变量，通过指针传递方式传递信息
const Fire_task_t *get_Fire_control_point(void)
{
    return &Fire_param;
}


/**火控任务**/
void FIRE_Task(void *pvParameters)
{
    //加载时间
    vTaskDelay(FIRE_TASK_INIT_TIME);

    Fire_param_init();

    while(1)
    {
        #ifdef fire_work
        fire_heart = !fire_heart; //火控任务心跳
        LEDE3 = fire_heart;

        //允许控制
        if (Return_Chassis_Mode() == 1 && (Return_RC_S1num() == RC_SW1_UP ||  Return_RC_S1num() == RC_SW1_MID))//1、3
        {
            Fire_Control();
            Laser_ON();
        }
        else //不允许控制
        {
            PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_SHOOT_STOP;
            Fire_param.GD_output = 0;
            Fire_param.shoot_speed = 0;
            Laser_OFF();
        }

        #endif
        //任务周期
        vTaskDelay(FIRE_CONTROL_TIME_MS);
    }
}


/**
  * @brief          参数初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Fire_param_init(void)
{
    //获取拨弹电机指针
    Fire_param.fire_motor_measure = get_Fire_Motor_Measure_Point();

    //射弹模式默认为停火
    Fire_param.fire_workstatus = STOP_FIRE;
    //摩擦轮模式停止模式
    Fire_param.Friction_workstatus = STOP_SHOOT;

    pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);

    Fire_param.GD_output = 0;
    Fire_param.shoot_speed = 0;
}


/**
  * @brief          发弹系统控制
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Fire_Control(void)
{
    PWM_Shoot_Left = PWM_Shoot_Right = FRICTION_ONE_SHOOT_SPEED;

    //获取摩擦轮模式（射速
    Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();
    //获取火控模式（是否开火
    Fire_param.fire_workstatus = Return_Fire_Mode();

    /* 摩擦轮控制 */
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

    /* 供弹电机控制 GD=供弹 */
    if(Fire_param.fire_workstatus == STOP_FIRE) //停止发射
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else if(Fire_param.fire_workstatus == FIRE) //发射
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, -LOADING_SPEED, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else if(Fire_param.fire_workstatus == BACK) //退弹
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, LOADING_SPEED, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else
    {
        Fire_param.GD_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
}


/**
  * @brief          火控系统关闭
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

