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
#include "gimbal_behaviour.h"
#include "LED.h"


int fire_heart = 0; //火控任务心跳


/*全局声明*/
Fire_task_t Fire_param;

/*  拨弹电机堵转计数 */
static  u16 loading_A_stop_count = 0;
static  u16 loading_B_stop_count = 0;
/*  拨弹电机堵转次数 */
 u16 loading_A_time = 0;
 u16 loading_B_time = 0;
/* 拨弹电机电流输出值 */
static uint16_t  loading_A_speed = LOADING_SPEED;
static uint16_t  loading_B_speed = -LOADING_SPEED;

 

#define shoot_laser_on()    Laser_ON()      //激光开启宏定义
#define shoot_laser_off()   Laser_OFF()     //激光关闭宏定义



static void Fire_Control(void);    //发弹系统控制
static void Fire_param_init(void); //参数初始化
static void shoot_hot_limit(void); //热量限制


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
        LEDE3 = fire_heart;		 //火控任务心跳

        //允许控制																	
        if (Return_Gimbal_Mode() == 1 && (Return_RC_S2num() == RC_SW2_UP ||  Return_RC_S2num() == RC_SW2_MID))			//5.10修改
        {
            Fire_Control();
            shoot_laser_on();
        }
        else //不允许控制
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
        //任务周期
        vTaskDelay(FIRE_CONTROL_TIME_MS);
    }
}

//int hot_current;
/*  热量限制 */
void shoot_hot_limit(void)
{
		uint16_t Residual_power_percentage;			
	
		Fire_param.hot_max 		 = get_shooter_cooling_limit(); //机器人 1 号 17mm 枪口热量上限
		Fire_param.hot_current = get_shooter_cooling_heat();	//1 号 17mm 枪口热量
//		hot_current = get_shooter_cooling_heat();
	
		Residual_power_percentage = (Fire_param.hot_max - Fire_param.hot_current);
		
		if(Residual_power_percentage > 0 && Residual_power_percentage < 60)
		{
			//做热量限制
			Fire_param.GD_speed_gain = (float)(Residual_power_percentage/ 60.0f)*(Residual_power_percentage / 60.0f);
		}
		
		else 
		{
			//不做热量限制
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

/**参数初始化**/
static void Fire_param_init(void)
{
		//获取拨弹电机指针
    Fire_param.fire_motorA_measure = Get_Fire_MotorA_Measure_Point();
    Fire_param.fire_motorB_measure = Get_Fire_MotorB_Measure_Point();

    //射弹模式默认为停火
    Fire_param.fire_workstatus = STOP_FIRE;
    //摩擦轮模式停止模式
    Fire_param.Friction_workstatus = STOP_SHOOT;

    pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);
		pid_init(&Fire_param.fire_p_pid, FIRE_P_P, FIRE_P_I, FIRE_P_D, 0, 0);

    Fire_param.GDA_output = 0;
	  Fire_param.GDB_output = 0;
    Fire_param.shoot_speed = 0;
	snail_motor_pwm(FRICTION_SHOOT_STOP);

}


/**发弹系统控制**/
static void Fire_Control(void)
{		
		//热量限制
		shoot_hot_limit();
    //获取摩擦轮模式（射速
    Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();				// 5.10 为测单独云台摩擦轮修改
    //获取火控模式（是否开火
    Fire_param.fire_workstatus = Return_Fire_Mode();

    /* 摩擦轮控制 */
    if (Fire_param.Friction_workstatus == LOW_SPEED)
    {
//		snail_motor_pwm(FRICTION_ONE_SHOOT_SPEED);
				PWM_Shoot_Upper_Left  = FRICTION_ONE_SHOOT_SPEED;//4.23测试打弹关掉一排
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

    /* 供弹电机控制 GD=供弹 */
    if(Fire_param.fire_workstatus == STOP_FIRE) //停止发射
    {
        Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
				Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
		
		
    else if(Fire_param.fire_workstatus == FIRE) //发射
    {		
			/*  当拨弹电机转速低于一定值 */
			if(Fire_param.fire_motorA_measure->speed >= (LOADING_SPEED + 700)&& Fire_param.fire_motorA_measure->speed <= (-(LOADING_SPEED + 700)))   
			{	
				/*  拨转电机堵转计数值 */
				loading_A_stop_count++;
				
				/*  堵转计数值高于一定数 ，记录堵转次数 +1 */
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
					

    else if(Fire_param.fire_workstatus == BACK) //退弹
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
