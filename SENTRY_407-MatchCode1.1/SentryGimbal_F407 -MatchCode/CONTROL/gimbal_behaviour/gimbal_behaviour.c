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

//云台初始化
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f);
//云台模式选择
static void Gimbal_Mode_Choose(gimbal_control_t *fir_gimbal_mode_choose_f);
//云台手动模式
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f);
//云台自动瞄准
static void Gimbal_Auto(gimbal_control_t *gimbal_auto_f);
//云台自动巡逻
static void Gimbal_Patrol(gimbal_control_t *gimbal_Auto_f);
//比赛模式
static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f);
	
VisionStatus_E  Last_VisionStatus;		//用来记录上一次的视觉状态
VisionStatus_E  VisionStatus = Enemy_Disappear;

WorkStatus_E	GimbalStatus;
WorkStatus_E  Now_GimbalMode;							
WorkStatus_E  Last_GimbalMode;				//用来记录上一次的云台模式

static u16 vision_status_count = 0;

//static
float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f; //云台电机受控量
u8 Friction_wheel_mode = STOP_SHOOT;
u8 Fire_mode = STOP_FIRE;


/**
  * @brief          初始化调用
  * @param[in]      fir_gimbal_behaviour_f
  * @retval         none
  * @attention
  */
void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f)
{		
	
    //!模式选择
    Gimbal_Mode_Choose(fir_gimbal_behaviour_f);
	
    //若为初始化
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_INITIALIZE)  //云台状态为初始化
    {
        Gimbal_Location_Init(fir_gimbal_behaviour_f); //云台初始化
    }
		
    //若为停机模式
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_STOP || fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_STANDBY)
    {
        Remote_reload();       //摇杆量清零
        Gimbal_Stop(fir_gimbal_behaviour_f); //停止
    }
		
    //若为遥控模式remotecontrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_REMOTECONTROL)
    {
				Gimbal_RemoteControl(fir_gimbal_behaviour_f);
    }
				
    //若为自瞄模式automatic
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_AUTOATTACK)
    {
//				Gimbal_Auto(fir_gimbal_behaviour_f);
//			
				Gimbal_AutoControl(fir_gimbal_behaviour_f);
				Friction_wheel_mode = STOP_SHOOT;
				Fire_mode = STOP_FIRE;
    }
		
    //若为巡逻模式patrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_PATROl)
    {
				Gimbal_Patrol(fir_gimbal_behaviour_f);
    }
		
    //若为比赛模式autocontrol
    if (fir_gimbal_behaviour_f->gimbal_behaviour == GIMBAL_AUTOCONTROL)
    {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
    }
}

/*遥控器指南
- 右边三档为主要当位，右下是停机，右中间是遥控，右上面是自动
- 在遥控档位，左摇杆控制云台，右摇杆控制底盘，左摇杆打下为仅动云台不开摩擦轮，左摇杆打中为动云台开摩擦轮射速为低射速，左摇杆打上为动云台开摩擦轮射速为高射速
- 在自动档位，左边档位为左上才是比赛模式，左下为自动，左中为走位，这两档仅用作测试，左上为比赛模式*/

/**
  * @brief          云台模式选择
  * @param[in]      fir_gimbal_mode_choose_f
  * @retval         none
  * @attention
  */
static void Gimbal_Mode_Choose(gimbal_control_t *fir_gimbal_mode_choose_f)
{

    //右开关打上，自动控制
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_UP)
    {
        //刚刚开机，必须先打到下，然后才能控制
        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STOP)
        {
            Remote_reload();       //摇杆量清零
            Gimbal_Stop(fir_gimbal_mode_choose_f);
        }

        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STANDBY) //之前的状态为待机
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_INITIALIZE; //状态设置为初始化
            Remote_reload();      //摇杆量清零
        }

        if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_DOWN)
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour =GIMBAL_STANDBY ; //巡逻模式(暂时为待机)
        }
				if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW1_MID)
				{
						fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_AUTOATTACK;  //自动攻击模式
				}
				if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 ==  RC_SW1_UP)
				{
						fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_AUTOCONTROL;  //比赛模式

            if(fir_gimbal_mode_choose_f->auto_c->auto_pitch_angle == 0.0f && fir_gimbal_mode_choose_f->auto_c->auto_yaw_angle == 0.0f)
            {
                vision_status_count++;
            }
            else
            {
                vision_status_count = 0;
                VisionStatus = Enemy_Appear;		//敌人出现
							
            }

            if(vision_status_count >= ENEMY_DISAPPEAR_TIMES)
            {
                vision_status_count = ENEMY_DISAPPEAR_TIMES;
                VisionStatus = Enemy_Disappear;		//敌人消失
							

            }
				}
    }

    //右开关打中，遥控控制
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_MID)
    {
        //刚刚开机，必须先打到右下，然后才能控制
        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STOP)
        {
            Remote_reload();       //摇杆量清零
            Gimbal_Stop(fir_gimbal_mode_choose_f);
        }

        if (fir_gimbal_mode_choose_f->gimbal_behaviour == GIMBAL_STANDBY) //之前的状态为待机
        {
            fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_INITIALIZE; //状态设置为初始化
            Remote_reload();      //摇杆量清零
        }

        fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_REMOTECONTROL; //状态设置为手动

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

    //右开关打下，停止工作
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW2_DOWN)
    {
        Gimbal_Stop(fir_gimbal_mode_choose_f); //云台无力
        Remote_reload();       //摇杆量清零
        fir_gimbal_mode_choose_f->gimbal_behaviour = GIMBAL_STANDBY; //进入待机模式
        fir_gimbal_mode_choose_f->Gimbal_all_flag = 0;   //初始化标志位清零
    }

    //出现严重错误
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_ERROR)
    {
        fir_gimbal_mode_choose_f->Gimbal_all_flag = 0;  //初始化标志位清零
        Remote_reload();      //摇杆量清零
    }
}



/**
  * @brief          云台初始化
  * @param[in]      fir_gimbal_location_init_f
  * @retval         none
  * @attention
  */
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f)
{
    //想跳过初始化的话直接加上就好了，不用的时候直接注释掉
//	  fir_gimbal_location_init_f->yaw_c.init_flag = 1;
//    fir_gimbal_location_init_f->pitch_c.init_flag = 1;
//    fir_gimbal_location_init_f->sec_gimbal_control.init_flag = 1;
    //若三个都初始化成功
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
  * @brief          云台遥控模式
  * @param[in]      gimbal_remotecontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f)
{

    Gimbal_ch2 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[2]) * RC_YAW_SPEED * 0.2f;  //Y轴位置环量累加   RC_YAW_SPEED
    Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f); //循环限幅
    Gimbal_ch3 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[3]) * RC_PITCH_SPEED * 0.09f;  //P轴位置环量累加  RC_PITCH_SPEED

    //yaw角度限制     -180~180
//    Gimbal_ch2 = float_limit(Gimbal_ch2, YAW_ANGLE_LIMIT, -YAW_ANGLE_LIMIT);
    //pitch角度限制   0 ~ -85  (哨兵上负下正)
    Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);

    if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //拨轮上  开火
    }
    else if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] <= -500)
    {
        Fire_mode = BACK;       //拨轮上  开火
    }
    else
    {
        Fire_mode = STOP_FIRE;  //拨轮中间  停止开火
    }


    Gimbal_Manual_Work(gimbal_remotecontrol_f, Gimbal_ch2, Gimbal_ch3);
}


/**
  * @brief          云台自动瞄准
  * @param[in]      gimbal_auto_f
  * @retval         none
  * @attention
  */
static void Gimbal_Auto(gimbal_control_t *gimbal_auto_f)
{
    //Change_camera(1); //自瞄模式下切换摄像头参数为短焦相机

    /*预处理*/
//    gimbal_auto_f->yaw_c.angle = gimbal_auto_f->yaw_c.yaw_motor_measure->angle - gimbal_auto_f->yaw_c.last_angle; //获取云台自动打击时的陀螺仪Y轴角度
//	Kalman_Filter_Calc(&kalman_Filter, Vision_Auto_Data.auto_yaw_angle+Gimbal_yaw_angle, Vision_Auto_Data.auto_pitch_angle);  //对小电脑传输角度进行卡尔曼滤波
		if( VisionStatus == Enemy_Disappear )
		{
				gimbal_auto_f->pitch_c.Auto_record_location = (gimbal_auto_f->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024);
		}

    /*云台视觉控制量*/    //(y轴，p轴还没用卡尔曼)
    gimbal_auto_f->auto_c->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 4.0f;
    gimbal_auto_f->auto_c->yaw_control_data = gimbal_auto_f->auto_c->auto_yaw_angle * 4.0f;

		gimbal_auto_f->auto_c->pitch_control_data = float_limit(gimbal_auto_f->auto_c->pitch_control_data, 5.0f*PITCH_ANGLE_LIMIT_UP , 25.0f*PITCH_ANGLE_LIMIT_DOWN);
		gimbal_auto_f->auto_c->yaw_control_data   = float_limit(gimbal_auto_f->auto_c->yaw_control_data , YAW_ANGLE_LIMIT , -YAW_ANGLE_LIMIT);
//			/* P轴滤波 */
//		Data_Accelerated_Control(&gimbal_auto_f->auto_c->pitch_control_data, 3.6f); ////斜坡加速度限制函数
//		gimbal_auto_f->auto_c->pitch_control_data = Sliding_Mean_Filter(&gimbal_auto_f->pitch_c.Slidmean_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data, 45); //均值滑窗滤波（有滞后）
//		gimbal_auto_f->auto_c->pitch_control_data = first_order_filter(&gimbal_auto_f->pitch_c.LowFilt_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data);       //一阶低通滤波
		
    Gimbal_Automatic_Work(gimbal_auto_f);
}



/**
  * @brief          云台无力
  * @param[in]      gimbal_stop_f
  * @retval         none
  * @attention
  */
void Gimbal_Stop(gimbal_control_t *gimbal_stop_f)
{
    //控制量
    gimbal_stop_f->pitch_c.output = 0;
    gimbal_stop_f->yaw_c.output = 0;
    gimbal_stop_f->sec_gimbal_control.output = 0;

    //自瞄控制量
    gimbal_stop_f->auto_c->pitch_control_data = 0;
    gimbal_stop_f->auto_c->yaw_control_data = 0;
	
		Friction_wheel_mode = STOP_SHOOT;  

    Gimbal_ch3 = 0.0f;
    Gimbal_ch2 = 0.0f;
}




/**
  * @brief          云台巡逻
  * @param[in]      *gimbal_Patro_f
  * @retval         none
  *///测试yaw通过4.24
static uint32_t yaw_direction = RIGHT ;
static uint32_t pitch_direction = PITCH_UP ;
float Auto_Yaw_Angle_Target = 0.0f;
float Auto_Pitch_Angle_Target = 0.0f;
static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f)
{
#if yaw_angle_limit	
		/*Y轴*//*------------------  这部分注释掉则yaw轴可以360度 -----------------------------*/
	
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
				Auto_Yaw_Angle_Target -=0.2f;   //更改云台自动的速度
		}
		if(yaw_direction == RIGHT)
		{
				Auto_Yaw_Angle_Target +=0.2f;
		}
		
		/*P轴*/
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
  * @brief          比赛模式(自瞄+巡逻)
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
  * @brief          返回摩擦轮模式
  * @param[in]      none
  * @retval         Friction_wheel_mode
  * @attention
  */
Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void)
{
//	LOW_SPEED,   //低射速
//	HIGH_SPEED,  //高射速
//	STOP_SHOOT,  //停止
    if (Friction_wheel_mode == LOW_SPEED)
        return LOW_SPEED;

    if (Friction_wheel_mode == HIGH_SPEED)
        return HIGH_SPEED;

    if (Friction_wheel_mode == STOP_SHOOT)
        return STOP_SHOOT;

    return SHOOT_ERROR;
}

/**
  * @brief          返回发弹模式
  * @param[in]      none
  * @retval         Fire_mode
  * @attention
  */
Fire_WorkStatus_e Return_Fire_Mode(void)
{
//	FIRE,          //发射
//	AUTO_FIRE,     //自动发射
//	STOP_FIRE,     //停止发射
//	BACK,          //退弹
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



