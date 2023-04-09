/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      底盘状态机。
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

static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f); 		 //底盘模式选择
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f); //底盘遥控
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f);							 		 //底盘自动模式
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f);					 //底盘走位模式
static int16_t Yaw_Angle_Limit(int16_t M3508_data);														 //底盘电机码盘处理

static int32_t real_round;
/*-------记数器--------*/
static int32_t register1;//都是用作为了自动走位下的记数
static int32_t register2;
static int32_t register3;
static int32_t register4;
/*-------记数器--------*/
static int32_t state;
static int32_t flag = 0;
float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //底盘电机受控量
int8_t Vision_flag = 0;             //视觉模式切换标志（0：关闭，1：自瞄，2：能量机关）



u8 Friction_wheel_mode = STOP_SHOOT;
u8 Fire_mode = STOP_FIRE;
int chassis_cos_calculate_jscope = 0;
int chassis_sin_calculate_jscope = 0;


/**
  * @brief          底盘模式设置
  * @param[in]      *chassis_pid_f：底盘主结构体
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f)
{
    //!底盘模式选择
    chassis_mode_choose(chassis_behaviour_f);

    //*自动模式
    if (chassis_behaviour_f->chassis_mode == CHASSIS_AUTO)
    {
        Chassis_Auto(chassis_behaviour_f);
    }

    //*遥控模式
    if (chassis_behaviour_f->chassis_mode == CHASSIS_REMOTECONTROL)
    {
        Chassis_RemoteControl(chassis_behaviour_f);
    }

    //*走位模式
    if (chassis_behaviour_f->chassis_mode == CHASSIS_BLOCKING)
    {
        Chassis_Blocking(chassis_behaviour_f);
    }

    //*停机模式
    if (chassis_behaviour_f->chassis_mode == CHASSIS_STOP)
    {
        Chassis_Stop(chassis_behaviour_f);
    }

    //*传入值处理
    chassis_set_remote(chassis_behaviour_f, Chassis_ch0, Chassis_ch1, Chassis_ch2);
}

/*遥控器指南
- 右边三档为主要当位，右下是停机，右中间是遥控，右上面是自动
- 在遥控档位，左摇杆控制云台，右摇杆控制底盘，左摇杆打下为仅动云台不开摩擦轮，左摇杆打中为动云台开摩擦轮射速为低射速，左摇杆打上为动云台开摩擦轮射速为高射速
- 在自动档位，左边档位为左上才是比赛模式，左下为自动，左中为走位，这两档仅用作测试，左上为比赛模式*/

/**
  * @brief          底盘模式选择
  * @param[in]      *chassis_mode_choose_f：底盘主结构体
  * @retval         none
  */
static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f)
{

    /* 右开关打上，进入比赛模式 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_UP)
    {
        /* 判断之前的状态是什么 */
        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STOP) //之前的状态为待机
        {
            Remote_reload(); //摇杆量清零
            Chassis_Stop(chassis_mode_choose_f);
        }

        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STANDBY) //之前的状态为待机
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_INITIALIZE; //状态设置为初始化
            Remote_reload();                                          //摇杆量清零
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_UP)//注意 这里是左边拨杆打上
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_AUTO;					//底盘运动
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_DOWN)  //注意 这里是左边拨杆打下
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_REMOTECONTROL;
        }

        if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW1_MID) //注意 这里是左边拨杆打中
        {
            chassis_mode_choose_f->chassis_mode = CHASSIS_AUTO;//CHASSIS_BLOCKING;		6.2调试
        }
    }

    /* 右开关打中，遥控控制 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_MID)
    {
        /* 判断之前的状态是什么 */
        if (chassis_mode_choose_f->chassis_mode == CHASSIS_STOP) //之前的状态为待机
        {
            Remote_reload(); //摇杆量清零
            Chassis_Stop(chassis_mode_choose_f);
        }

        chassis_mode_choose_f->chassis_mode = CHASSIS_REMOTECONTROL;
    }

    /* 右开关打下，停止工作 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW2_DOWN)
    {
        chassis_mode_choose_f->chassis_mode = CHASSIS_STANDBY; //进入待机状态
        Remote_reload();                                        //摇杆量清零
    }

    /* 出现严重错误 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_ERROR)
    {
        chassis_mode_choose_f->chassis_mode = CHASSIS_STANDBY; //进入待机模式
        //Remote_reload();                                        //摇杆量清零
    }
}


/**
  * @brief          底盘遥控
  * @param[in]      *Chassis_Independent_f：底盘主结构体
  * @retval         none
  */
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{

    Chassis_ch1 = Chassis_RemoteControl_f->chassis_RC->rc.ch[1];
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[1];

    if (Chassis_RemoteControl_f->chassis_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //拨轮上  开火
        Friction_wheel_mode = LOW_SPEED;
    }
    else if (Chassis_RemoteControl_f->chassis_RC->rc.ch[4] <= -500)
    {
        Fire_mode = FIRE;       //拨轮上  开火
        Friction_wheel_mode = LOW_SPEED; //摩擦轮模式低速模式
    }
    else
    {
        Fire_mode = STOP_FIRE;  //拨轮中间  停止开火
        Friction_wheel_mode = LOW_SPEED; //摩擦轮模式低速模式
    }
}


/**
  * @brief          底盘无力
  * @param[in]      *Chassis_Stop_f：底盘主结构体
  * @retval         none
  */
void Chassis_Stop(chassis_control_t *Chassis_Stop_f)
{
    //*控制量
    Chassis_Stop_f->chassis_motor[0].output = 0;
    Chassis_Stop_f->chassis_motor[1].output = 0;
    Chassis_Stop_f->chassis_motor[2].output = 0;
    Chassis_Stop_f->chassis_motor[3].output = 0;
    Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0;
    Fire_mode = STOP_FIRE;
    Friction_wheel_mode = STOP_SHOOT;
}

/**
  * @brief          底盘自动
  * @param[in]      *Chassis_Auto_f：底盘主结构体
  * @retval         none
  */

VisionStatus_E  Enemy_status = Enemy_Disappear;			//敌人出现状态

static uint16_t outpost_HP;													//前哨站血量
 uint8_t  Enemy_color;												//调试用
static u16 		  speed_status_count = 0;							// 变速时间计时
static u16 		  speed_status_count2 = 0;						// 变向时间计时
static int32_t  chassis_register1 = 5000;//5000;//360;						//速度记录值
static int32_t 	chassis_register2 = 0;							//速度记录值
static u16 			speed_status_flag = 0;							// 边缘转向紧止随机
static u16 			forward_flag ,back_flag = 0;					// 随机模式1 强制转向标志位
#if (SPEED_MODE == 4)
  u16 		  speed_status_count3 = 0;									// 变向模式二时间计时
  u16 			chassis_flag = 0;													//变向模式二标志位
  u16  		speed_time_back_flag = SPEED_TIME_FLAG;			//计数时间1
  u16  		speed_time_forward_flag = SPEED_TIME_FLAG;	//计数时间1
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
 u16 			last_remain_HP = 600; 											//上次剩余血量
 u16 			changed_HP;																	// 变化血量		
 u16      speed_status_count4 = 0;									// 模式5计数值
#endif	

static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{
		 
		
//	static VisionStatus_E  Enemy_status = Enemy_Disappear;			//敌人出现状态
    Enemy_status = get_Enemy_status();

    if		 (automatic_Enemy_color() == Enemy_color_blue)		// 敌人为蓝方
        outpost_HP = 	referee_red_outpost_HP();							// 获取红方前哨站血量

    else if(automatic_Enemy_color() == Enemy_color_red)			// 敌方为红方
        outpost_HP =  referee_blue_outpost_HP();						// 获取蓝方前哨站血量

    if(outpost_HP >= 10	)																		// 前哨站血量较高
        Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;
		
    else if(outpost_HP < 10)
    {
        if	(Enemy_status == Enemy_Disappear)													//敌人未出现，底盘速度正常
        {
					
#if   (SPEED_MODE == 1)										
            Chassis_ch2 = CHASSIS_AUTO_SPPED;
#elif (SPEED_MODE == 2 || SPEED_MODE == 3 || SPEED_MODE == 4)
						
						speed_status_count ++;
					
					if ((Get_Laser_Forward() == HAVE_THING) || (Get_Laser_Back() == HAVE_THING))	// 运动到边缘 ： 标志位置1 -> 一段时间内运动不收随机算法影响
					{
							speed_status_flag = 1;
						
						
							speed_status_count 	= 0;
							speed_status_count2 = 0; 
						
							#if (SPEED_MODE == 4)																																												// 防止频繁转向导致电机出错
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif												
					}
					
					  if(speed_status_count > SPEED_TIME_FLAG)
					{	
						speed_status_flag = 0;																																// 边缘转向标志位
						
						if(speed_status_flag == 0 )
						{
							speed_status_count = 0;															  															//计数值清零
							chassis_register1 = RNG_Get_RandomRange(4900,5200);//(4900,5000);											//产生4000~7000的随机数(4900 5000)
							Chassis_ch2 = chassis_register1;																										//赋值新的随机速度值
						}							
					}
						else
					{
						Chassis_ch2 = chassis_register1;												//速度值为上一次随机数的值
					}
#endif					
#if (SPEED_MODE == 3)
					speed_status_count2 ++;
			
					if(speed_status_count2 > SPEED_TIME_FLAG)									// 定时时间到 ： 产生随机数
					{
						speed_status_count2 =0;
						speed_status_flag = 0;																	
						chassis_register2 = RNG_Get_RandomRange(0,1);						// 产生0~2随机数
						
						if (chassis_register2 > 0)
						{
							forward_flag ++;
							back_flag = 0;
							if (forward_flag > 3 )
								chassis_register2 = 0;
							else
								chassis_register2 = 1;
						}
																																		// 控制变向的时间在 3 个周期内
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
					
					
					if(speed_status_flag == 0)																// 标志位为1时
				{
					if (chassis_register2 > 0)
					{	
						Chassis_Auto_f->sign = GOFORWARD;												// 随机数大于1， 前进
					}
					
					if (chassis_register2 <= 0)															
					{	
            Chassis_Auto_f->sign = GOBACK;														// 小于1, 后退
					}
		   	}	

#endif
#if (SPEED_MODE == 4)

				speed_status_count3 ++;
//				Chassis_ch2 = chassis_register1;													

        if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))			 
				{
					chassis_flag = 1; 																																		// 此时底盘后退开启新的周期						
					speed_time_forward_flag = SPEED_TIME_FLAG;																						// 不断重置另一种转态的计数上限
					speed_status_count3 = 0;
				}
				if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
				{
					chassis_flag = 2;
					speed_time_back_flag = SPEED_TIME_FLAG;
					speed_status_count3 = 0;
				}
				
			
			if(speed_status_flag == 0)																															// 边缘转向禁用随机
			{
				if (chassis_flag == 1)																															
				{
					if(speed_status_count3 > speed_time_back_flag && Chassis_Auto_f->sign == GOBACK)		// 计时器计数值到达 -> 往初始方向反向走
					{ 
						Chassis_Auto_f->sign = GOFORWARD;
						speed_status_count3 = 0;
						speed_time_back_flag += SPEED_TIME_FLAG;																					// 计数值达到指定值 -> 指定值增加一个周期
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
				
				changed_HP = last_remain_HP - referee_remain_HP();										// 变化的血量
				last_remain_HP = referee_remain_HP();																	// 上次血量
				speed_status_count4++;
				
				if(speed_status_count4 > 0xFFF0)																			
					speed_status_count4 = 0;
				
				if(changed_HP >= 10)
			{
					if(speed_status_count4 > SPEED_TIME_FLAG)
				{
						if(speed_status_flag == 0)																			// 边缘转向禁用随机
						{	
							(Chassis_Auto_f->sign == GOBACK)? (Chassis_Auto_f->sign = GOFORWARD) : (Chassis_Auto_f->sign = GOBACK);			// 挨打反向
							chassis_register1 = CHASSIS_BLOCKING_SPPED;
							
							speed_status_count = 0;
							speed_status_count2 = 0; 
							#if (SPEED_MODE == 4)																																												// 防止频繁转向导致电机出错
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

				
				
        else if(Enemy_status == Enemy_Appear)												//敌人出现，底盘速度降低
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
  * @brief          底盘固定
* @param[in]      *Chassis_Blocking_f:  底盘主结构体
  * @retval         none
  */
static void Chassis_Fixation(chassis_control_t * Chassis_Fixation_f)
{

}


/**
  * @brief          底盘走位
* @param[in]      *Chassis_Blocking_f:  底盘主结构体
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
        //积分得出电机转的角度 为后续自动走位处理做准备
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
  * @brief          临界差角处理
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
  * @brief          返回摩擦轮模式
  * @param[in]      none
  * @retval         Friction_wheel_mode
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

