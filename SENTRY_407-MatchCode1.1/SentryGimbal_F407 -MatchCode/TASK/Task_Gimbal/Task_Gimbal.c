/**
  ******************************************************************************
  * @file       Task_Gimbal.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */

#include "Task_Gimbal.h"
#include "gimbal_behaviour.h"
#include "maths.h"
#include "rmmotor.h"
#include "filter.h"
#include "RefereeDeal.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "photoelectric.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "IMU.h"
#include "upper_machine.h"

/*--------------------变量-----------------------*/
int gimbal_heart = 0; //云台任务心跳
int yaw_limit_heart = 0;//yaw角度限制心跳


//申明主云台变量
gimbal_control_t gimbal_control;
float Gimbal_set_position = 0;

static float Last_ch3 = 0;
static float Now_ch3 = 0;
 u16 pitch_angle_init_flag = 0;

#if GIMBAL_TEST_MODE
    int32_t pitch_real_jscope = 0;   //P轴打印实际曲线
    int32_t pitch_set_jscope = 0;    //P轴打印设定曲线
    int32_t yaw_real_jscope = 0;     //Y轴打印实际曲线
    int32_t yaw_set_jscope = 0;      //Y轴打印设定曲线
    int32_t sec_yaw_set_jscope = 0;  //副Y轴打印设定曲线
    int32_t sec_yaw_real_jscope = 0; //副Y轴打印实际曲线
    int32_t accel_up_jscope = 0;
    int32_t accel_down_jscope = 0;
		
		int32_t Vision_pitch_jscope = 0;
		int32_t Vision_yaw_jscope = 0;

#endif


/*--------------------函数-----------------------*/
static void Gimbal_Task_Control(gimbal_control_t *gimbal_task_control);
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose);
static void Gimbal_data_init(gimbal_control_t *gimbal_initialization);
#if GIMBAL_TEST_MODE
    /* jscope打印曲线 */
    static void Gimba_jscope_print_curve(void);
#endif
 u16 abcde = 0;
/* 云台主任务 */
void GIMBAL_Task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    Gimbal_data_init(&gimbal_control);

   

    while (1)
    {
	
			if (pitch_angle_init_flag == 0 )
				gimbal_pitch_init();
		
			else 
	 {
				#if yaw_angle_limit
						yaw_limit_heart = 1;
				#endif
				

//				printf (" %d \r\n",get_hurt_status());
				
				LEDE2 = gimbal_heart;   //云台任务心跳
				LEDE1 = yaw_limit_heart;//yaw 轴角度限制心跳
				LEDE0 = 1;							//遥控
			
				pid_parameter_receive(&gimbal_control.yaw_c.yaw_s_pid,&gimbal_control.yaw_c.yaw_p_pid);				// 蓝牙
			
        Gimbal_Task_Control(&gimbal_control); //云台工作

        /* 进行pitch轴3508电机和两个拨弹电机的控制 */
        CAN1_Gimbal_Fire(gimbal_control.Fire_task_control->GDA_output,
                         gimbal_control.Fire_task_control->GDB_output,
                         gimbal_control.pitch_c.output,//
                         0); //两个拨弹为- +
			

// 
				/* 进行yaw轴3508电机 */
				CAN2_yaw_Setmsg(gimbal_control.yaw_c.output);
        vTaskDelay(GIMBAL_CONTROL_TIME_MS); //系统延时
		
		
	 }
    }
}


/**
  * @brief          云台数据初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Gimbal_data_init(gimbal_control_t *Gimbal_data_init_f)
{
    /*--------------------获取指针--------------------*/
    {

        //获取遥控器指针(数据)
        Gimbal_data_init_f->gimbal_RC = Get_Remote_Control_Point();
        //获取云台指针
        Gimbal_data_init_f->yaw_c.yaw_motor_measure = Get_Yaw_Gimbal_Motor_Measure_Point();
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = Get_Pitch_Gimbal_Motor_Measure_Point();
        //获取自瞄指针
        Gimbal_data_init_f->auto_c = Get_Auto_Control_Point();
        Gimbal_data_init_f->Fire_task_control = get_Fire_control_point();
    }

    /*--------------------检查遥控器数值是否正确--------------------*/
    RC_data_is_error();

    #if IMU_BMI160
    /*--------------------陀螺仪初始化--------------------*/
    {
        BMI160_Zero_Correct();  //发送校准信号给陀螺仪模块
        Gimbal_data_init_f->yaw_c.last_angle = IMU_t.yaw_angle; //更新初始角度
    }
    #endif

    /*--------------------滤波初始化--------------------*/
    {
        //pitch轴低通滤波
        first_order_filter_init(&Gimbal_data_init_f->pitch_c.LowFilt_Pitch_Data, Gimbal_Pitch_Fir_Ord_Low_Fil_Param);
        //初始化P轴滑动滤波器
        Sliding_Mean_Filter_Init(&Gimbal_data_init_f->pitch_c.Slidmean_Pitch_Data);
    }

    /*--------------------PID初始化--------------------*/
    {
#if (PITCH_PID_MODE == 1)		
        //Pitch上升的pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_p_pid, GIMBAL_UP_P_PITCH_P, GIMBAL_UP_P_PITCH_I, GIMBAL_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_s_pid, GIMBAL_UP_S_PITCH_P, GIMBAL_UP_S_PITCH_I, GIMBAL_UP_S_PITCH_D, 0, 0);
        //Pitch下降的pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_p_pid, GIMBAL_DOWN_P_PITCH_P, GIMBAL_DOWN_P_PITCH_I, GIMBAL_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_s_pid, GIMBAL_DOWN_S_PITCH_P, GIMBAL_DOWN_S_PITCH_I, GIMBAL_DOWN_S_PITCH_D, 0, 0);
				
				//自瞄：Pitch的上升pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_p_pid, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_s_pid, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, 0, 0);
			
        //自瞄：Pitch的下降pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_p_pid, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_s_pid, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D, 0, 0);



#elif (PITCH_PID_MODE == 2)	
			
        //Pitch pid参数初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, GIMBAL_P_PITCH_P, GIMBAL_P_PITCH_I, GIMBAL_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;   //180.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -200.0f;  //-250.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 10.0f;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 1;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8;
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, GIMBAL_S_PITCH_P, GIMBAL_S_PITCH_I, GIMBAL_S_PITCH_D, 0, 0);
			
			//自瞄：Pitch的pid参数初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.maximum = 180.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.minimum = -250.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.stepIn = 7.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmin = 10;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmax = 30;
		pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0);
#endif			
        //Yaw的pid参数初始化
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, GIMBAL_P_YAW_P, GIMBAL_P_YAW_I, GIMBAL_P_YAW_D, 0, 0);
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, GIMBAL_S_YAW_P, GIMBAL_S_YAW_I, GIMBAL_S_YAW_D, 0, 0);

        //自瞄：Yaw的pid参数初始化
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_YAW_P, GIMBAL_AUTO_INDUSTRY_P_YAW_I, GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_YAW_P, GIMBAL_AUTO_INDUSTRY_S_YAW_I, GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);

    }

    /*--------------------设置开机状态--------------------*/
    {
        Gimbal_data_init_f->Gimbal_all_flag = 0;
        Gimbal_data_init_f->gimbal_behaviour = GIMBAL_STOP;  //只有刚刚开机的时候才能为 STOP
        Gimbal_Stop(Gimbal_data_init_f);
    }
		
		
}


void gimbal_pitch_init(void)
{ 
	#if (pitch_angle_position == 0)
		CAN1_Gimbal_Fire(0,0,-4000,0);
	vTaskDelay(2); //系统延时
	
		if(gimbal_control.pitch_c .pitch_motor_measure -> speed < 100) //
		{
			abcde ++;
			if(abcde > 250 )
			{
				gimbal_control.pitch_c.pitch_motor_measure->actual_Position = 0;
				pitch_angle_init_flag = 1;
			}
		}
		else 
			abcde = 0;
	#endif		
	#if (pitch_angle_position == 1)	
			pitch_angle_init_flag = 1;

	#endif
}


/**
  * @brief          云台状态控制
  * @param[in]      none
  * @retval         none
  * @attention
  */

static void Gimbal_Task_Control(gimbal_control_t *gimbal_task_control)
{

    #if IMU_BMI160
    IMU_Data_Deal(); //接收陀螺仪数值

    if (gimbal_task_control->Gimbal_all_flag == 1)
    {
        Gyro_usart_iwdg();
    } //给陀螺仪喂狗粮 在右边为下的时候不会进入这里，所以陀螺仪数据初始化，angle为0

    BMI160_Zero_Correct();
    #endif
		

		
    MiniPC_Data_Deal();  //通过串口三接收并保存MiniPC发来的数据
	
    MiniPC_Send_Data(get_Enemy_status(),Return_Pitch_angle(gimbal_control.pitch_c .pitch_motor_measure ->pitch_angle ), get_bullet_speed_from_chassis());  //发送敌人颜色，p轴角度，射速给视觉
//    MiniPC_Send_Data(0,Return_Pitch_angle(gimbal_control.pitch_c .pitch_motor_measure ->pitch_angle ), get_bullet_speed_from_chassis());  //发送敌人颜色，p轴角度，射速给视觉

    //云台模式选择
    Gimbal_remote_mode_choose(gimbal_task_control);
}


/**
  * @brief          云台行为选择
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose)
{
    Gimbal_behaviour_mode_set(fir_gimbal_choose);
}

/**
  * @brief          云台正常控制
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3)
{

    #if IMU_BMI160
    gimbal_working->yaw_c.angle = IMU_t.yaw_angle - gimbal_working->yaw_c.last_angle; //获取云台正常工作时的陀螺仪Y轴角度
    #else
//    gimbal_working->yaw_c.angle = 0.0f; // 云台不会保持一个绝对角度
    #endif


    /* 主云台控制 前一个变量为遥控器控制，后一个变量为控制地面绝对角度 */
    Gimbal_set_position = loop_fp32_constrain(( gimbal_ch2 - (YAW_ANGLE_FLAG * gimbal_working->yaw_c.yaw_motor_measure->yaw_angle)), -180.0f, 180.0f); //云台设定位置循环限幅

#if (PITCH_PID_MODE == 1)
    /* P轴加速度 速度的PID微分 */
    gimbal_working->pitch_c.accel_down = gimbal_working->pitch_c.pitch_down_s_pid.Derror[0] * 100.0f;
    gimbal_working->pitch_c.accel_up = gimbal_working->pitch_c.pitch_up_s_pid.Derror[0] * 100.0f;
#endif
    //	Data_Accelerated_Control();

    /* P轴滤波 */
    gimbal_working->pitch_c.output = Sliding_Mean_Filter(&gimbal_working->pitch_c.Slidmean_Pitch_Data, gimbal_working->pitch_c.output, 55); //均值滑窗滤波（有滞后）
    gimbal_working->pitch_c.output = first_order_filter(&gimbal_working->pitch_c.LowFilt_Pitch_Data, gimbal_working->pitch_c.output);       //一阶低通滤波

    Now_ch3 = gimbal_ch3;
    Last_ch3 = Now_ch3;

#if (PITCH_PID_MODE == 1)

        if (Now_ch3 - Last_ch3 > 0.0f)
        {
            /* Pitch位置控制 */
            gimbal_working->pitch_c.output = Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_up_s_pid,
                                             &gimbal_working->pitch_c.pitch_up_p_pid,
                                             gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,             /*真实位置*/
                                             gimbal_working->pitch_c.pitch_motor_measure->speed,                   /*真实速度*///IMU_t.Gyro_X
                                             (gimbal_ch3), /*设定位置*///布瑞特编码器:1024  3508:8192
                                             PITCH_OUTPUT_LIMIT);                                         				 /*输出限制*/
        }
        else
        {
            /* Pitch位置控制 */
            gimbal_working->pitch_c.output = Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_down_s_pid,
                                             &gimbal_working->pitch_c.pitch_down_p_pid,
                                             gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,               /*真实位置*/
                                             gimbal_working->pitch_c.pitch_motor_measure->speed,                     /*真实速度*///IMU_t.Gyro_X
                                             (gimbal_ch3),  /*设定位置*/
                                             PITCH_OUTPUT_LIMIT);                                             		   /*输出限制*/
        }
#elif (PITCH_PID_MODE == 2)

		gimbal_working->pitch_c.output = motor_position_Stepping(&gimbal_working->pitch_c.pitch_s_pid,
																	  &gimbal_working->pitch_c.pitch_p_pid,
																	  gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,                                                           /*真实位置*/
                                    gimbal_working->pitch_c.pitch_motor_measure->speed,                   /*真实速度*///IMU_t.Gyro_X
																		(gimbal_ch3)	,				// -((gimbal_ch3) - (gimbal_working->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024)), /*设定位置*/
																	  PITCH_OUTPUT_LIMIT); 
#endif
        /* Yaw位置控制 以哨兵的方向来讲 yaw轴左正右负 pitch轴上负下正 */
        gimbal_working->yaw_c.output = Motor_Position_Speed_Control(&gimbal_working->yaw_c.yaw_s_pid,
                                       &gimbal_working->yaw_c.yaw_p_pid,
                                       0,                          /*真实位置*///gimbal_working->yaw_c.yaw_motor_measure->yaw_angle
                                       gimbal_working->yaw_c.yaw_motor_measure->speed, 															/*真实速度*/
                                       (Gimbal_set_position),                            																		/*设定位置*/
                                       YAW_OUTPUT_LIMIT);                               														/*输出限制*/

				
    #if GIMBAL_TEST_MODE
    /*打印曲线*/
    Gimba_jscope_print_curve();
    #endif
}


/**
  * @brief          自瞄模式
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f)
{

    /*输出量闭环处理*/
    gimbal_automatic_work_f->yaw_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->yaw_c.yaw_auto_s_pid,
                                            &gimbal_automatic_work_f->yaw_c.yaw_auto_p_pid,
                                            0,																																					/*真实位置*/
                                            gimbal_automatic_work_f->yaw_c.yaw_motor_measure->speed,
                                            (gimbal_automatic_work_f->auto_c->yaw_control_data),												/*设定位置*/
                                            YAW_OUTPUT_LIMIT); //Yaw位置控制
#if (PITCH_PID_MODE == 1)
    Now_ch3 = gimbal_automatic_work_f->pitch_c.Auto_record_location;
    Last_ch3 = Now_ch3;
    if (Now_ch3 - Last_ch3 > 0.0f)//max 未测		min -10
    {
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*真实位置*/
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*设定位置*/
																									PITCH_OUTPUT_LIMIT);
		}
		else
		{
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*真实位置*///
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*设定位置*/
																									PITCH_OUTPUT_LIMIT);
		}
		
#elif (PITCH_PID_MODE == 2)		
				gimbal_automatic_work_f->pitch_c.output = motor_position_Stepping(&gimbal_automatic_work_f->pitch_c.pitch_auto_s_pid, 
																		  &gimbal_automatic_work_f->pitch_c.pitch_auto_p_pid, 
																		  -gimbal_automatic_work_f->pitch_c.Auto_record_location ,  
																		  gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																		  (-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data), 
																		  PITCH_OUTPUT_LIMIT);
#endif		
		
    #if GIMBAL_TEST_MODE
    /*打印曲线*/
    Gimba_jscope_print_curve();
    #endif		
		
}


/**
  * @brief          云台初始化检测
  * @param[in]      none
  * @retval         none
  * @attention
  */
u8 Return_Gimbal_Mode(void)
{
//    if (gimbal_control.yaw_c.init_flag == 1 && gimbal_control.pitch_c.init_flag == 1 && gimbal_control.sec_gimbal_control.init_flag == 1)
//    {
    return 1; //初始化完成
//    }

//    if (gimbal_control.Gimbal_all_flag == 1)
//    {
//        return 1; //初始化未完成
//    }

//    return 0;
}


// pitch角度处理成 15~85度传给视觉
uint8_t Return_Pitch_angle(int32_t Pitch_angle)
{
		Pitch_angle += 49;
		Pitch_angle =90 - Pitch_angle;
		return Pitch_angle;
}


#if GIMBAL_TEST_MODE
/* jscope打印曲线 */
static void Gimba_jscope_print_curve(void)
{
    pitch_real_jscope = (gimbal_control.pitch_c.pitch_motor_measure->speed)*5; //P轴打印实际曲线
    pitch_set_jscope = gimbal_control.pitch_c.output;                      //P轴打印设定曲线

    yaw_real_jscope = gimbal_control.yaw_c.yaw_motor_measure->speed; //Y轴打印实际曲线
    yaw_set_jscope = gimbal_control.yaw_c.output;                    //Y轴打印设定曲线

    sec_yaw_set_jscope = 0;  //副Y轴打印设定曲线
    sec_yaw_real_jscope = 0; //副Y轴打印实际曲线

    accel_up_jscope = gimbal_control.pitch_c.accel_up;
    accel_down_jscope = gimbal_control.pitch_c.accel_down;
	
		Vision_pitch_jscope = gimbal_control.auto_c->auto_pitch_angle;
		Vision_yaw_jscope   = gimbal_control.auto_c->auto_yaw_angle;

}
#endif


