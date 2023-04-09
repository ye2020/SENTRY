/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      完成底盘控制任务。
  ******************************************************************************
  */
#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H
#include "main.h"
#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "Task_Fire.h"
#include "maths.h"
#include "pid.h"


/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

//底盘测试模式 宏定义 0 为不使用测试模式
#define CHASSIS_TEST_MODE   1

//底盘功率限制 要用就设为1
#define power_limit  1

/**********************底盘电机pid参数**********************/
#define CHASSIS_MOTOR1_PID_Kp    8.0f
#define CHASSIS_MOTOR1_PID_Ki    2.0f
#define CHASSIS_MOTOR1_PID_Kd    0.0f

#define CHASSIS_MOTOR2_PID_Kp    8.0f
#define CHASSIS_MOTOR2_PID_Ki    2.0f
#define CHASSIS_MOTOR2_PID_Kd    0.0f

#define CHASSIS_MOTOR3_PID_Kp    8.0f
#define CHASSIS_MOTOR3_PID_Ki    0.0f
#define CHASSIS_MOTOR3_PID_Kd    0.2f

#define CHASSIS_MOTOR4_PID_Kp    8.0f
#define CHASSIS_MOTOR4_PID_Ki    0.0f
#define CHASSIS_MOTOR4_PID_Kd    0.2f

#define CHASSIS_LOCATION_PID_P  100.0f
#define CHASSIS_LOCATION_PID_I  10.0f
#define CHASSIS_LOCATION_PID_D  10.8f

#define CHASSIS_ROTATE_FOLLOW_P  10.0f   //底盘静止跟随PID   8.0
#define CHASSIS_ROTATE_FOLLOW_I  0.0f   //0.01
#define CHASSIS_ROTATE_FOLLOW_D  1.8f   //5.02   10.02

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f

/**********************功率限制参数**********************/
#define PowerLimit_Param  6.5f         //功率限制输出参数
#define PowerLimit_Thres  190.0f        //功率限制阈值（缓冲功率剩余量）50.0f

/**********************运动加速度限制**********************/
#define STRAIGHT_ACCELERAD        3.5f      //直行底盘加速度限制
#define TRANSLATION_ACCELERAD     5.5f      //平移底盘加速度限制
#define ROTATING_ACCELERAD        19.0f     //旋转底盘加速度限制

#define CHASSIS_ROTATION_SPEED    600      //小陀螺的旋转速度  2000
#define CHASSIS_ROTATION_MOVE_SPEED  600   //小陀螺移动时为防止轨迹失真减转速   1700
#define CHASSIS_TWIST_SPEED       600      //扭腰速度  1600
#define CHASSIS_AUTO_SPPED				7000			 //自动速度  3000					
#define CHASSIS_BLOCKING_SPPED		6700 //6700//	900			 //走位速度  7000
#define CHASSIS_AUTO_MAX_SPEED		4800			 //自动限速  4800
#define CHASSIS_AUTO_SLOW_SPPED				0			// 遇到敌人 速度减慢		5.17   3000->0  5.25


#define NORMAL_FORWARD_BACK_SPEED 	300.0f   //键盘普通直行速度
#define NORMAL_LEFT_RIGHT_SPEED   	300.0f   //键盘普通平移速度

#define HIGH_FORWARD_BACK_SPEED 	600.0f     //键盘加速直行速度
#define HIGH_LEFT_RIGHT_SPEED   	600.0f     //键盘加速平移速度

#define SPEED_TIME_FLAG				430	//510			//变速时间
#define SPEED_MODE						  	4//底盘变速：1 -> 不变速； 2 -> 变速  ; 3 -> 变速 + 变向1 ;4 -> 变向2
/*底盘模式*/
typedef enum
{
    CHASSIS_STOP,       //停止
    CHASSIS_INITIALIZE, //初始化中
    CHASSIS_STANDBY,    //待机

    //CHASSIS_WORKING,   //工作

    CHASSIS_FOLLOW,    //跟随
    CHASSIS_NO_FOLLOW, //不跟随

    CHASSIS_TWIST_WAIST, //扭腰
    CHASSIS_ROTATION,    //小陀螺
    CHASSIS_BATTERY,     //炮台模式

    CHASSIS_REMOTECONTROL,//遥控模式
    CHASSIS_AUTO,				   //自动模式
    CHASSIS_BLOCKING,      //走位模式
    CHASSIS_FIXATION			// 底盘固定模式
} Chassis_mode_e;


/*敌人颜色*/
typedef enum
{
    Enemy_color_red = 0,    //敌人颜色为红色
    Enemy_color_blue		//敌人颜色为蓝色

} Enemy_color_e;
/*键盘按键结构体*/
typedef struct
{
    float KEY_W;
    float KEY_S;
    float KEY_A;
    float KEY_D;
    u8 KEY_SHIFT;
    u8 KEY_CTRL;
    u8 KEY_Q;   //整车左旋
    u8 KEY_E;   //整车右旋
    u8 KEY_R;   //小陀螺
    u8 KEY_F;   //扭腰
    u8 KEY_G;   //补给模式
    u8 KEY_Z;   //退弹
    u8 KEY_X;   //
    u8 KEY_C;   //低射速
    u8 KEY_V;   //高射速
    u8 KEY_B;   //
    u8 Mouse_L; //开火
    u8 Mouse_R; //自动瞄准
    u16 Mouse_X; //云台左右
    u16 Mouse_Y; //云台上下
} Key_Pressed_Sigh_t;

/*底盘电机数据*/
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 position;
    fp32 last_position;
    int16_t output;
} Motor_t;

/*底盘功率限制*/
typedef struct Power_Limit
{
    float Real_Power[3]; //2最新 1上次 0上上次
    float RemainPower[3];
    int32_t SumOutValue;
    int32_t LimitOutValue;
    float scale;
} PowerLimit_t;

/*底盘整体数据结构体*/
typedef struct
{
    const RC_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
    const gimbal_yaw_receive_t *gimbal_re_data; //云台板处理数据，数据由can2传输给底盘板，底盘板再输出给yaw轴电机
    const Fire_task_t *Fire_task_control;
    motor_measure_t *yaw_motor_measure;      //can1直接接收的yaw轴数据
    motor_measure_t *chassis_motor_measure;
    Key_Pressed_Sigh_t  Key_Press;

    PowerLimit_t Chassis_PowerLimit; //底盘功率限制结构体
    Motor_t chassis_motor[4];        //底盘电机数据(包含电机统一结构体指针)
    PidTypeDef chassis_speed_pid[4]; //正常移动pid
    PidTypeDef chassis_location_pid; //底盘位置环pid
    PidTypeDef chassis_rotate_pid;   //旋转pid

    Chassis_mode_e chassis_mode; //底盘控制状态机

    first_order_filter_type_t LowFilt_chassis_vx; //低通滤波器
    first_order_filter_type_t LowFilt_chassis_vy; //低通滤波器

//	fp32 speed_x;                         //底盘速度 前进方向 前为正，单位 m/s
//	fp32 speed_y;                         //底盘速度 左右方向 左为正  单位 m/s
//	fp32 speed_z;                         //底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 speed_x_set; //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 speed_y_set; //底盘设定速度 左右方向 左为正，单位 m/s
    fp32 speed_z_set; //底盘设定旋转角速度，逆时针为正 单位 rad/s

    fp32 chassis_gimbal_angel; //底盘与云台的角度

    uint8_t SuperCap_discharge_flag;  //超级电容放电标志位
    uint8_t Chassis_keyboad;

    u8 sign;								//前后走标志
    u8 sign_last;						//延续前后走标志

//	fp32 max_speed_x;   //前进方向最大速度 单位m/s
//	fp32 min_speed_x;   //前进方向最小速度 单位m/s
//	fp32 max_speed_y;   //左右方向最大速度 单位m/s
//	fp32 min_speed_y;   //左右方向最小速度 单位m/s

//	fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
//	fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
//	fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
} chassis_control_t;


//底盘主任务
extern void CHASSIS_Task(void *pvParameters);
//底盘控制量设置
extern void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2);

extern u8 Return_Chassis_Mode(void);

void Chassis_Task_OFF(u8 options);

uint8_t  automatic_Enemy_color(void);


#endif
