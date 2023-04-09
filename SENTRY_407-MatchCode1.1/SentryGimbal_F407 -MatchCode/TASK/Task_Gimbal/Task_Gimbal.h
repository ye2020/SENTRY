/**
  ******************************************************************************
  * @file       Task_Gimbal.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */

#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H
#include "main.h"
#include "pid.h"
#include "Task_Fire.h"
#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "automatic_strike.h"
#include "usart2.h"


/*OS控制任务周期以及启动时间*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE    1
#define PITCH_PID_MODE  2

/**********************P陀螺仪角速度环**********************/

#if (PITCH_PID_MODE == 1)

    #define GIMBAL_UP_P_PITCH_P  40.5f   //P位置环  53   410    160       58  41.5
    #define GIMBAL_UP_P_PITCH_I  0.0f     //8f
    #define GIMBAL_UP_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_UP_S_PITCH_P  20.5f   //P速度环(不要加i)       1.5     9.5
    #define GIMBAL_UP_S_PITCH_I  0.0f
    #define GIMBAL_UP_S_PITCH_D  3.8f

    #define GIMBAL_DOWN_P_PITCH_P  38.0f   //P位置环  53   410    160       58
    #define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f
    #define GIMBAL_DOWN_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_DOWN_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
    #define GIMBAL_DOWN_S_PITCH_I  0.0f
    #define GIMBAL_DOWN_S_PITCH_D  3.8f

#elif (PITCH_PID_MODE == 2)


    #define GIMBAL_P_PITCH_P  60.0f//60.0f   //P位置环  70
    #define GIMBAL_P_PITCH_I 	1.5f			//1.5f     // 1.1f
    #define GIMBAL_P_PITCH_D  75.0f//75.0f     //0.0f   75

    #define GIMBAL_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
    #define GIMBAL_S_PITCH_I  0.0f
    #define GIMBAL_S_PITCH_D  6.0f	//6.0f    //3.5  4

#endif
/**********************外接陀螺仪**********************/
#define GIMBAL_P_YAW_P  130.0f   //Y位置环    150     62    130
#define GIMBAL_P_YAW_I  0.0f
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  9.0f     //Y速度环     8      10     9
#define GIMBAL_S_YAW_I  0.0f
#define GIMBAL_S_YAW_D  0.0f     //2                   0

/**********************低通滤波比例**********************/
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f

/**********************云台pitch和yaw角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP    50
#define PITCH_ANGLE_LIMIT_DOWN  (-15)
#define YAW_ANGLE_LIMIT         15

/**********************运动加速度限制**********************/
#define GIMBAL_PITCH_ACCELERAD    2         //云台俯仰加速度限制
#define GIMBAL_YAW_ACCELERAD      2         //云台偏航加速度限制
#define GIMBAL_AUTO_YAW_ACCELERAD 1         //云台自动偏航加速度限制

/**********************pitch和yaw输出量限制**********************/
#define YAW_OUTPUT_LIMIT         11000
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000
#define PITCH_INIT_OUTPUT_LIMIT  5000

/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED       0.011f    //鼠标yaw轴速度增益     0.021f 
#define MOUSE_PITCH_SPEED     0.009f     //鼠标pitch轴速度增益  0.13
#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026



/*********视觉PY轴数据pid参数定义***************/

/**后面视觉需要三套pid，自瞄短焦，自瞄工业，打符工业**/

//短焦摄像头4mm
#define GIMBAL_AUTO_SHORT_P_PITCH_P 20.5f     //P自动位置环   7.5
#define GIMBAL_AUTO_SHORT_P_PITCH_I 0.0f
#define GIMBAL_AUTO_SHORT_P_PITCH_D 0.1f

#define GIMBAL_AUTO_SHORT_S_PITCH_P 20.5f     //P自动速度环   13.5
#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f
#define GIMBAL_AUTO_SHORT_S_PITCH_D 0.0f

#define GIMBAL_AUTO_SHORT_P_YAW_P 15.0f       //Y自动位置环   150
#define GIMBAL_AUTO_SHORT_P_YAW_I 0.0f
#define GIMBAL_AUTO_SHORT_P_YAW_D 0.0f

#define GIMBAL_AUTO_SHORT_S_YAW_P 15.6f       //Y自动速度环    7.6
#define GIMBAL_AUTO_SHORT_S_YAW_I 0.0f
#define GIMBAL_AUTO_SHORT_S_YAW_D 0.0f

//工业摄像头

#if (PITCH_PID_MODE == 1)
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P 57.0f    //P自动位置环  530.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D 3.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P 25.0f     //P自动速度环
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_D 3.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P 20.0f    //P自动位置环  530.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D 0.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P 10.0f     //P自动速度环
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D 0.0f


#elif (PITCH_PID_MODE == 2)
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 36.0f//30.0f    //P自动位置环  530.0f   35   30
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.1f
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 10.0f     //P自动速度环  12  8
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5
#endif

#define GIMBAL_AUTO_INDUSTRY_P_YAW_P 100.0f//63.0f//60.0f       //Y自动位置环 170
#define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f
#define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f

#define GIMBAL_AUTO_INDUSTRY_S_YAW_P 15.0f       //Y自动速度环 12
#define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f
#define GIMBAL_AUTO_INDUSTRY_S_YAW_D 1.0f




typedef enum
{
//	GIMBAL_ZERO_FORCE = 0, //云台无力
//	GIMBAL_INIT,           //云台初始化
//	GIMBAL_CALI,           //云台校准
//	GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
//	GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
//	GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移

    GIMBAL_STOP,         //停止
    GIMBAL_INITIALIZE,   //初始化状态
    GIMBAL_STANDBY,      //待机状态

    GIMBAL_WORKING,      //初始化结束位,暂时没什么用
    GIMBAL_REMOTECONTROL,       //手动状态

    GIMBAL_AUTOCONTROL,  //比赛模式
    GIMBAL_PATROl,       //巡逻状态
    GIMBAL_AUTOATTACK,   //自瞄状态
    GIMBAL_AUTOBUFF,     //打符状态
    GIMBAL_REPLENISHMEN, //补给状态

    GIMBAL_DOUBLE_GIMBAL,//双云台状态（主云台自瞄，副云台操作）
} gimbal_behaviour_e;


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


typedef struct  //申明副云台电机变量
{
    const motor_measure_t *sec_gimbal_motor_measure;

    PidTypeDef second_yaw_pid;  //pid

    int8_t init_flag;           //初始化成功标志
    int8_t photoelectric_zero;  //副Y轴中值光电标志

    int16_t Yaw_different_angle;       //副云台相对主云台的差角
    int16_t chassis_different_angle;   //副云台相对底盘的差角

    int16_t output;

} gimbal_second_control_t;

typedef struct  //申明pitch轴电机变量
{
    motor_measure_t *pitch_motor_measure;
    #if (PITCH_PID_MODE == 1)

    PidTypeDef pitch_up_p_pid;  //pid
    PidTypeDef pitch_up_s_pid;  //pid

    PidTypeDef pitch_down_p_pid;  //pid
    PidTypeDef pitch_down_s_pid;  //pid

    PidTypeDef pitch_down_auto_p_pid;  //pid
    PidTypeDef pitch_down_auto_s_pid;  //pid

    PidTypeDef pitch_up_auto_p_pid;  //pid
    PidTypeDef pitch_up_auto_s_pid;  //pid
    #elif (PITCH_PID_MODE == 2)
    PidTypeDef pitch_p_pid;  //pid
    PidTypeDef pitch_s_pid;  //pid

    PidTypeDef pitch_auto_p_pid;  //pid
    PidTypeDef pitch_auto_s_pid;  //pid
    #endif
    float accel_up;
    float accel_down;

    int8_t init_flag;    //初始化成功标志

    float Auto_record_location;


    first_order_filter_type_t LowFilt_Pitch_Data;    //P轴低通滤波器
    sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P轴滑动滤波器

    first_order_filter_type_t LowFilt_auto_pitch;    //自瞄P轴低通滤波器
    sliding_mean_filter_type_t Slidmean_auto_pitch;  //自瞄P轴滑动滤波器

    int16_t filt_output; //P轴滤波值

    int16_t output;

} gimbal_pitch_control_t;

typedef struct  //申明yaw轴电机变量
{
    const motor_measure_t *yaw_motor_measure;

    PidTypeDef yaw_p_pid;  //pid
    PidTypeDef yaw_s_pid;  //pid

    PidTypeDef yaw_auto_p_pid;  //pid
    PidTypeDef yaw_auto_s_pid;  //pid


    bool_t init_flag;           //Y轴初始化成功标志
    int8_t photoelectric_zero;  //Y轴中值光电标志

//	int16_t chassis_different_angle;  //云台底盘差角

    float angle;              //云台当前角度
    float last_angle;         //云台保存角度

    int16_t filt_output;  //Y轴滤波值

    int16_t output;

} gimbal_yaw_control_t;


typedef struct
{
    const RC_ctrl_t *gimbal_RC; //底盘使用的遥控器指针
    Vision_Auto_Data_t *auto_c;    //申明自瞄变量

    const gimbal_yaw_receive_t *gimbal_re_data; //云台板处理数据，数据由can2传输给底盘板，底盘板再输出给yaw轴电机
    const Fire_task_t *Fire_task_control;
    gimbal_behaviour_e gimbal_behaviour;
    Key_Pressed_Sigh_t  Key_Press;

    gimbal_second_control_t sec_gimbal_control;  //申明副云台电机变量
    gimbal_pitch_control_t pitch_c;   //申明pitch轴电机变量
    gimbal_yaw_control_t yaw_c;       //申明yaw轴电机变量
    motor_measure_t motor_chassis[4];

    u8 Gimbal_keyboad;
    u8 Gimbal_all_flag;  //全部初始化完成标志
    u8 Gimbal_supply_flag;

} gimbal_control_t;


/* 云台主任务 */
extern void GIMBAL_Task(void *pvParameters);
/* 云台行为 */
extern void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f);
extern u8 Return_Gimbal_Mode(void);
/*  处理pitch角度传给视觉 */
uint8_t Return_Pitch_angle(int32_t Pitch_angle);
void gimbal_pitch_init(void);


#endif
