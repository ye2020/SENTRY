#ifndef __FIRETASK_H
#define __FIRETASK_H
#include "main.h"
#include "pid.h"
#include "RemoteControl.h"
#include "CAN_1_Receive.h"


/*OS控制任务周期以及启动时间*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

/*摩擦轮*/
#define PWM_Shoot_Upper_Left   TIM3->CCR1  //PA6
#define PWM_Shoot_Lower_Left   TIM3->CCR2	//PA7
#define PWM_Shoot_Upper_Right  TIM5->CCR3  //PA2
#define PWM_Shoot_Lower_Right  TIM5->CCR4	 //PA3



/**********拨弹电机速度环pid参数定义**********/
#define FIRE_S_P 12.0f   //供弹电机M2006速度环
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

#define FIRE_P_P 50.0f
#define FIRE_P_I 0.1f  //0 TODO
#define FIRE_P_D 0.1f

/**********发弹系统速度设定**********/
//零级：15m/s   一级：18m/s  二级：22m/s  三级：30m/s
#define FRICTION_THREE_SHOOT_SPEED    1070  //	1400				//(2000 - 1215)//   24 m/s// (2000 - 1205) //30摩擦轮高速pwm			1205 -> 22.5m/s     (1243)
#define FRICTION_TWO_SHOOT_SPEED       	1020					//(2000 - 1193)     //22摩擦轮高速pwm			
#define FRICTION_ONE_SHOOT_SPEED      1000 //	1298					//(2000 - 1160)//(2000 - 1187)       //18摩擦轮高速pwm		原1173  13.5m/s			（1187）
#define FRICTION_ZERO_SHOOT_SPEED     1000	//1260					 //(2000 - 1172)       //15摩擦轮低速pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define FRICTION_SHOOT_STOP            	800						//	(2000 - 1000)       //0摩擦轮停止pwm
#define LOADING_SPEED         (-500)//(-3300)    //供弹电机速度  //5.2 2000调整为2900


#define	LOADING_STOP_TIMES  700



/* 拨弹电机状态*/
typedef enum
{
	motor_A_blocking_mode,		// 拨弹电机A堵转模式
	motor_B_blocking_mode,		// 拨弹电机B堵转模式
	normal_mode,			// 正常模式

} Loading_motor_e;



/*发弹模式*/
typedef enum
{
    FIRE,          //发射
    AUTO_FIRE,     //自动发射
    STOP_FIRE,     //停止发射
    BACK,          //退弹
    FIRE_ERROR,

} Fire_WorkStatus_e;


/*摩擦轮模式*/
typedef enum
{
    LOW_SPEED,
    HIGH_SPEED,
    STOP_SHOOT,
    SHOOT_ERROR,

} Shoot_WorkStatus_e;

/*火控结构体*/
typedef struct Fire_param
{
    const motor_measure_t *fire_motorA_measure;
		const motor_measure_t *fire_motorB_measure;
    const RC_ctrl_t *fire_RC;   //开火任务使用的遥控器指针
    PidTypeDef fire_s_pid;      //拨弹2006电机pid
		PidTypeDef fire_p_pid;			//拨弹2006电机位置pid

	
		Loading_motor_e Loading_motorStatus; //拨弹电机状态
    Fire_WorkStatus_e fire_workstatus;   //射弹模式
    Shoot_WorkStatus_e Friction_workstatus;  //摩擦轮模式

    int16_t GDA_output;          //供弹电机输出
		int16_t GDB_output;          //供弹电机输出
    uint8_t shoot_speed;        //当前设置射速
    uint8_t dead_mode;          //死亡模式开关
		uint16_t hot_max;						//枪口热量上限
		uint16_t hot_current;				//枪口热量
		float GD_speed_gain;				//热量限制 增益值

} Fire_task_t;



///*全局声明*/
//extern Fire_task_t Fire_param;

static void snail_motor_pwm(uint32_t snial_pwm);			

extern const Fire_task_t *get_Fire_control_point(void);
extern void FIRE_Task(void *pvParameters);

#endif
