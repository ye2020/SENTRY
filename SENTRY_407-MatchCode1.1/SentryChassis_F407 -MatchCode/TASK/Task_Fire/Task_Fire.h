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
#define PWM_Shoot_Left   TIM4->CCR1  //PD12
#define PWM_Shoot_Right  TIM4->CCR2	 //PD13

/**********拨弹电机速度环pid参数定义**********/
#define FIRE_S_P 12.0f   //供弹电机M2006速度环
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

/**********发弹系统速度设定**********/
//零级：15m/s   一级：18m/s  二级：22m/s  三级：30m/s
#define FRICTION_THREE_SHOOT_SPEED     1226       //30摩擦轮高速pwm
#define FRICTION_TWO_SHOOT_SPEED       1193       //22摩擦轮高速pwm
#define FRICTION_ONE_SHOOT_SPEED       1172       //18摩擦轮高速pwm
#define FRICTION_ZERO_SHOOT_SPEED      1165       //15摩擦轮低速pwm    1160-10.7m/s   1180-17.3m/s   1200-20.4m/s   1220-25m/s   1240
#define FRICTION_SHOOT_STOP            1000       //0摩擦轮停止pwm
#define LOADING_SPEED                  (-3000)    //供弹电机速度




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
    const motor_measure_t *fire_motor_measure;
    const RC_ctrl_t *fire_RC;   //开火任务使用的遥控器指针
    PidTypeDef fire_s_pid;      //拨弹2006电机pid

    Fire_WorkStatus_e fire_workstatus;   //射弹模式
    Shoot_WorkStatus_e Friction_workstatus;  //摩擦轮模式

    int16_t GD_output;          //供弹电机输出
    uint8_t shoot_speed;        //当前设置射速
    uint8_t dead_mode;          //死亡模式开关

} Fire_task_t;



///*全局声明*/
//extern Fire_task_t Fire_param;


extern const Fire_task_t *get_Fire_control_point(void);
extern void FIRE_Task(void *pvParameters);
/**
  * @brief          火控系统关闭
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Fire_Task_OFF(void);

#endif
