#include "Task_Test.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "RemoteControl.h"
#include "pid.h"
#include "rmmotor.h"

#ifdef TEST_MODE
PidTypeDef Motor_2006_test_PID;


static float test_motor_2006 = 0.0f;
//static float test_2006_pid_P_out_jscope;
//static float test_2006_pid_I_out_jscope;
//static float test_2006_pid_D_out_jscope;


static void Motor_2006_fire_test(void);
static void Motor_2006_speed_test(void);
	

/* 用于测试各种电机 */
void TEST_Task(void *pvParameters)
{
	//测试电机pid
	pid_init(&Motor_2006_test_PID, 8.0f, 0.0f, 0.2f, 0, 0);
	
    while (1)
    {
		//检查遥控器数值是否正确
		RC_data_is_error();
		
//		limit_int16((-(Remote_data.RC_ch3)) + 140, 169, 111);
//		delay_ms(1000);
//		TIM3->CCR1 = 175;
//		delay_ms(1000);
//		TIM3->CCR1 = 180;
//		delay_ms(1000);
//		TIM3->CCR1 = 185;
//		delay_ms(1000);
//		TIM3->CCR1 = 190;
//		delay_ms(1000);
//		TIM3->CCR1 = 195;
		
//		CAN1_Chassis_Gimbal_Fire(500, 500, 500);
//		CAN1_Chassis_SetMsg(500, 500, 500, 500);
		
//		Motor_2006_speed_test();  //ch1上下调速度
		Motor_2006_fire_test();   //直接控制速度
        vTaskDelay(2);
    }
}

/* 记得把拨弹电机的宏定义加上，电调：0x207 */
static void Motor_2006_speed_test(void)
{
	test_motor_2006 += (rc_ctrl.rc.ch[1]) * 0.02f;
	float_limit(test_motor_2006, 2000, -2000);
	
	Rmmotor_Speed_control(&Motor_2006_test_PID, test_motor_2006, motor_fire.speed, 2000);
	
	CAN1_Chassis_Gimbal_Fire(0, 0, Motor_2006_test_PID.out); 
}

/* 和底盘一样控制 */
static void Motor_2006_fire_test(void)
{
	fp32 outttt;
	
	outttt = Rmmotor_Speed_control(&Motor_2006_test_PID, ((rc_ctrl.rc.ch[1])*5), motor_fire.speed, 2000);
	CAN1_Chassis_Gimbal_Fire(0, 0, outttt);
}
#endif
