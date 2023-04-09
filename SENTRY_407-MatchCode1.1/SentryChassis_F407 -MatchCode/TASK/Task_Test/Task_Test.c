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
	

/* ���ڲ��Ը��ֵ�� */
void TEST_Task(void *pvParameters)
{
	//���Ե��pid
	pid_init(&Motor_2006_test_PID, 8.0f, 0.0f, 0.2f, 0, 0);
	
    while (1)
    {
		//���ң������ֵ�Ƿ���ȷ
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
		
//		Motor_2006_speed_test();  //ch1���µ��ٶ�
		Motor_2006_fire_test();   //ֱ�ӿ����ٶ�
        vTaskDelay(2);
    }
}

/* �ǵðѲ�������ĺ궨����ϣ������0x207 */
static void Motor_2006_speed_test(void)
{
	test_motor_2006 += (rc_ctrl.rc.ch[1]) * 0.02f;
	float_limit(test_motor_2006, 2000, -2000);
	
	Rmmotor_Speed_control(&Motor_2006_test_PID, test_motor_2006, motor_fire.speed, 2000);
	
	CAN1_Chassis_Gimbal_Fire(0, 0, Motor_2006_test_PID.out); 
}

/* �͵���һ������ */
static void Motor_2006_fire_test(void)
{
	fp32 outttt;
	
	outttt = Rmmotor_Speed_control(&Motor_2006_test_PID, ((rc_ctrl.rc.ch[1])*5), motor_fire.speed, 2000);
	CAN1_Chassis_Gimbal_Fire(0, 0, outttt);
}
#endif
