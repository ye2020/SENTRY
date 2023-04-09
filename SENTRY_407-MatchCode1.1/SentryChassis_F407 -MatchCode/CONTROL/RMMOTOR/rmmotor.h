#ifndef _RMMOTOR_H
#define _RMMOTOR_H
#include "main.h"
#include "pid.h"
#include "maths.h"
#include "math.h"
#include "CAN_1_Receive.h"


/*=====��������========*/
void Motor_Actual_Position(motor_measure_t *rmMotor, int16_t gear_Ratio,int16_t lap_encoder); //������ʵ����ֵ
int16_t Angle_Limiting_Int16(int16_t Angl_Err, int16_t lap_encoder);                           //�ٽǴ���16λ
int32_t Angle_Limiting_Int32(int32_t Angl_Error,int16_t buff,int16_t lap_encoder);            //�ٽǴ���32λ�����ٱȼ���
int32_t Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);               //���ٽ�ֵ��λ����ֵ
int16_t Check_Motor_Block(int16_t position);                                                   //�������ת
float Get_Yaw_Different_Angle(const motor_measure_t *yaw_position , int16_t Ratio);                  //��ȡ��̨����̲��
int16_t Encoder_Real(int32_t read_code);                                                       //��Ȧ����ֵ����������ת��
int16_t Yaw_Actual_Code_Conversion(int16_t actual_code , int16_t max_code , int16_t middle_code);//Yaw����ʵλ�ã���Ȧ���������������Ҹ���

int16_t Rmmotor_Speed_control(PidTypeDef *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit);  //����ٶȱջ�
int16_t Motor_Position_Speed_Control(PidTypeDef *speed_pid, PidTypeDef *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit);


#endif
