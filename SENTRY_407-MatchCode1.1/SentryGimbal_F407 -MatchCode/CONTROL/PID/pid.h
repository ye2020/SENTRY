#ifndef _PID_H
#define _PID_H
#include "main.h"


typedef struct //pid�ṹ�����
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 SetValue;
    fp32 ActualValue;

		fp32 stepIn;

	    /* ����� */
    fp32 errorabsmax; //ƫ�����ֵ���ֵ
    fp32 errorabsmin; //ƫ�����ֵ��Сֵ

	
		    /* �����ֱ��� */
    fp32 maximum; //���ֵ
    fp32 minimum; //��Сֵ
	
    fp32 out;
	fp32 lastout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
	fp32 Ierror;
    fp32 Derror[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3];   //����� 0���� 1��һ�� 2���ϴ�
} PidTypeDef;


extern int32_t Location_Pid_Int32(PidTypeDef *pid , float actualValue);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void pid_init(PidTypeDef *pid,fp32 kp,fp32 ki,fp32 kd,fp32 i_max,fp32 out_max);
extern void PID_clear(PidTypeDef *pid);
float step_in_processing(PidTypeDef *vPID, float sp);
int32_t pid_regulator(PidTypeDef *vPID , float actualValue);


#endif















