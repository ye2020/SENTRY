#include "pid.h"
#include "maths.h"


/*
*���ܣ��Դ������ݽ���λ��ʽpid���㲢���ֵ
*���룺1.PID�ṹ������   2.��ǰֵ
*�������Ը��豸�Ŀ�����
*˵����Ŀ��ֵ�ڱջ����ƺ����︳ֵ
*/
int32_t Location_Pid_Int32(PidTypeDef *pid, float actualValue)
{
    int32_t dError;

    //error
    pid->error[0] = pid->SetValue - actualValue;

    //������
    pid->Ierror += pid->Ierror;
    //�����޷�
    pid->Ierror = int32_limit(pid->Ierror, 10000, -10000);

    //΢��
    dError = pid->error[0] - pid->error[1];
    //΢���� ��¼
    pid->Derror[2] = pid->Derror[1];
    pid->Derror[1] = pid->Derror[0];
    pid->Derror[0] = (pid->error[0] - pid->error[1]);

    //���pid����
    pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * dError);

    //PIDÿһ�����
    pid->Pout = (pid->Kp * pid->error[0]);
    pid->Iout = (pid->Ki * pid->Ierror);
    pid->Dout = (pid->Kd * dError);

    //��¼error
    pid->error[1] = pid->error[0];

    return pid->out;
}



fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)//������ (��û��ģʽ������)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->SetValue = set;
    pid->ActualValue = ref;
    pid->error[0] = set - ref;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Derror[2] = pid->Derror[1];
    pid->Derror[1] = pid->Derror[0];
    pid->Derror[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Derror[0];
    pid->Iout = float_limit(pid->Iout, pid->max_iout, -pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
//	pid->out = float_limit(pid->out, pid->max_out, -pid->max_out);

    return pid->out;
}



//����pid��ʼ��
void pid_init(PidTypeDef *pid, fp32 kp, fp32 ki, fp32 kd, fp32 i_max, fp32 out_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->SetValue = 0;
    pid->ActualValue = 0;

    pid->out = 0;
    pid->lastout = 0;

    pid->Ierror = 0;

//	pid->max_iout = i_max;
//	pid->max_out = out_max;

    pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
    pid->error[0] = pid->error[0] = pid->error[0] = 0.0f;
    pid->Dout = pid->Iout = pid->Pout = pid->out = 0.0f;
}



void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->ActualValue = pid->SetValue = 0.0f;
}























