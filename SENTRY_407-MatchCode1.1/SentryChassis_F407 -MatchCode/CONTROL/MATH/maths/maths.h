#ifndef __MATHS_H
#define __MATHS_H

#include "sys.h"
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>



//�˶����ٶ�����б�º���
typedef __packed struct
{
    float Input;      //��ǰȡ��ֵ
    float Last_Input; //�ϴ�ȡ��ֵ
    float Output;     //���ֵ
    float acc_now;    //��ǰ���ٶ�
    float acc_limit;  //��Ҫ���Ƶļ��ٶ�
} acceleration_control_type_t;

//������ֵ�˲����������㣩
typedef __packed struct
{
    float Input;     //��ǰȡ��ֵ
    int count_num;   //ȡ������
    float Output;    //�˲����
    float Sum;       //�ۼ��ܺ�
    float FIFO[250]; //����
    int sum_flag;    //�Ѿ���250����־
} sliding_mean_filter_type_t;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
	fp32 input;        //��������
	fp32 last_input;   //�ϴ�����
	fp32 out;          //�˲����������
	fp32 num;          //�˲�����
}first_order_filter_type_t;



int32_t int32_limit(int32_t x, int32_t max, int32_t min);
int16_t int16_limit(int16_t x,int16_t max,int16_t min);
float float_limit(float x,float max,float min);
signed long limit_long(signed long x,signed long max,signed long min);
int16_t int16_t_abs(int16_t x);
signed long long_abs(signed long x);
float float_abs(float x);			// ����ֵ����
int16_t max_abs(int16_t x, int16_t y); //ȡ���ֵ�ľ���ֵ
float invSqrt(float x);       //ƽ��������

/* �˶�����б�º��������ٶ����ƣ���16λ�� */
int16_t Motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit); //�˶����ٶ�����

/* ��ͨ�˲� */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* ƽ���˲� */
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input, int num);  //��ֵ�����˲�
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter);  //��ֵ�����˲���ʼ�����ɲ��ã�ֱ�Ӷ���ṹ��ʱ����ֵ��

/* ѭ������ */
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num);          //16λѭ���޷�
float Loop_Restriction_Float(float num, float limit_num);                //����ѭ���޷�
float loop_fp32_constrain(float Input, float minValue, float maxValue);  //ѭ�����ƣ���̨�Ƕȴ���
float cos_calculate(float angle);
float sin_calculate(float angle);


/* б�º��� */
void Data_Accelerated_Control(float *input, float acc);  //���ٶ�����б�º���

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif

