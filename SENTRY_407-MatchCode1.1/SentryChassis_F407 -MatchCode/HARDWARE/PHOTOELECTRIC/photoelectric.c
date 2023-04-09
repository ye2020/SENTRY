#include "photoelectric.h"
#include "CAN_1_Receive.h"


/*����̨yaw����ֵ���*/
void Yaw_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*����̨yaw����ֵ���*/
void Yaw_Two_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}



/*
* ���ܣ��������̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
u8 Yaw_Zero_Value(void)   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
}

/*
* ���ܣ���⸱��̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
u8 Sec_Yaw_Zero_Value(void)   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}


/*===============================Yaw��������λ�õĹ��У��===============================*/
static int16_t correctYawValue[5] = {0};
void Correct_Yaw_Zero(chassis_control_t *fir_gimbal_correct)
{
    u8 i = 0;
    u16 correctSum = 0;

    for (i = 0; i < 5; i++)
    {
        correctYawValue[i - 1] = correctYawValue[i];
    }
    correctYawValue[4] = !fir_gimbal_correct->gimbal_re_data->photoelectric_zero; //��̨��ͨ��can2���ص���ֵ

    for (i = 0; i < 5; i++)
    {
        correctSum += correctYawValue[i];
    }

    correctSum /= 5;

    if (correctSum == 0)  //��Ϊ0�Ļ���˵��ȫΪ1��Ҳ����˵��5������ⶼû�дﵽ��硣ֻҪ��һ�������磬����0
    {
		fir_gimbal_correct->yaw_motor_measure->actual_Position = 0;
    }
}

