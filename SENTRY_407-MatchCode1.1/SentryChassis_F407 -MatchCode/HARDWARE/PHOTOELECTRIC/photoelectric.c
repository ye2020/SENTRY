#include "photoelectric.h"
#include "CAN_1_Receive.h"


/*主云台yaw轴中值光电*/
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

/*副云台yaw轴中值光电*/
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
* 功能：检测主云台Y轴中值光电返回值
* 输入：无
* 输出：光电返回值1或0（到中值返回1，非中值返回0）
*/
u8 Yaw_Zero_Value(void)   //检测Y轴是否到位   到中值返回0，否则返回1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
}

/*
* 功能：检测副云台Y轴中值光电返回值
* 输入：无
* 输出：光电返回值1或0（到中值返回1，非中值返回0）
*/
u8 Sec_Yaw_Zero_Value(void)   //检测Y轴是否到位   到中值返回0，否则返回1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}


/*===============================Yaw轴电机码盘位置的光电校正===============================*/
static int16_t correctYawValue[5] = {0};
void Correct_Yaw_Zero(chassis_control_t *fir_gimbal_correct)
{
    u8 i = 0;
    u16 correctSum = 0;

    for (i = 0; i < 5; i++)
    {
        correctYawValue[i - 1] = correctYawValue[i];
    }
    correctYawValue[4] = !fir_gimbal_correct->gimbal_re_data->photoelectric_zero; //云台板通过can2返回的数值

    for (i = 0; i < 5; i++)
    {
        correctSum += correctYawValue[i];
    }

    correctSum /= 5;

    if (correctSum == 0)  //不为0的话，说明全为1，也就是说，5次数检测都没有达到光电。只要有一个到达光电，就清0
    {
		fir_gimbal_correct->yaw_motor_measure->actual_Position = 0;
    }
}

