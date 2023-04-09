#include "sensor.h"

u8 inputValue;
int16_t watch1;
int16_t watch2;
extern int16_t ms_count;
//���⴫������IO�ڳ�ʼ��
//PA6 PA7
void Laser_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				//����
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          //PA6
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;           //PA7
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


//��ȡ���⴫������ֵ
//�������ж���Ϊ0, û����Ϊ1
//����������
u8 Get_Laser_Back()
{	
	static int8_t b;
	
	{
		b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
	}

	return b;
	
}
u8 Get_Laser_Forward()
{
	static int8_t a;

	{
		a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		
	}
  return a;
	
}

