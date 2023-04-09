#include "rng.h"

//��ʼ��RNG
uint8_t RNG_Init(void)
{
	uint16_t retry=0; 
	
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);//����RNGʱ��,����PLL48CLK
	
	RNG_Cmd(ENABLE);	//ʹ��RNG
	
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET&&retry<10000)	//�ȴ����������
	{
		retry++;
		delay_us(100);
	}
	if(retry>=10000)return 1;//���������������������
	return 0;
}
//�õ������
//����ֵ:��ȡ���������
uint32_t RNG_Get_RandomNum(void)
{	 
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET);	//�ȴ����������  
	return RNG_GetRandomNumber();	
}

//����[min,max]��Χ�������
int RNG_Get_RandomRange(int min,int max)
{ 
   return RNG_Get_RandomNum()%(max-min+1) +min;
}
















