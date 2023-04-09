#include "adc.h"

#define N 20
double sum[20] = {0.0}, sum0 =0.0; 
int num_i = 0, num_j = 0;


//��ʼ��ADC	PB0
void Adc_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��
	
	//�ȳ�ʼ��ADC1ͨ��8 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0 ͨ��8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // ����ģʽ
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
	ADC_Cmd(ADC1,ENABLE);//����ADת����
	
	
}

//��ȡADC��ֵ 
//ch:ͨ��
//����ֵ��ת�����

u16 Get_Adc(u8 ch)
{
	//����һ��ͨ����ת����1�Ϳ�����
	ADC_RegularChannelConfig(ADC1, ch, 1,ADC_SampleTime_480Cycles);//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
	
	ADC_SoftwareStartConv(ADC1);
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת��������EPC�ǽ�����־

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

//�Զ����ֵ�˲�
//��ȡch��ת��ֵ��ȡtimes��
//ch:ͨ�����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
//��ȡ��ѹ��ADC
float Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		vTaskDelay(4);
	}
	return (float)temp_val/(float)times;
} 
//20�λ����˲�
double _sum(double a)
{
	sum[num_i] = a;
	num_i ++;
	if(num_i == N) num_i = 0;
	for(num_j = 0; num_j <N; num_j ++)
	{
		sum0 += sum[num_j];
	}
	a = sum0/(double)N;
	sum0 = 0.0;
	return a;
}
