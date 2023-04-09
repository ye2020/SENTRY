#include "pwm.h"

/**
  * @brief          Ħ���ֳ�ʼ��
  * @param[in]      null
  * @retval         null
  * @attention			pwm�ź����500Hz ����473.9 ��ֹ��ѧƯ��
  */
void Friction_Init(void)
{


    /* ��C615����ֲ᣺
     *       �����ݿ����ź�Ƶ��: 500Hz  ��Ŀǰ��473Hz ��ֹ����
     *       �����ź��г�: 400~2200΢�� ��0.4ms~2.2ms��
     *       Ĭ�����PWMƵ��: 16kHz ����SNAIL����ֲ������Ĭ������PWMƵ��16kHz��Ӧ��
     */
    PWM_Init(2111 - 1, 84 - 1); //Ħ���ֳ�ʼ��

    #if (REVERES_LIGHT_COUPLING == 1)
    TIM4->CCR1 = 2111 - 1000;
    TIM4->CCR2 = 2111 - 1000;
    #else
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    #endif



//		PWM_Init(2000, 84-1);
//		TIM_SetCompare1(TIM3, (2000 - 0));
//    TIM_SetCompare2(TIM3, (2000 - 0));
//    TIM_SetCompare3(TIM5, (2000 - 0));
//    TIM_SetCompare4(TIM5, (2000 - 0));
//		delay_ms(3000);
//		TIM_SetCompare1(TIM3, (2000- 1000));
//    TIM_SetCompare2(TIM3, (2000 - 1000));
//    TIM_SetCompare3(TIM5, (2000 - 1000));
//    TIM_SetCompare4(TIM5, (2000 - 1000));
//		delay_ms(3000);



}

/**
  * @brief          Ħ������������
  * @param[in]      compare1��compare2
  * @retval         null
  * @attention			��������Ħ����������ֹͣ snail���ֹͣ��ֹ��crr��Ϊ0,����ᱨ��
  */
void PWM_Set_All_Compare_Value(uint32_t compare1, uint32_t compare2)
{
    TIM_SetCompare1(TIM3, compare1);
    TIM_SetCompare2(TIM3, compare1);
    TIM_SetCompare3(TIM5, compare2);
    TIM_SetCompare4(TIM5, compare2);
}

/**
  * @brief          pwm�ײ�����
  * @param[in]      arr,psc
  * @retval         none
  * @attention			TIM3->PA6,PA7 TIM5->PA2,PA3
  */
void PWM_Init(u32 arr, u16 psc)
{
    GPIO_InitTypeDef  				GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
    TIM_OCInitTypeDef 				TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //TIM3
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5); //TIM5
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler 		= psc;
//		TIM_TimeBaseStructure.TIM_Period    		= TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period    		= arr;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM5, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}


