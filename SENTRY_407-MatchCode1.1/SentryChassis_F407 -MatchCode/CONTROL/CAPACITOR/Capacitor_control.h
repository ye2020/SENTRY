#ifndef __CAPACITOR_H
#define __CAPACITOR_H
#include "main.h"

//����������ز����ṹ��
typedef struct
{
	uint8_t SupeCap_init_flag;    //�������ݳ�ʼ���ɹ���־λ
    uint8_t Recharged_flag;       //�����ϱ�־λ
} Super_Cap_t;


void Capacitance_Control_Init(void);
void Capacitance_usart_iwdg(void);
void SuperCap_Send(u8 send_3);
void Capacitance_Power_Send(u8 send_3);
void SuperCap_OFF(void);

#endif
