#ifndef __CAPACITOR_H
#define __CAPACITOR_H
#include "main.h"

//超级电容相关参数结构体
typedef struct
{
	uint8_t SupeCap_init_flag;    //超级电容初始化成功标志位
    uint8_t Recharged_flag;       //充电完毕标志位
} Super_Cap_t;


void Capacitance_Control_Init(void);
void Capacitance_usart_iwdg(void);
void SuperCap_Send(u8 send_3);
void Capacitance_Power_Send(u8 send_3);
void SuperCap_OFF(void);

#endif
