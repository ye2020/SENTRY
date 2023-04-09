#ifndef __PWM_H
#define __PWM_H
#include "main.h"

#define REVERES_LIGHT_COUPLING  0 					// 是否使用反向光偶   1 -> 使用 2 -> 未使用

extern void Friction_Init(void);
extern void PWM_Set_All_Compare_Value(uint32_t compare1, uint32_t compare2);
extern void PWM_Init(u32 arr, u16 psc);

#define Gimbal_Pitch_Output  TIM3->CCR3  //PB0

#endif
