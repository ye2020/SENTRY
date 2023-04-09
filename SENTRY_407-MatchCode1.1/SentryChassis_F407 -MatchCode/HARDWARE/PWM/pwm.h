#ifndef __PWM_H
#define __PWM_H
#include "main.h"


extern void Friction_Wheel_Init(void);
extern void Steering_Gear_Init(void);

extern void TIM12_PWM_Init(u32 arr,u32 psc);
extern void TIM5_PWM_Init(u32 arr,u32 psc);   
extern void TIM3_PWM_Init(u32 arr,u32 psc); 
extern void TIM3_PWM_6020_Init(u32 arr, u32 psc);
extern void TIM4_PWM_Init(u32 arr,u32 psc);

#define Gimbal_Pitch_Output  TIM3->CCR3  //PB0

#endif
