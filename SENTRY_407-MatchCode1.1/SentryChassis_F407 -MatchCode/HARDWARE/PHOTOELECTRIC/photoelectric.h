#ifndef __PHOTOELECTRIC_H
#define __PHOTOELECTRIC_H
#include "main.h"
#include "Task_Chassis.h"

extern void Photoelectric_GPIO_Init(void);
extern void Yaw_Zero_GPIO_Init(void);     //���У׼������
extern void Yaw_Two_Zero_GPIO_Init(void);

extern u8 Yaw_Zero_Value(void);
extern u8 Sec_Yaw_Zero_Value(void);   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1

extern void Correct_Yaw_Zero(chassis_control_t *fir_gimbal_correct);

#endif
