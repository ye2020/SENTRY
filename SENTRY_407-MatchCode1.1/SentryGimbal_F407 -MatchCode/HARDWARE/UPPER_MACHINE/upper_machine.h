#ifndef __UPPER_MACHINE_H__
#define __UPPER_MACHINE_H__
#include "pid.h"
#include "main.h"
void upper_machine_communication(void);
void upper_machine_usart2_callback(uint8_t *usart2_data, uint16_t Len);
void pid_parameter_receive(PidTypeDef *pid_speed, PidTypeDef *pid_position);
void vofa_test(void);
#endif















