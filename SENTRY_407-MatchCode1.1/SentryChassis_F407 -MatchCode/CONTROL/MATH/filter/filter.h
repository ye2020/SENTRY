#ifndef __FILTER_H
#define __FILTER_H
#include "sys.h"
#include "stm32f4xx.h" 

extern float angle, angle_dot;

void Kalman_Filter(float Accel, float Gyro);
void Improve_Complementary_Filter(float angle_m, float gyro_m);
	
#endif
