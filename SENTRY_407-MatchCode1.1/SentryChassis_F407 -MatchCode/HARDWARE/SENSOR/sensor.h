#ifndef LASER_H
#define LASER_H
#include "main.h"

#define HAVE_THING 1		//距离内有东西0
#define NO_THING   0		//距离内没东西1

#define BLOCKING   2				//走位模式
#define	GOFORWARD	 1				//自动模式下的向前
#define	GOBACK		 0				//自动模式下的向后

void Laser_GPIO_Init(void);

u8 Get_Laser_Forward(void);
u8 Get_Laser_Back(void);

#endif
