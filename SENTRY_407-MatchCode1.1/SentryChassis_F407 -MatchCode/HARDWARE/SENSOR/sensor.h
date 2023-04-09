#ifndef LASER_H
#define LASER_H
#include "main.h"

#define HAVE_THING 1		//�������ж���0
#define NO_THING   0		//������û����1

#define BLOCKING   2				//��λģʽ
#define	GOFORWARD	 1				//�Զ�ģʽ�µ���ǰ
#define	GOBACK		 0				//�Զ�ģʽ�µ����

void Laser_GPIO_Init(void);

u8 Get_Laser_Forward(void);
u8 Get_Laser_Back(void);

#endif
