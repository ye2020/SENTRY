#ifndef __LED_H
#define __LED_H
#include "main.h"


//LED�˿ڶ���
#define LEDE0 PEout(0)  //ң��
#define LEDE1 PEout(1)  //����
#define LEDE2 PEout(2)	//��̨	
#define LEDE3 PEout(3)  //���	
#define LEDE4 PEout(4)	//����
#define LEDB7 PBout(7)  //����ָʾ��



void LED_Init(void);
void Laser_Init(void);
void Laser_ON(void);
void Laser_OFF(void);
void Double_GPIO_Init(void);
void Cap_OUT_IO_Init(void);

#endif
