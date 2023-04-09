#ifndef __RNG_H
#define __RNG_H
#include "main.h"

uint8_t RNG_Init(void);
uint32_t RNG_Get_RandomNum(void);
int RNG_Get_RandomRange(int min,int max);


#endif
