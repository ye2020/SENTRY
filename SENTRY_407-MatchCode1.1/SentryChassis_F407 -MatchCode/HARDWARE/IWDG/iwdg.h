#ifndef __WDG_H
#define __WDG_H
#include "main.h"

void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);
 
#endif
