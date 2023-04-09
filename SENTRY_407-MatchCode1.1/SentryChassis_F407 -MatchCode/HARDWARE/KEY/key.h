#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

#define KEY PDin(10)

void KEY_Init(void);
u8 KEY_Scan(u8 mode);
#endif
