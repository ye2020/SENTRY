#ifndef __TEST_TASK
#define __TEST_TASK
#include "main.h"
#include "pid.h"

#ifdef TEST_MODEextern 

PidTypeDef Motor_2006_test_PID;


/*全局函数声明*/
void TEST_Task(void *pvParameters);
#endif

#endif
