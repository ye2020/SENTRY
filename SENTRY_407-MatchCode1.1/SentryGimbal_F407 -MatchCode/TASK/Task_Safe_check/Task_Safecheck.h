#ifndef __SAFECHECK_TASK
#define __SAFECHECK_TASK
#include "main.h"

#define CHECK_CONTROL_TIME_MS 20

typedef struct
{	
	//remote
	int8_t  RC_Connected;
	int8_t  RC_Receive_Flag;
	int16_t RC_Error;
	
	//can1
	//can2
	
} DogTypeDef;

/*�ⲿʹ������*/
extern DogTypeDef Safecheck_dog;


/*ȫ�ֺ�������*/
void SAFECHECK_Task(void *pvParameters);   

#endif
