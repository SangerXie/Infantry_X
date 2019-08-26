#ifndef __CAN2_H__
#define __CAN2_H__

#include "system.h"
//#include "stm32f4xx.h"


void CAN2_Init(void);
void CAN2_Revolver_QueueSend(float output);

#endif 
