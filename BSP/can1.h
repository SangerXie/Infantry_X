#ifndef _CAN2_H
#define _CAN2_H

#include "system.h"	


void CAN1_Init(void);
void CAN1_Chassis_QueueSend(float*);
void CAN1_Chassis_Send(float*);
void CAN1_Cloud_QueueSend(float*);
void CAN1_Cloud_Send(float*);

#endif
