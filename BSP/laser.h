#ifndef _LASER_H
#define _LASER_H


#include "system.h"

#define Laser_On  	GPIO_SetBits(GPIOD,GPIO_Pin_9)
#define Laser_Off 	GPIO_ResetBits(GPIOD,GPIO_Pin_9)

void LASER_Init(void);


#endif
