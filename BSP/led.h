#ifndef _LED_H
#define _LED_H

#include "system.h"


#define Green_On  	     GPIO_ResetBits(GPIOC,GPIO_Pin_10)
#define Green_Off  	     GPIO_SetBits(GPIOC,GPIO_Pin_10)

#define Red_On	    	 GPIO_ResetBits(GPIOC,GPIO_Pin_11)
#define Red_Off	    	 GPIO_SetBits(GPIOC,GPIO_Pin_11)

#define Blue_On  	     GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define Blue_Off  	     GPIO_SetBits(GPIOC,GPIO_Pin_13)

#define Orange_On	     GPIO_ResetBits(GPIOC,GPIO_Pin_14)
#define Orange_Off	     GPIO_SetBits(GPIOC,GPIO_Pin_14)



void Led_Init(void);

#endif
