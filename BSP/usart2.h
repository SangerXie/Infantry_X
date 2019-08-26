#ifndef __USART2_H
#define __USART2_H

#include "system.h"
#include "stm32f4xx.h"




void USART2_Init( void );
void vCom2RxTxTest( void );
//volatile uint8_t *pucCom2ReadBuffer( void );


#endif
