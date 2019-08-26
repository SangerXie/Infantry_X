#ifndef _USART5_H
#define _USART5_H

#include "system.h"

//#define    COM5_BUFFER_NUM           5
#define    JUDGE_BUFFER_LEN           200

extern uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ];

void UART5_Init( void );
void UART5_DMA_Init( void );

//uint8_t *pucCom5ReadBuffer( void );
void UART5_SendChar( uint8_t cData );

//void UART5_UpdateMemoryAddr( DMA_InitTypeDef *xDMAInit, uint8_t *buffer );


#endif
