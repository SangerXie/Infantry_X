#ifndef _USART4_H
#define _USART4_H

#include "system.h"



/* 接收缓存 */
#define    VISION_BUFFER_LEN           100   //稍微给大点

extern uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ];


void UART4_Init(void);
void UART4_DMA_Init(void);

void UART4_SendChar(uint8_t cData);

#endif
