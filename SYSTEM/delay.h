#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  


void Delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u32 nms);
void delay_xms(u32 nms);

#endif





























