#include "led.h"

void Led_Init(void)
{
	GPIO_InitTypeDef gpioc;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	gpioc.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpioc.GPIO_Speed = GPIO_Speed_50MHz;	
	gpioc.GPIO_Mode = GPIO_Mode_OUT;										
	gpioc.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;  		
	GPIO_Init(GPIOC,&gpioc);	

	Green_Off;
	Red_Off;
	Blue_Off;
	Orange_Off;
}
