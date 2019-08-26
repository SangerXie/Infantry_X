#ifndef _ADDA_H
#define _ADDA_H

#include "system.h"

void SuperCap_DAC_Init(void);
void SuperCap_ADC_Init(void);
void SuperCap_IO_Init(void);
void Dac1_Set_Vol(u16 vol);
u16 Get_Adc(void);
u16 Get_Adc_Average(u8 times);
float Get_Realvoltage(void);
int Capvoltage_Percent(void);

#define CAP_Charge_On		   GPIO_SetBits(GPIOA,GPIO_Pin_2)
#define CAP_Charge_Off	   GPIO_ResetBits(GPIOA,GPIO_Pin_2)

#define CAP_OUT_On		 GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define CAP_OUT_Off	   GPIO_ResetBits(GPIOA,GPIO_Pin_5)


#endif
