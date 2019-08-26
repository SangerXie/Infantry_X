#ifndef __IIC_H
#define	__IIC_H

#include "system.h"

/***************I2C GPIO定义******************/
#define GPIO_I2C			GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define RCC_I2C				RCC_AHB1Periph_GPIOB
/*********************************************/
/***************I2C SDA/SCL定义******************/
#define SCL_H         GPIO_I2C->BSRRL = I2C_Pin_SCL		//ANO_GPIO_I2C->BSRRL
#define SCL_L         GPIO_I2C->BSRRH  = I2C_Pin_SCL
   
#define SDA_H         GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         GPIO_I2C->BSRRH  = I2C_Pin_SDA

#define SCL_read      GPIO_I2C->IDR  & I2C_Pin_SCL	//判断是否是高电平
#define SDA_read      GPIO_I2C->IDR  & I2C_Pin_SDA

extern u8 I2C_FastMode;

void I2C_Soft_Init(void);
bool I2C_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
int I2C_Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
bool I2C_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);
	
#endif
