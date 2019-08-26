#include "MPU_Temperature.h"
#include "mpu6050.h"

void MPU_TempPID_Init_IO(void)
{
		GPIO_InitTypeDef gpio;	
		TIM_TimeBaseInitTypeDef   tim;
		TIM_OCInitTypeDef         oc;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_Mode = GPIO_Mode_AF;										
		gpio.GPIO_Pin = GPIO_Pin_8 ;  		

		GPIO_Init(GPIOA, &gpio);	
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8, GPIO_AF_TIM1); 
		

		tim.TIM_Prescaler = 84-1;
		tim.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
		tim.TIM_Period = 1000;   //2ms	计数周期
		tim.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割，不为1的话会乘2
		TIM_TimeBaseInit(TIM1,&tim);
		
		oc.TIM_OCMode = TIM_OCMode_PWM2;		//选择定时器模式
		oc.TIM_OutputState = TIM_OutputState_Enable;		//选择输出比较状态
		oc.TIM_OutputNState = TIM_OutputState_Disable;	//选择互补输出比较状态
		oc.TIM_Pulse = 0;		//设置待装入捕获比较器的脉冲值
		oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//设置输出极性
		oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//设置互补输出极性
		oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		//选择空闲状态下的非工作状态
		oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//选择互补空闲状态下的非工作状态
		TIM_OC1Init(TIM1,&oc);		//通道1
		TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
		
					 
		TIM_ARRPreloadConfig(TIM1,ENABLE);
		TIM_CtrlPWMOutputs(TIM1,ENABLE);
		
		TIM_Cmd(TIM1,ENABLE);
		
}


short Temperature_Value;
float TEMP_Error,TEMP_Last_Error,TEMP_Pterm,TEMP_Iterm,TEMP_Dterm,TEMP_PIDterm;
float TEMP_P=10/*12*/,TEMP_I=1/*1.5*/,TEMP_D=2;


void Tempeture_PID(void)
{
	Temperature_Value =MPU_Get_Temperature();//更新6050温度值
	
	TEMP_Error 	 = 	3800 - Temperature_Value;
	TEMP_Pterm 	 = 	TEMP_Error * TEMP_P;
	
	TEMP_Iterm	 += TEMP_Error * TEMP_I * 0.02f;
	TEMP_Iterm	 = 	constrain_float(TEMP_Iterm,-100,+100);
	
	TEMP_Dterm	 =	(TEMP_Error - TEMP_Last_Error) * TEMP_D/0.02f;
	TEMP_PIDterm =	TEMP_Pterm+ TEMP_Iterm + TEMP_Dterm;

	TEMP_PIDterm =	constrain_float(TEMP_PIDterm,0,+1000);  
	
	TEMP_Last_Error=	TEMP_Error;
	TIM1->CCR1  = TEMP_PIDterm ;


}
