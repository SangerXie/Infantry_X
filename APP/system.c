#include "system.h"


#include "remote.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"

#include "can1.h"
#include "usart2.h"
#include "pwm.h"
#include "laser.h"
#include "usart5.h"
#include "can2.h"
#include "usart4.h"
#include "led.h"
#include "adda.h"

#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "delay.h"
#include "MPU_Temperature.h"
#include "iwdg.h"


//陀螺仪自检用
bool pass_flag=1;

//限幅
int constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

float constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						now     += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}


/**********************************************/
//参数初始化
void Parameter_Init(void)
{	
	CHASSIS_InitArgument();//底盘参数初始化
	GIMBAL_InitArgument();//云台参数初始化
}

//外设初始化
void SYSTEM_InitPeripheral(void)
{
	static portTickType ulCurrentTime = 0;
	static portTickType ulLoopTime    = 0;
	static int16_t  	sTimeCnt      = 0;
	
	Delay_init(168);//配置时钟
	CAN1_Init();//底盘和云台电机初始化
	USART2_Init();//接收机初始化
	UART5_Init();//裁判系统初始化
	TIM4_Init();//摩擦轮初始化
	TIM1_Init();//弹仓舵机初始化
	LASER_Init();//红外瞄准初始化
	CAN2_Init();//拨盘电机初始化
	UART4_Init();//视觉串口初始化
//	MPU_TempPID_Init_IO();//MPU温度漂移初始化,会造成陀螺仪角度读取出现单位脉冲，影响云台稳定性
	Led_Init();//指示灯初始化
	SuperCap_ADC_Init();//电容电量读取初始化
	SuperCap_DAC_Init();//电容充电控制初始化
	SuperCap_IO_Init();//电容IO口初始化
	IWDG_Init(4,30000);
	
	//祖传MPU初始化
	MPU_Init();
	while (mpu_dmp_init( )) 
	{
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS延时 */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms屏蔽自检 */
					if (sTimeCnt >= 2) 
					{
							pass_flag = 0;
							sTimeCnt  = 0;//10;
					}
					else
					{
							sTimeCnt++;
					}
			}
	}
}
/*********************************************/


//系统初始化
void System_Init(void)
{
	Parameter_Init();//参数初始化	
	REMOTE_vResetData();//遥控初始化
	SYSTEM_InitPeripheral();//外设初始化
}
