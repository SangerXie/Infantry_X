#include "loop.h"




//主循环
void Loop(void)
{	
	static uint32_t currentTime = 0;
	
	static uint32_t ulLoopTime_STOPMOTOR = 0;//失控保护
	
	static uint32_t loopTime_1ms = 0;
	static uint32_t loopTime_2ms = 0;
	static uint32_t loopTime_4ms = 0;
	static uint32_t loopTime_10ms = 0;
	static uint32_t loopTime_50ms = 0;
	static uint32_t loopTime_100ms = 0;

	
	currentTime = micros();	//获取当前系统时间
	
	//失控保护,通过获取系统时间与串口2中断中获取的系统时间做比较
	if (currentTime >= REMOTE_ulGetLostTime( ))//遥控失联,不失联的话应该一直处于小于的状态(时间差为REMOTE_LOST_TIME)
	{
		if (currentTime >= ulLoopTime_STOPMOTOR)
		{
			ulLoopTime_STOPMOTOR = currentTime + 4000;//4ms
			
			SYSTEM_OutCtrlProtect( );//失控保护
		}
		//退出循环
		return;
	}
	else if (REMOTE_IfDataError() == TRUE      //遥控接收数据出错
		       || REMOTE_IfKeyReset() == TRUE )     //手动重启
	{	
		//重启芯片
		delay_ms(1);
		__set_FAULTMASK(1);
		NVIC_SystemReset();			
	}

	
	
	/**********************正常循环******************************/
	
	if((int32_t)(currentTime - loopTime_1ms) >= 0)  
	{			
		loopTime_1ms = currentTime + 1000;	//1ms
		
		GIMBAL_UpdatePalstance();//更新云台数值
		Vision_UpdatePalstance();//更新视觉数值
		SYSTEM_UpdateRemoteMode();//更新遥控/键盘模式
		SYSTEM_UpdateSystemState();//更新系统状态,启动或运行状态
		
		REVOLVER_Ctrl();//拨盘电机控制,严格放此执行,否则卡弹时间判断不准
	}
	
	if((int32_t)(currentTime - loopTime_2ms) >= 0)  
	{			
		loopTime_2ms = currentTime + 2000;	//2ms

		CHASSIS_Ctrl();		//底盘控制
		GIMBAL_Ctrl();		//云台控制
	}
	
	if((int32_t)(currentTime - loopTime_4ms) >= 0)  
	{			
		loopTime_4ms = currentTime + 4000;	//4ms
		
		Judge_Read_Data(Judge_Buffer);		//读取裁判系统数据	
	}
	
	if((int32_t)(currentTime - loopTime_10ms) >= 0)  
	{			
		loopTime_10ms = currentTime + 10000;	//10ms

//		Magazine_Ctrl();		//弹仓控制
		FRICTION_Ctrl();//摩擦轮控制
	}
	
	if((int32_t)(currentTime - loopTime_50ms) >= 0) 
	{
		loopTime_50ms = currentTime + 50000;	 //50ms	
		
//		Tempeture_PID();//解决陀螺仪温度漂移问题
	}	
	
	if((int32_t)(currentTime - loopTime_100ms) >= 0)  
	{			
		loopTime_100ms = currentTime + 100000;	//100ms
//		JUDGE_Show_Data();//用户数据上传,10Hz官方限制速度
		Vision_Ctrl();//视觉,指令更新
	}

}
