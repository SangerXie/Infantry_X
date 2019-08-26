#include "Info_Transmitter.h"

#include "control.h"
#include "judge.h"
#include "usart5.h"
#include "vision.h"

void Info_Transmitter_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_100MS);				//100ms
		
		JUDGE_Show_Data();//用户数据上传,10Hz官方限制速度
		Vision_Ctrl();//视觉,指令更新
	}
}

void Info_JudgeRead_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_2MS);			//2ms
		
		Judge_Read_Data(Judge_Buffer);		//读取裁判系统数据	
	}
}
