#include "Info_Update.h"

#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "control.h"
#include "Info_Transmitter.h"
//#include "super_cap.h"
#include "remote.h"
#include "led.h"
#include "iwdg.h"


#define APP_INFO_TASK_PRIO  1  				//任务优先级
TaskHandle_t AppInfoTask_Handler; 			//任务句柄


#define APP_INFO_TRANSMIT_PRIO  6  			//任务优先级
TaskHandle_t AppInfoTransmit_Handler; 		//任务句柄

#define APP_INFO_JUDGEREAD_PRIO  3  		//任务优先级
TaskHandle_t AppInfoJudge_Handler; 			//任务句柄

/**
  * @brief  创建状态更新任务函数
  * @param  void
  * @retval void
  * @attention 更新系统各类状态
  */
void App_Info_Create(void)
{
	taskENTER_CRITICAL(); 	//进入临界区
	
	/* 状态数值获取 */
	xTaskCreate((TaskFunction_t )Info_Update_Task, 		//任务函数
				(const char* )"Info_Update_Task", 		//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )APP_INFO_TASK_PRIO, 		//任务优先级
				(TaskHandle_t* )&AppInfoTask_Handler); 	//任务句柄
	
	/* 视觉、裁判发送 */
	xTaskCreate((TaskFunction_t )Info_JudgeRead_Task,	 //任务函数
				(const char* )"Info_Update_Task", 		//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )APP_INFO_TRANSMIT_PRIO, 	//任务优先级
				(TaskHandle_t* )&AppInfoTransmit_Handler); 	//任务句柄

	/* 裁判读取 */
	xTaskCreate((TaskFunction_t )Info_Transmitter_Task, //任务函数
				(const char* )"Info_Update_Task", 		//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )APP_INFO_JUDGEREAD_PRIO, 	//任务优先级
				(TaskHandle_t* )&AppInfoJudge_Handler); 	//任务句柄
				
	taskEXIT_CRITICAL(); 								//退出临界区
}

/**
  * @brief  状态更新任务
  * @param  void
  * @retval void
  * @attention 更新系统各类状态,1ms相对延时
  */
void Info_Update_Task(void *pvParameters)
{
	static portTickType currentTime;
	
	for(;;)
	{
//		vTaskDelay(TIME_STAMP_1MS);				//1ms
		
		currentTime = xTaskGetTickCount();	//获取当前系统时间	
		
		GIMBAL_UpdatePalstance();//更新云台数值
		SYSTEM_UpdateRemoteMode();//更新遥控/键盘模式
		/* 数据出错处理，防暴走 */
		if( REMOTE_IfDataError() == TRUE 
				|| REMOTE_IfKeyReset() == TRUE)//实验
		{
			SYSTEM_OutCtrlProtect();
		}
		else
		{
			IWDG_Feed();//喂狗
		}
		if(currentTime >= REMOTE_ulGetLostTime( ))//遥控失联时间太长
		{
//			SYSTEM_Reset();//系统恢复至重启状态
		}
		else
		{
			SYSTEM_UpdateSystemState();//更新系统状态,启动或运行状态
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
	}
}

