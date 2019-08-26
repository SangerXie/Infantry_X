#include "Task_Start.h"

#include "main.h"
#include "Timer_Send_Task.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Revolver.h"

#include "remote.h"
#include "control.h"
#include "magazine.h"
#include "friction.h"
#include "MPU_Temperature.h"
#include "super_cap.h"
#include "judge.h"
#include "usart5.h"
#include "led.h"


//任务创建
#define START_TASK_PRIO  11  				//任务优先级
TaskHandle_t StartTask_Handler; 			//任务句柄
void Start_Task(void *pvParameters); 		//任务函数

/*失控保护*/
#define TASK_Control_Protect_PRIO 2  		//任务优先级,失控任务函数
TaskHandle_t  Task_Control_Protect_Handler; //任务句柄
void Task_Control_Protect(void *pvParameters);

/*底盘*/
#define TASK_Chassis_PRIO 3  				//任务优先级
TaskHandle_t  Task_Chassis_Handler; 		//任务句柄

/*云台*/
#define TASK_Gimbal_PRIO 4 					//任务优先级
TaskHandle_t  Task_Gimbal_Handler; 			//任务句柄

/*拨盘*/
#define TASK_REVOLVER_PRIO 5  				//任务优先级
TaskHandle_t  Task_Revolver_Handler; 		//任务句柄

#define TASK_10ms_PRIO 6  					//任务优先级
TaskHandle_t  Task_10ms_Handler; 			//任务句柄
void Task_10ms(void *pvParameters); 		//任务函数

#define TASK_500ms_PRIO 10  					//任务优先级
TaskHandle_t  Task_500ms_Handler; 			//任务句柄
void Task_500ms(void *pvParameters); 		//任务函数

#define TASK_OUTCTRL 7  					//任务优先级
TaskHandle_t  Task_OutCtl_Handler; 			//任务句柄
void Task_OutCtl(void *pvParameters); 		//任务函数

/***********************************************************/
void App_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )Start_Task, 			//任务函数
				(const char* )"Start_Task", 			//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )START_TASK_PRIO, 			//任务优先级
				(TaskHandle_t* )&StartTask_Handler); 	//任务句柄

}


/**
  * @brief  创建开始任务任务函数
  * @param  void
  * @retval void
  * @attention 控制任务在此创建
  */
void Start_Task(void *pvParameters)
{
		
	taskENTER_CRITICAL(); 	//进入临界区
								 
	/*------------------------------------------------*/
	//创建 底盘 任务，实时任务，绝对延时
	xTaskCreate((TaskFunction_t )Task_Chassis,
				(const char* )"Task_Chassis",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_Chassis_PRIO, 
				(TaskHandle_t* )&Task_Chassis_Handler);
				
	//创建 云台 任务，实时任务，绝对延时
	xTaskCreate((TaskFunction_t )Task_Gimbal,
				(const char* )"Task_Gimbal",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_Gimbal_PRIO,
				(TaskHandle_t* )&Task_Gimbal_Handler);
				
	//创建 拨盘 任务，实时任务，绝对延时
	xTaskCreate((TaskFunction_t )Task_Revolver,
				(const char* )"Task_Revolver",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_REVOLVER_PRIO,
				(TaskHandle_t* )&Task_Revolver_Handler);
				
	//创建 10ms 任务
	xTaskCreate((TaskFunction_t )Task_10ms,
				(const char* )"Task_10ms",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_10ms_PRIO,
				(TaskHandle_t* )&Task_10ms_Handler);
				
	//创建 500ms 任务
	xTaskCreate((TaskFunction_t )Task_500ms,
				(const char* )"Task_500ms",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_500ms_PRIO,
				(TaskHandle_t* )&Task_500ms_Handler);
				
	//创建失控保护任务
	xTaskCreate((TaskFunction_t )Task_Control_Protect,
				(const char* )"Task_Control_Protect",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_Control_Protect_PRIO,
				(TaskHandle_t* )&Task_Control_Protect_Handler);
				
	//创建 失控控制 任务
	xTaskCreate((TaskFunction_t )Task_OutCtl,
				(const char* )"Task_OutCtl",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_OUTCTRL,
				(TaskHandle_t* )&Task_OutCtl_Handler);

				
	vTaskDelay(500);
	vTaskSuspend(StartTask_Handler); 					//删除开始任务			

	taskEXIT_CRITICAL(); 								//退出临界区		
}

//每10ms执行一次任务函数
void Task_10ms(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_10MS);				//10ms
//代码部分		
		Magazine_Ctrl();		//弹仓控制
		FRICTION_Ctrl();//摩擦轮控制	
		
		Super_Charging_Control();
		SuperCap_Giveout_Control();		
	}
}

//每500ms执行一次任务函数
void Task_500ms(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_200MS);				//500ms
//代码部分		
		Send_to_Teammate();
	}
}

//失控控制任务若解挂则每4ms执行一次，相对延迟
void Task_OutCtl(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_4MS);				//4ms
//代码部分		
		SYSTEM_OutCtrlProtect( );//失控保护
	}
}

//每50ms执行一次任务函数
void Task_Control_Protect(void *pvParameters)
{
	static portTickType currentTime;	
	
	for(;;)
	{	
		//代码部分	
		currentTime = xTaskGetTickCount();	//获取当前系统时间	
		
//		Tempeture_PID();//解决陀螺仪温度漂移问题
		Green_Off;
//调用vTaskSuspend函数是不会累计的：即使多次调用vTaskSuspend ()函数将一个任务挂起，也只需调用一次vTaskResume ()函数就能使挂起的任务解除挂起状态。
		if(currentTime >= REMOTE_ulGetLostTime( ) || REMOTE_IfDataError() == TRUE )//遥控失联时间太长
		{
			vTaskSuspend(Task_Chassis_Handler);		//将任务挂起
			vTaskSuspend(Task_Gimbal_Handler);
			vTaskSuspend(Task_Revolver_Handler);
			vTaskSuspend(Task_10ms_Handler);
			
			vTaskResume(Task_OutCtl_Handler);//解挂失控保护控制任务
		}
		else 
		{
			vTaskResume(Task_Chassis_Handler);		//恢复任务
			vTaskResume(Task_Gimbal_Handler);
			vTaskResume(Task_Revolver_Handler);
			vTaskResume(Task_10ms_Handler);
			
			vTaskSuspend(Task_OutCtl_Handler);//挂起失控保护控制任务
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_50MS);//绝对延时50ms
	}
}

