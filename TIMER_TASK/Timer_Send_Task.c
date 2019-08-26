#include "Timer_Send_Task.h"


/* 队列 */
QueueHandle_t	CAN1_Queue;					//CAN1消息队列句柄
QueueHandle_t	CAN2_Queue;					//CAN2消息队列句柄

TimerHandle_t	CAN1_Timer_Handle; 			//周期定时器句柄					
TimerHandle_t	CAN2_Timer_Handle; 			//周期定时器句柄

/**
  * @brief  软件定时回调函数
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

	while(xQueueReceive(CAN1_Queue, &SendCanTxMsg, 0))//接收队列信息,禁止等待！！！
	{
//        do
//		{
//			//CAN故障重启
//			if(CAN1->ESR)
//			{
//				CAN1->MCR |= 0x02;
//				CAN1->MCR &= 0xFD;
//			}
//		}while(!(CAN1->TSR & 0x1C000000));
		
		CAN_Transmit(CAN1, &SendCanTxMsg);//发送目标值
    }
}

/**
  * @brief  软件定时回调函数
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

	while(xQueueReceive(CAN2_Queue, &SendCanTxMsg, 0))//接收队列信息,禁止等待！！！
	{
//        do
//		{
//			//CAN故障重启
//			if(CAN2->ESR)
//			{
//				CAN2->MCR |= 0x02;
//				CAN2->MCR &= 0xFD;
//			}
//		}while(!(CAN2->TSR & 0x1C000000));
		
		CAN_Transmit(CAN2, &SendCanTxMsg);//发送目标值
    }
}

void Timer_Send_Create(void)
{
	taskENTER_CRITICAL(); 	//进入临界区
	
	/*--------------------------数据接收--------------------------*/	

	//创建can1接收队列
		//can1接收到的报文存放在此队列中
	CAN1_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//最多可保持64个CanTxMsg
	
	//创建can2接收队列
		//can2接收到的报文存放在此队列中
	CAN2_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//
	
	//创建can1发送定时器
	 CAN1_Timer_Handle=xTimerCreate((const char*	)"CAN1_Timer",
									 (TickType_t 	)TIME_STAMP_2MS,//2ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)0,				//编号一般给0
									 (TimerCallbackFunction_t)CAN1_Timer_Callback);//回调函数
	
	//开启CAN1定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
	if( CAN1_Timer_Handle != NULL )
	{
		xTimerStart(CAN1_Timer_Handle,0);//不等待
	}
									 
	//创建can2发送定时器
	 CAN2_Timer_Handle=xTimerCreate((const char*	)"CAN2_Timer",
									 (TickType_t 	)TIME_STAMP_1MS,//1ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)1,				//编号一般给0
									 (TimerCallbackFunction_t)CAN2_Timer_Callback);//回调函数
	
			
									 
	//开启CAN2定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
	if( CAN2_Timer_Handle != NULL )
	{
		xTimerStart(CAN2_Timer_Handle,0);//不等待
	}	
	
	taskEXIT_CRITICAL();	//退出临界区
}
