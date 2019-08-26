/*!
 * \brief: 串口2,新祖传代码
 */

#include "usart2.h"

/* ARM®-based 32-bit MCUs  
   Table 42. DMA1 request mapping 
	 
   UART5_RX  ----> DMA1 Ch4 Stream0 
   UART4_RX  ----> DMA1 Ch4 Stream2 
   USART3_RX ----> DMA1 Ch4 Stream1 
   USART2_RX ----> DMA1 Ch4 Stream5   */




/* 串口2引脚定义 */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_2
#define    GPIO_PINSOURCE_TX         GPIO_PinSource2
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_3
#define    GPIO_PINSOURCE_RX         GPIO_PinSource3
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA




/* 串口2用于读取遥控器接收机数据 */

/* 当接收机和发射机建立连接后，接收机会每隔 7ms 通过 DBUS 发送一帧数据（18 字节）
	 配置DMA接收串口数据时，就将传输数据长度设为 一帧数据的长度 */
#include "remote.h"




/* 串口2接收缓存长度 */
#define    COM2_BUFFER_LEN          30




/* 串口2接收缓存，以字节为单位 */
static char ucCom2Buffer[2][ COM2_BUFFER_LEN ] = {0};

/* 串口2接收数据包数 */
uint16_t usCom2PackageNum = 0;

/* 串口2中断服务函数 */
bool Start_RC_Mode;
uint32_t this_time_rx_len = 0;
void USART2_IRQHandler( void )
{
	uint8_t cTemp;
	

	/* 消除编译器警告 */
	cTemp = cTemp;

	/* 触发空闲中断 说明已经完成一个数据包的接收 */
	if (USART_GetITStatus( USART2, USART_IT_IDLE ) != RESET)
	{
		/* 清空闲中断标志位 */
		cTemp = USART2->SR;
		cTemp = USART2->DR;

		Start_RC_Mode = 1;//如果接收到遥控信号，则令该标志为1

		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			this_time_rx_len = COM2_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)COM2_BUFFER_LEN;         //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
			if(this_time_rx_len == REMOTE_DBUS_FRAME_LEN)
			{
				REMOTE_vReadData(ucCom2Buffer[0]);
			}
		}
		//Target is Memory1
		else 
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			this_time_rx_len = COM2_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)COM2_BUFFER_LEN;          //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream5, ENABLE);
			if(this_time_rx_len == REMOTE_DBUS_FRAME_LEN)
			{
				REMOTE_vReadData(ucCom2Buffer[1]);
			}
		}

		/* 刷新失联倒计时 */
		REMOTE_vUpdateLostTime( );
	}
}


/* 串口2配置 */
void USART2_Init( void )
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart2;
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef nvic;
	
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

	gpio.GPIO_Pin = GPIO_Pin_3 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio);
	
	USART_DeInit(USART2);
	usart2.USART_BaudRate = 100000;
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_Even;
	usart2.USART_Mode = USART_Mode_Rx;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
	USART_Cmd(USART2,ENABLE);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
/* -------------- Configure NVIC ---------------------------------------*/
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//usart rx idle interrupt  enabled
	USART_Cmd(USART2,ENABLE);
/* -------------- Configure DMA -----------------------------------------*/
	
	DMA_DeInit(DMA1_Stream5);
	DMA_StructInit(&dma);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)&ucCom2Buffer[0][0];
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = sizeof(ucCom2Buffer)/2;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5,&dma);
	
	//配置Memory1,Memory0是第一个使用的Memory
	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)&ucCom2Buffer[1][0], DMA_Memory_0);   //first used memory configuration
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);

	DMA_Cmd(DMA1_Stream5,ENABLE);
}
