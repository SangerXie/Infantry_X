#include "remote.h"

#include "control.h"
#include "usart2.h"
#include "led.h"
#include "iwdg.h"

#include "Task_Chassis.h"
#include "Task_Gimbal.h"

/* 超过这个时间没有收到新的遥控器数据就认为已经失控 */
#define    REMOTE_LOST_TIME    ((uint32_t)50)   //50ms


/* 存放读取到的遥控器数据 */
RC_Ctl_t RC_Ctl;


portTickType ulRemoteLostTime = 0;




/* 获取遥控器失联倒计时 */
uint32_t REMOTE_ulGetLostTime( void )
{
	/* */
	return  ulRemoteLostTime;
}


/* 复位数据 */
void REMOTE_vResetData( void )
{
	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch0 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch1 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch2 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch3 = RC_CH_VALUE_OFFSET;

	/* Switch left, right */
	RC_Ctl.rc.s1  = RC_SW_MID;
	RC_Ctl.rc.s2  = RC_SW_MID;

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = 0;
	RC_Ctl.mouse.y = 0;
	RC_Ctl.mouse.z = 0;

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = 0;
	RC_Ctl.mouse.press_r = 0;

	/* KeyBoard value */
	RC_Ctl.key.v = 0;
}


/* 刷新失联时间 */
void REMOTE_vUpdateLostTime( void )
{
	/* 当接收到数据时，刷新失联倒计时 */
	ulRemoteLostTime = xTaskGetTickCount( ) + REMOTE_LOST_TIME;
}



//遥控数据混乱
bool_t REMOTE_IfDataError( void )
{
	if ( (RC_Ctl.rc.s1 != RC_SW_UP && RC_Ctl.rc.s1 != RC_SW_MID && RC_Ctl.rc.s1 != RC_SW_DOWN)
		|| (RC_Ctl.rc.s2 != RC_SW_UP && RC_Ctl.rc.s2 != RC_SW_MID && RC_Ctl.rc.s2 != RC_SW_DOWN)
		|| (RC_Ctl.rc.ch0 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch0 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch1 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch1 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch2 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch3 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}




bool_t REMOTE_IfKeyReset( void )
{
	uint8_t IfReset = FALSE;


	if (SYSTEM_GetRemoteMode( ) == KEY)
	{
		if ( IF_KEY_PRESSED_CTRL 
			&& IF_KEY_PRESSED_SHIFT 
			&& IF_KEY_PRESSED_Z     )
		{
			IfReset = TRUE;
		}
	}

	return IfReset;
}




/* 读取串口缓存的数据，按照遥控器通信协议转换为遥控器数据
   接收机发送数据的频率是每隔 7ms 发送一帧
   为防止在DMA写入缓存的同时读取缓存中的数据（可能导致读取到的数据无效），
   直接在中断服务函数中读取遥控器数据。
   另一种方法是定义多缓存，但需要重新初始化DMA，权衡之下
   还是使用上一种方法较直接简单。
*/
void REMOTE_vReadData(char *pucRxBuffer )
{
	if(pucRxBuffer == NULL)
	{
	return;
	}
	/* 缓存数据转换为遥控器数据 */

	/* 字节位数与数据的对应关系：
		 字节    数据
		 第0个   0000 0000    0：ch0        uint16_t  实际有效位数 11 位
		 第1个   1111 1000    1：ch1        uint16_t
		 第2个   2211 1111    2：ch2        uint16_t
		 第3个   2222 2222    3：ch3        uint16_t
		 第4个   3333 3332    
		 第5个   1122 3333    1：s1  2：s2  uint8_t   实际有效位数 2 位
		 第6个   xxxx xxxx    x：mouse.x    int16_t   实际有效位数 16 位
		 第7个   xxxx xxxx    
		 第8个   yyyy yyyy    y：mouse.y    int16_t   实际有效位数 16 位
		 第9个   yyyy yyyy
		 第10个  zzzz zzzz    z：mouse.z    int16_t   实际有效位数 16 位
		 第11个  zzzz zzzz
		 第12个  llll llll    l：mouse.left   uint8_t   实际有效位数 8 位
		 第13个  rrrr rrrr    r：mouse.right  uint8_t   实际有效位数 8 位
		 第14个  kkkk kkkk    k：key          uint16_t  实际有效位数 16 位
		 第15个  kkkk kkkk    k：key

		 总共16个字节，有效位数 128 = 11*4 + 2*2 + 16*3 + 8*2 + 16*1         */

	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch0 = (  pucRxBuffer[0]       | (pucRxBuffer[1] << 8 ) ) & 0x07ff;
	RC_Ctl.rc.ch1 = ( (pucRxBuffer[1] >> 3) | (pucRxBuffer[2] << 5 ) ) & 0x07ff;
	RC_Ctl.rc.ch2 = ( (pucRxBuffer[2] >> 6) | (pucRxBuffer[3] << 2 ) | (pucRxBuffer[4] << 10) ) & 0x07ff;
	RC_Ctl.rc.ch3 = ( (pucRxBuffer[4] >> 1) | (pucRxBuffer[5] << 7 ) ) & 0x07ff;

	

	/* Switch left, right */
	RC_Ctl.rc.s1  = ( (pucRxBuffer[5] >> 4) & 0x000C ) >> 2;
	RC_Ctl.rc.s2  = ( (pucRxBuffer[5] >> 4) & 0x0003 );

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = pucRxBuffer[6]  | (pucRxBuffer[7 ] << 8);
	RC_Ctl.mouse.y = pucRxBuffer[8]  | (pucRxBuffer[9 ] << 8);
	RC_Ctl.mouse.z = pucRxBuffer[10] | (pucRxBuffer[11] << 8);

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = pucRxBuffer[12];
	RC_Ctl.mouse.press_r = pucRxBuffer[13];

	/* KeyBoard value */
	RC_Ctl.key.v = pucRxBuffer[14] | (pucRxBuffer[15] << 8);
	
	
	
	/* 数据出错处理，防暴走 */
	if( REMOTE_IfDataError() == TRUE 
			|| REMOTE_IfKeyReset() == TRUE)//实验
	{
		Green_On;
		Red_On;
		Blue_On;
		Orange_On;
		REMOTE_vResetData();
		SYSTEM_OutCtrlProtect();
	}
	else
	{
		IWDG_Feed();//喂狗
	}

}
