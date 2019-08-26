#include "vision.h"
#include "math.h"

#include "remote.h"
#include "crc.h"
#include "usart4.h"
#include "Task_Gimbal.h"
#include "kalman.h"
#include "control.h"
#include "judge.h"


//右键自瞄

/*-------视觉分辨率预编译--------*/
#define	VISION_1280P	0
#define	VISION_640P		1

#define VISION_DPI		VISION_1280P

#if	VISION_DPI == VISION_1280P

	#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
		#define VISION_MID_YAW		444//640
		#define VISION_MID_PITCH	500//360
	
	#elif		INFANTRY_DEBUG_ID == DEBUG_ID_ONE
		#define VISION_MID_YAW		444//640
		#define VISION_MID_PITCH	500//360
	
	#elif		INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		#define VISION_MID_YAW		444//640
		#define VISION_MID_PITCH	500//360
		
	#elif		INFANTRY_DEBUG_ID == DEBUG_ID_THREE
		#define VISION_MID_YAW		444//640
		#define VISION_MID_PITCH	500//360
		
	#elif		INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
		#define VISION_MID_YAW		444//640
		#define VISION_MID_PITCH	500//360
		
	#endif
	
#elif VISION_DPI == VISION_640P
	#define VISION_MID_YAW		320
	#define VISION_MID_PITCH	240
	
#endif
	
/*------------------自瞄预编译,角度初始化补偿------------------------*/
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0;//-0.38f//-0.28f//-0.12f//23m/s 
	#define COMPENSATION_PITCH_DIST 0//-0.2f
	float SB_K_comps = 3.f;

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH  0;//-0.1f//-0.2f//23.5m/s
	#define COMPENSATION_PITCH_DIST 0//-0.2f
	float SB_K_comps = 6.f;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0
	#define COMPENSATION_PITCH_DIST 0
	float SB_K_comps = 3.f;

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0
	#define COMPENSATION_PITCH_DIST 0
	float SB_K_comps = 3.f;

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0
	#define COMPENSATION_PITCH_DIST 0
	float SB_K_comps = 3.f;

#endif

//角度初始化补偿
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//根据距离补偿
/*----------------------------------------------------------------*/

typedef enum
{
	VISION_MANU = 0,	//手动模式
    VISION_BUFF = 1,	//它站着不动你屏息干嘛
    VISION_AUTO = 2,	//朋友开挂
}eVisionAction;
eVisionAction actVison;


extVisionSendHeader_t    VisionSendHeader;  //头

extVisionRecvData_t      VisionRecvData;    //视觉接收结构体


extVisionSendData_t      VisionSendData;    //视觉发送结构体

uint8_t Attack_Color_Choose = ATTACK_NONE;//默认不识别

//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send   = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;

//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;

//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

/************************************************************************************/
/************************************************************************************/

/**
  * @brief  读取视觉信息
  * @param  usart4缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint32_t Vision_Time_Test[2] = {0};//前后两次事件
uint16_t Vision_Ping = 0;//测试时间间隔
void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == VISION_SOF)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				memcpy( &VisionRecvData, ReadFromUsart, VISION_LEN_PACKED);	
				Vision_Get_New_Data = TRUE;//标记视觉数据更新了
				
				//帧计算
				Vision_Time_Test[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
				
				if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//标记打符打中了，换装甲了
				{
					if(VisionRecvData.identify_buff == 2)//发2说明换装甲板了
					{
						Vision_Armor = TRUE;//发2换装甲
					}
				}
			}
		}
	}
	
//	if(VisionRecvData.yaw_angle == 99.99f)
//	{
//		memset(Com4_Vision_Buffer, 0, 100);
//	}
}

/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲
  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
uint8_t vision_send_pack[50] = {0};//大于22就行
void Vision_Send_Data( uint8_t CmdID )
{
//	uint8_t vision_send_pack[50] = {0};//大于22就行
	int i;    //循环发送次数

	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( vision_send_pack, VISION_LEN_HEADER );
	
	//中间数据不用管,视觉用不到,用到了也是后面自瞄自动开火,用到角度补偿数据
	VisionSendData.pitch_angle = 0.f;
	VisionSendData.yaw_angle   = 0.f;
	VisionSendData.distance    = 999.99f;
	if( GIMBAL_AUTO_PITCH_SB() == TRUE )
	{
		VisionSendData.lock_sentry = 1;//识别哨兵，发1
	}
	else
	{
		VisionSendData.lock_sentry = 0;//不在识别哨兵，发0
	}
	
	if(GIMBAL_If_Base() == TRUE)
	{
		VisionSendData.base = 1;//吊射基地，发1
	}
	else
	{
		VisionSendData.base = 0;//不在吊射，发0
	}
	
	VisionSendData.blank_a = 0;
	VisionSendData.blank_b = 0;
	VisionSendData.blank_c = 0;
	memcpy( vision_send_pack + VISION_LEN_HEADER, &VisionSendData, VISION_LEN_DATA);
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( vision_send_pack, VISION_LEN_PACKED );
	
	//将打包好的数据通过串口移位发送到裁判系统
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		UART4_SendChar( vision_send_pack[i] );
	}
	
	memset(vision_send_pack, 0, 50);
}

/**********************************视觉控制*****************************************/
/**
  * @brief  视觉总控制,指令更新
  * @param  void
  * @retval void
  * @attention  8位,只有键盘模式有视觉
  */
void Vision_Ctrl(void)
{
	if(1)//SYSTEM_GetRemoteMode() == KEY)//键盘模式
	{
		if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//自动打符模式
		{
			actVison = VISION_BUFF;
		}
		else if (GIMBAL_IfManulHit() == TRUE)//手动模式
		{
			actVison = VISION_MANU;
		}
		else//默认朋友开挂,常威你还敢说你不会武功
		{
			actVison = VISION_AUTO;
		}

		switch(actVison)
		{
			/*- 打符 -*/
			case VISION_BUFF:
				Vision_Buff_Ctrl();
			break;
			
			/*- 自瞄 -*/
			case VISION_AUTO:
				Vision_Auto_Attack_Ctrl();
			break;
			
			/*- 手动 -*/
			case VISION_MANU:
				Vision_Auto_Attack_Off();
			break;
		}
	}
	else
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  打符控制
  * @param  void
  * @retval void
  * @attention  
  */
#define buff_blue 0
#define buff_red  1
bool Buff_Color = 0;
uint8_t Buff_Type = 0;//裁判系统指示灯标记
void Vision_Buff_Ctrl(void)
{
	/* 大符颜色根据裁判系统反馈确定，与自身颜色相同 */
	//s1上顺时针,下逆时针,中间不识别
	Buff_Color = is_red_or_blue();
	if(GIMBAL_If_Small_Buff() == TRUE)//小符模式
	{
		if(Buff_Color == buff_blue)
		{
			Vision_Send_Data( VISION_BBUFF_STAND );//蓝小
			Buff_Type = VISION_BBUFF_ANTI;//指示灯为绿色
		}
		else if(Buff_Color == buff_red)
		{
			Vision_Send_Data( VISION_RBUFF_STAND );//红小
			Buff_Type = VISION_RBUFF_CLOCKWISE;//指示灯为红色
		}
	}
	else if(GIMBAL_If_Big_Buff() == TRUE)//大符模式
	{
		if(IF_RC_SW1_UP)//顺时针
		{
			if(Buff_Color == buff_blue)
			{
				Vision_Send_Data( VISION_BBUFF_CLOCKWISE );//蓝顺
				Buff_Type = VISION_BBUFF_CLOCKWISE;
			}
			else if(Buff_Color == buff_red)
			{
				Vision_Send_Data( VISION_RBUFF_CLOCKWISE );//红顺
				Buff_Type = VISION_RBUFF_CLOCKWISE;
			}
		}
		else if(IF_RC_SW1_DOWN)//逆时针
		{
			if(Buff_Color == buff_blue)
			{
				Vision_Send_Data( VISION_BBUFF_ANTI );//蓝逆
				Buff_Type = VISION_BBUFF_ANTI;
			}
			else if(Buff_Color == buff_red)
			{
				Vision_Send_Data( VISION_RBUFF_ANTI );//红逆
				Buff_Type = VISION_RBUFF_ANTI;
			}
		}
	}
	else
	{
		Vision_Send_Data( VISION_OFF );
		Buff_Type = VISION_OFF;
	}
}

/**
  * @brief  自瞄控制
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* 确定敌方颜色 */
	//s1上识别红,下识别蓝,中间不识别
	if(IF_RC_SW1_UP)
	{
		Attack_Color_Choose = ATTACK_RED;
	}
	else if(IF_RC_SW1_DOWN)
	{
		Attack_Color_Choose = ATTACK_BLUE;
	}
	else
	{
		Attack_Color_Choose = ATTACK_NONE;
	}
	
	
	//向小电脑发送颜色识别指令
	if(Attack_Color_Choose == ATTACK_BLUE)
	{
		Vision_Send_Data( VISION_BLUE );
	}
	else if(Attack_Color_Choose == ATTACK_RED)
	{
		Vision_Send_Data( VISION_RED );
	}
	else if(Attack_Color_Choose == ATTACK_NONE)
	{
		Vision_Auto_Attack_Off();
	}
}


/**
  * @brief  关闭自瞄
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
	Vision_Send_Data( VISION_OFF );
}

/*******************************视觉误差获取*************************************/
/**
  * @brief  获取yaw误差像素(x轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点左边,正数表示在右边
  */
void Vision_Error_Yaw(float *error)
{
	if(VisionRecvData.yaw_angle != 0)
	{
		//输出为负时云台右移,为正时左移
		*error = -(VisionRecvData.yaw_angle - VISION_MID_YAW);
	}
	else
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差像素(y轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点上方,正数表示在下方
  */
void Vision_Error_Pitch(float *error)
{	
	if(VisionRecvData.pitch_angle != 0)
	{
		*error = VisionRecvData.pitch_angle - VISION_MID_PITCH;
	}
	else
	{
		*error = 0;
	}
}
/*-----------------------------------------------------------------*/
/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = (-VisionRecvData.yaw_angle + Vision_Comps_Yaw * VisionRecvData.distance/100) * 20;
//				* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//距离很远时开放鼠标补偿
float vision_pitch_dist = 2;//固定距离,超过此距离开启距离补偿
float vision_pitch_dist_far = 4.4f;//超过此距离开放鼠标补偿
void Vision_Error_Angle_Pitch(float *error)
{	
	
	if(GIMBAL_AUTO_PITCH_SB() == TRUE)
	{
		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch/SB_K_comps * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角
	}
	else if(VisionRecvData.distance/100 >= vision_pitch_dist_far)
	{
		mouse_pitch_comps += MOUSE_Y_MOVE_SPEED * kvision_mouse_pitch;//注意正负
		//限幅，防止太大
		mouse_pitch_comps = constrain_float(mouse_pitch_comps, -3, 0);
		*error = (VisionRecvData.pitch_angle
						+ Vision_Comps_Pitch * VisionRecvData.distance/100
							+ mouse_pitch_comps
				 )
				 * 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角		
		
	}
	else
	{
		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角
	}
	
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}

/*-----------------------------------------------------------------*/
/**
  * @brief  获取yaw误差角度，打符专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在底盘
  */
void Vision_Buff_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = -VisionRecvData.yaw_angle*YAW_POSITION;//取负是为了对应yaw左转机械角增加
	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取yaw误差角度，打符专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在云台
  */
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = -VisionRecvData.yaw_angle;//取负是为了对应yaw左转机械角增加
	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度，打符专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
void Vision_Buff_Error_Angle_Pitch(float *error)
{	
	//视觉上负下正,注意云台正负是抬头还是低头(上减下加)
	*error = VisionRecvData.pitch_angle;//pitch云台在右边时，向上转云台机械角减小
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}

/**
  * @brief  获取yaw误差像素，桥头吊射基地专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在云台
  */
void Vision_Base_Yaw_Pixel(float *error)
{
	if(VisionRecvData.yaw_angle != 0)
	{
		//输出为负时云台右移,为正时左移
		*error = -(VisionRecvData.yaw_angle - 640);
	}
	else
	{
		*error = 0;
	}
}

/*-----------------------------------------------------------------*/

/**
  * @brief  获取距离
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance < 0)
	{
		*distance = 0;
	}
}

/**
  * @brief  位置补偿
  * @param  yaw,pitch
  * @retval void
  * @attention  正数pitch低头,正数yaw左扭头
  */
float vc_distdebug_y = 0;
float vc_distdebug_p = 0;
void Vision_Compensation(float *comps_yaw, float *comps_pitch)
{
	float dista_level = 0;//单位米
	
	//debug注释掉
//	dista_level = VisionRecvData.distance;//读取距离
	
	if( GIMBAL_IfBuffHit() == TRUE )//打符模式
	{
		*comps_yaw   = Vision_Comps_Yaw;
		*comps_pitch = Vision_Comps_Pitch;
	}
	else
	{
		//根据距离调整角度补偿
		if(dista_level >= 0 && dista_level < 4)
		{
			*comps_yaw   = Vision_Comps_Yaw;
			*comps_pitch = Vision_Comps_Pitch;
			
		}
		else if(dista_level >= 4 && dista_level < 8)
		{
			*comps_yaw   = Vision_Comps_Yaw   + vc_distdebug_y;
			*comps_pitch = Vision_Comps_Pitch + vc_distdebug_p;
		}
		else if(dista_level >= 8 && dista_level < 10)
		{
			*comps_yaw   = Vision_Comps_Yaw   + vc_distdebug_y;
			*comps_pitch = Vision_Comps_Pitch + vc_distdebug_p;
		}
		else		//防止距离出错
		{
			*comps_yaw   = Vision_Comps_Yaw;
			*comps_pitch = Vision_Comps_Pitch;
		}
	}
	
	Vision_Comps_Yaw_Send   = (int)(*comps_yaw);
	Vision_Comps_Pitch_Send = (int)(*comps_pitch);
}


/*******************视觉辅助函数*********************/

/**
  * @brief  自瞄颜色判断
  * @param  void
  * @retval
  * @attention  裁判系统左到右第二个灯红为识别红,绿为识别蓝,不识别红绿闪烁
  */
uint8_t VISION_isColor(void)
{
	return Attack_Color_Choose;
}

/**
  * @brief  打符指令判断
  * @param  void
  * @retval 
  * @attention  裁判系统左到右第三个灯红为顺时针,绿为逆时针,不识别红绿闪烁
  */
uint8_t VISION_BuffType(void)
{
	return Buff_Type;
}

/**
  * @brief  判断发送的指令与视觉接收到的指令是否相同
  * @param  void
  * @retval TRUE指令一样    FALSE指令不一样
  * @attention  视觉收到什么指令,就发同样的指令回来
  */
bool VISION_IfCmdID_Identical(void)
{
	if (VisionRecvData.CmdID == VisionSendHeader.CmdID)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool Vision_If_Armor(void)
{
	return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Ammor_Flag(void)
{
	Vision_Armor = FALSE;
}
