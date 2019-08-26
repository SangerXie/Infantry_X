#include "Task_Revolver.h"


#include "can2.h"
#include "control.h"
#include "remote.h"
#include "Task_Gimbal.h"

#include "judge.h"
#include "friction.h"
#include "vision.h"
#include "super_cap.h"
#include "led.h"


/*****************高射频必须降低射速，拨弹必须和摩擦轮配合***********************/


#define REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速

/*       拨盘型号预编译       */
#define SEVEN_REVOLVER    0    	//7格拨盘
#define EIGHT_REVOLVER    1		//8格拨盘
#define TEN_REVOLVER      2		//10格拨盘
#define TWELVE_REVOLVER   3		//12格拨盘

#define REVOLVER_CHOOSE  TWELVE_REVOLVER	//选择拨盘型号

#if REVOLVER_CHOOSE == EIGHT_REVOLVER			
	#define		REVOL_SPEED_GRID      8				//拨盘格数
	#define 	AN_BULLET         (36864.0f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == SEVEN_REVOLVER
	#define 	REVOL_SPEED_GRID      7				//拨盘格数
	#define   	AN_BULLET     	  (42130.2857f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == TEN_REVOLVER
	#define 	REVOL_SPEED_GRID  	  10			//拨盘格数
	#define    	AN_BULLET     	  (29491.2f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == TWELVE_REVOLVER
	#define 	REVOL_SPEED_GRID      12			//拨盘格数
	#define    	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值

#endif	

/******拨盘,控制逻辑与云台类似*********/

//拨盘电机模式,位置环与速度环
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//拨盘模式选择
//拨盘电机模式,位置环与速度环
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;
eRevolverCtrlMode Revolver_mode;

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;
eShootAction actShoot;
#define    REVOL_CAN_OPEN    350  //摩擦轮实际速度超过这个值才允许拨盘转动,根据摩擦轮最小目标速度来改变


/*******************拨盘参数**********************/
//拨盘测量速度
int16_t Revolver_Speed_Measure;

//拨盘测量角度
int16_t Revolver_Angle_Measure;

//拨盘速度误差
float Revolver_Speed_Error;

//拨盘角度误差
float Revolver_Angle_Error[2];//  inner/outer

//累计和
float Revolver_Angle_Measure_Sum;//拨盘测量角度累计和,用于位置PID
int16_t Revolver_Angle_Measure_Prev;//上次剩下的累加和角度,用于圈数计算判断

//拨盘目标角度
float  Revolver_Angle_Target;

//拨盘目标角度累计和,用于位置PID计算
float  Revolver_Angle_Target_Sum;
float  Revolver_Buff_Target_Sum;//打符模式下的目标值暂存，此时目标角度用斜坡处理
float  Revolver_Buff_Ramp = AN_BULLET/40;//40ms转一格,一定不能超过50ms

//拨盘目标转速
float  Revolver_Speed_Target;//转速过低容易卡弹,尽量让转速上6000

//拨盘电机输出量,正数逆时针
float Revolver_Final_Output;

/****************射频控制******************/
#define SHOOT_LEFT_TIME_MAX  150	//左键连按切换间隔

//拨盘速度环射频
int16_t Revolver_Freq;

//位置环射击间隔,实时可变,数值越小位置环射击间隔越短
uint32_t Shoot_Interval = 0;

//射击间隔总响应，模式切换时及时重置成当前时间
uint32_t  Revol_Posit_RespondTime = 0;

////打符射击间隔
//uint32_t Shoot_Buff_Interval = TIME_STAMP_400MS;

////自瞄自动发弹射击间隔
//uint32_t Shoot_Auto_Interval = TIME_STAMP_1000MS;

/*****************PID参数*****************/
float pTermRevolSpeed, iTermRevolSpeed;	
float pTermRevolAngle[2], iTermRevolAngle[2];//  inner/outer
float Revolver_Speed_kpid[3];//	kp/ki/kd
float Revolver_Angle_kpid[2][3];//  inner/outer    kp/ki/kd


/**********限幅*************/
//最终输出限幅
float Revolver_Output_Max;
float iTermRevolSpeedMax;//速度环微分限幅
float iTermRevolPosiMax;//位置环微分限幅



/********射击**********/
//发射子弹数,按一下加一颗,发一颗减一次
int16_t Key_ShootNum;//鼠标射击计数
int16_t ShootNum_Allow = 0;//还剩几颗弹可以打
//int16_t Residue_ShootNum = 0;//剩余可发弹量
uint16_t Residue_Heat;//剩余可用热量,热量限制控制
uint16_t Shoot_HeatLimit;//当前等级最大热量上限
uint16_t Shoot_HeatIncSpeed;//当前摩擦轮转速下单发子弹热量增加值

/************卡弹************/
#define Stuck_Revol_PIDTerm   4000      //PID输出大于这个数则认为有可能卡弹
#define Stuck_Speed_Low       60       //测量速度低于这个数,则认为有可能卡弹

#define Stuck_SpeedPID_Time   100       //速度连续 ms过小,PID连续  ms过大
#define Stuck_TurnBack_Time   100       //倒转时间,时间越长倒得越多
uint32_t Stuck_Speed_Sum = 0;//计算卡弹次数,速度环
uint32_t Stuck_Posit_Sum = 0;//计算卡弹次数,位置环

portTickType posishoot_time;//射击延时测试

/*点射*/
uint8_t Revol_Switch_Left = 0;
u8 	 Revol_Key_Left_Change = 0;

uint8_t First_Into_Buff = FALSE;
uint8_t Buff_Shoot_Begin = FALSE;
bool 	buff_fire = 0;
bool	buff_change_fire = 0;
/************************************************************************************/
/************************************************************************************/

uint8_t revol_remot_change = TRUE;
void Task_Revolver(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		CAP_Ctrl();//电容充放电控制函数,时间戳问题，所以丢这里了
		
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)
		{
			REVOLVER_Rest();
			REVOLVER_InitArgument();
		}
		else
		{
			if (SYSTEM_GetRemoteMode() == RC)
			{
				REVOLVER_Rc_Ctrl();		
			}
			else
			{
				REVOLVER_Key_Ctrl();
				revol_remot_change = TRUE;
			}
		}
		
		//热量限制
		if(Revolver_Heat_Limit() == FALSE //一定要放在速度、位置控制之前
			&& GIMBAL_IfBuffHit() != TRUE 
				&& GIMBAL_IfManulHit() != TRUE)//打符时不限热量
		{
			REVOLVER_Rest();//发弹清零,哪怕鼠标继续点击也不给发弹
		}
		
		if(Revolver_mode == REVOL_SPEED_MODE)
		{
			REVOL_SpeedLoop();
		}
		else if(Revolver_mode == REVOL_POSI_MODE)
		{
			REVOL_PositionLoop();
		}
		
		if(Fric_GetSpeedReal() > REVOL_CAN_OPEN)//摩擦轮开启
		{
			REVOLVER_CANbusCtrlMotor();
		}
		else
		{
			Revolver_Speed_Target = 0;//摩擦轮关闭,拨盘不给动
			Revolver_Angle_Rest();//摩擦轮关闭，屏蔽这期间的打弹指令
			REVOL_SpeedLoop();
			REVOLVER_CANbusCtrlMotor();
		}

		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
	}
}
	


/********************************************************************************/
/**
  * @brief  拨盘失控保护
  * @param  void
  * @retval void
  * @attention 所有输出置0
  */
void REVOLVER_StopMotor(void)
{
	Revolver_Final_Output = 0;
	CAN2_Revolver_QueueSend(Revolver_Final_Output);
}

/**
  * @brief  拨盘参数初始化
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_InitArgument(void)
{
	/* 目标值 */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;
	
	/* PID参数 */
	  //速度环
	Revolver_Speed_kpid[KP] = 15;
	Revolver_Speed_kpid[KI] = 0;
	Revolver_Speed_kpid[KD] = 0;
	  //位置环
	Revolver_Angle_kpid[OUTER][KP] = 0.4;
	Revolver_Angle_kpid[OUTER][KI] = 0;
	Revolver_Angle_kpid[OUTER][KD] = 0;
	Revolver_Angle_kpid[INNER][KP] = 6;
	Revolver_Angle_kpid[INNER][KI] = 0;
	Revolver_Angle_kpid[INNER][KD] = 0;
	
	/* 限幅 */
	iTermRevolSpeedMax  = 250;
	iTermRevolPosiMax   = 2500;
	Revolver_Output_Max = 9999;
	
	/* 射击 */
	Key_ShootNum    = 0;
	Shoot_HeatLimit = 240;//热量初始化
	Revolver_Freq   = 0;//射频初始化
	
	/* 位置环目标角度 */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;//不能置0,否则上电会反转
	Revolver_Buff_Target_Sum  = Revolver_Angle_Measure;
}

/**
  * @brief  拨盘重启
  * @param  void
  * @retval void
  * @attention 枪口超热量重置
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//位置环发弹清零
	Revolver_Speed_Target = 0;//速度环停止转动
	
	//速度环位置重置
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Measure;
	
	//PID积分清零
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

/**
  * @brief  拨盘角度清零
  * @param  void
  * @retval void
  * @attention 模式切换时用,防止下次切回去会突然动一下
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Target_Sum;
}

/**
  * @brief  拨盘的遥控模式
  * @param  void
  * @retval void
  * @attention 遥控用速度环
  */
void REVOLVER_Rc_Ctrl(void)
{
	/**************连发版*******************/
//	Revolver_mode = REVOL_SPEED_MODE;
//	
//	if(IF_RC_SW1_DOWN)//sw1下打弹
//	{
//		if (IF_RC_SW2_DOWN)//机械模式
//		{
//			Revolver_Freq = 10;//射频选择
//			//速度环转速设置
//			Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
//		}
//		else if (IF_RC_SW2_MID)//陀螺仪模式
//		{
//			Revolver_Freq = 14;//射频选择
//			//速度环转速设置
//			Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
//		}
//		else	//键盘模式,速度环另外算
//		{
//			Revolver_Speed_Target = 0;
//			Revolver_Freq = 0;
//		}
//	}
//	else	//遥控模式关闭拨盘
//	{
//		Revolver_Speed_Target = 0;
//		Revolver_Freq = 0;
//	}

//	REVOL_SpeedStuck();//卡弹判断及倒转
	/********************************************/
	
	/*******************点射版**********************/
	
	if(IF_RC_SW2_MID)//机械模式下单发，方便测试弹道
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环
		if(REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++;//打一颗
		}
		
		if(revol_remot_change == TRUE)//刚从键盘模式切换过来，清空发弹数据
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest();//防止突然从键盘切到遥控狂转
		}
		
		if(Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Buff_Target_Sum += AN_BULLET;
		}
		
		if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//缓慢转过去
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
		}
		
		REVOL_PositStuck();//卡弹判断及倒转
	}
	/**************连发版*******************/
	else if(IF_RC_SW2_DOWN)//陀螺仪模式
	{
		Revolver_mode = REVOL_SPEED_MODE;//速度环
		
		if(IF_RC_SW1_DOWN)//sw1下打弹
		{
			Revolver_Freq = 20;//射频选择
			//速度环转速设置
			Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
		}
		else	//遥控模式关闭拨盘
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
		}

		REVOL_SpeedStuck();//卡弹判断及倒转
	}
	/********************************************/
}

/**
  * @brief  拨盘遥控打弹
  * @param  void
  * @retval void
  * @attention 
  */
#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志
uint8_t	Revolver_Switch = 0;//弹仓遥控模式开关标志位转换
bool REVOLVER_Rc_Switch(void)
{
//	if (IF_RC_SW2_MID || IF_RC_SW2_DOWN)//机械或陀螺仪模式
	if (IF_RC_SW2_MID)//机械模式
	{
		if (IF_RC_SW1_DOWN)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else		
		{
			Revolver_Switch = REVOL_STEP1;
		}
	}
	else
	{
		Revolver_Switch = REVOL_STEP0;
	}
	
	if (Revolver_Switch == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*******键盘模式************/

/**
  * @brief  拨盘的键盘模式
  * @param  void
  * @retval void
  * @attention 键盘用位置环控制
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE;//防止出错用,默认位置环
	
	SHOOT_NORMAL_Ctrl();//确定射击模式
	
	/*- 确定射击间隔和射击模式 -*/
	switch(actShoot)
	{
		case SHOOT_NORMAL:
			//射击模式选择,默认不打弹
			SHOOT_NORMAL_Ctrl();
		break;
		
		case SHOOT_SINGLE:
			//按一下左键单发,长按连发
			SHOOT_SINGLE_Ctrl();
		break;
		
		case SHOOT_TRIPLE:
			//长按连发
			SHOOT_TRIPLE_Ctrl();
//			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_HIGHTF_LOWS:
			//B高射频
			SHOOT_HIGHTF_LOWS_Ctrl();
		break;
		
		case SHOOT_MIDF_HIGHTS:
			//Z推家
			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_BUFF:
			//打符自动打弹
			Revolver_mode = REVOL_POSI_MODE;
		
			#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
				SHOOT_BUFF_Ctrl();
		
			#elif 	BUFF_CAM_TYPE == BUFF_CAM_GIMB
				SHOOT_BUFF_Ctrl_Gimbal();

			#endif
		break;
		
		case SHOOT_AUTO:
			//右键自瞄时自动打弹
//			SHOOT_AUTO_Ctrl();
		break;		
	}
	
	/*- 开始发弹,计数减 -*/
	if(Revolver_mode == REVOL_SPEED_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
	{
		REVOLVER_KeySpeedCtrl();
	}
	else if(Revolver_mode == REVOL_POSI_MODE && Fric_GetSpeedReal() > REVOL_CAN_OPEN)
	{
		REVOLVER_KeyPosiCtrl();
	}
}

/************************底盘键盘模式各类模式小函数****************************/
/**
  * @brief  键盘模式下发弹模式选择
  * @param  void
  * @retval void
  * @attention  普通模式清零计算,且不发弹
  */
void SHOOT_NORMAL_Ctrl(void)
{
	static uint32_t shoot_left_time = 0;//计算左键连按时间,时间过长切换成连发
	
	/*------ 左键抬起后才能打下一颗 -------*/
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (Revol_Switch_Left == 1)
		{
			Revol_Switch_Left = 2;
		}
		else if (Revol_Switch_Left == 2)
		{
			Revol_Switch_Left = 0;
		}
	}
	else if(!IF_MOUSE_PRESSED_LEFT)		
	{
		Revol_Switch_Left = 1;
		shoot_left_time = 0;//左键重新计时
	}
	/*------------------------------------*/
	
	if(IF_MOUSE_PRESSED_LEFT &&	shoot_left_time <= SHOOT_LEFT_TIME_MAX	//左键发弹
			&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环打弹
		shoot_left_time++;//判断长按,切换
		actShoot = SHOOT_SINGLE;	
	}
	else if(IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX	//连按大于200ms
				&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE;//连发模式
	}
	else if(IF_KEY_PRESSED_B	//高射频低射速
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_HIGHTF_LOWS;
		shoot_left_time = 0;
	}
	else if(IF_KEY_PRESSED_Z	//高射频极低射速,推家专用
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
//	else if(GIMBAL_IfAutoHit() == TRUE /*&& VisionRecvData.centre_lock==TRUE*/  //视觉说可以打了
//				 && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
//	{
//		Revolver_mode = REVOL_POSI_MODE;
//		actShoot = SHOOT_AUTO;
//		shoot_left_time = 0;
//	}
	else if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//打符模式且非手动打符模式
	{
		Revolver_mode  = REVOL_POSI_MODE;
		actShoot = SHOOT_BUFF;
		shoot_left_time = 0;
	}
	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval  = 0;//重置射击间隔
		Revol_Posit_RespondTime = xTaskGetTickCount();//重置响应
		shoot_left_time = 0;
//		Revol_Angle_Clear();//模式切换时清零计数
		Key_ShootNum = 0;
	}
	
	if(GIMBAL_IfBuffHit() == FALSE)//退出了打符模式
	{
		First_Into_Buff = TRUE;	
		Buff_Shoot_Begin = FALSE;
		buff_fire = FALSE;
	}
}

/**
  * @brief  单发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/8;//最快一秒8发
	
	if(RespondTime < CurrentTime
			&& Revol_Switch_Left == 2//与弹仓开关同理
				&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  连发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{
/********************************************************/
//	static portTickType  CurrentTime = 0;
//	static uint32_t  RespondTime = 0;//响应间隔计时
////	static uint32_t  Shoot_Interval_TRIPLE  =  0;//自瞄射击间隔计时
//	
//	CurrentTime = xTaskGetTickCount();
//	
//	Shoot_Interval = TIME_STAMP_50MS;//以20射频打3连发
//	
//	if(RespondTime < CurrentTime
//			&& Key_ShootNum == 0)
//	{
//		RespondTime = CurrentTime + TIME_STAMP_400MS;//隔0.4秒确定一次连发
//		
//		switch(JUDGE_ucGetRobotLevel())//等级决定左键连按情况下的射击间隔
//		{
//			case 0://未连服务器
//				Key_ShootNum = 3;//3连发
//			break;
//			
//			case 1://1级
//				Key_ShootNum = 3;//3连发
//			break;
//			
//			case 2://2级
//				Key_ShootNum = 4;//4连发
//			break;
//			
//			case 3://3级
//				Key_ShootNum = 5;//5连发
//			break;
//		}	
//	}
/********************************************************/


//	static portTickType  CurrentTime = 0;
//	static uint32_t  RespondTime = 0;//响应间隔计时	
//	
//	CurrentTime = xTaskGetTickCount();
//	
//	if(JUDGE_usGetShootCold() <= 40)
//	{
//		Shoot_Interval = TIME_STAMP_1000MS/8;//确定射频
//	}
//	else if(JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
//	{
//		Shoot_Interval = TIME_STAMP_1000MS/10;//确定射频
//	}
//	else if(JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
//	{
//		Shoot_Interval = TIME_STAMP_1000MS/12;//确定射频
//	}
//	else if(JUDGE_usGetShootCold() >= 160)//占领碉堡
//	{
//		Shoot_Interval = TIME_STAMP_1000MS/20;//确定射频
//	}
//	else
//	{
//		Shoot_Interval = TIME_STAMP_1000MS/8;//确定射频
//	}
//	
//	if(RespondTime < CurrentTime
//			&& Key_ShootNum == 0)
//	{
//		RespondTime = CurrentTime + Shoot_Interval;
//		Key_ShootNum++;
//	}

/*************************************************/
	Revolver_mode = REVOL_SPEED_MODE;

	if(JUDGE_usGetShootCold() <= 40)
	{
		Revolver_Freq = 8;//射频选择
	}
	else if(JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
	{
		Revolver_Freq = 8;//10;//射频选择
	}
	else if(JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
	{
		Revolver_Freq = 8;//12;//射频选择
	}
	else if(JUDGE_usGetShootCold() >= 160)//占领碉堡
	{
		Revolver_Freq = 14;//12;//射频选择
	}
	else
	{
		Revolver_Freq = 8;//射频选择
	}
	
	//速度环转速设置
	Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
}

/**
  * @brief  高射频低射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	
	CurrentTime = xTaskGetTickCount();
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  中射频高射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	static portTickType  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	
	CurrentTime = xTaskGetTickCount();//当前系统时间
	
	Shoot_Interval = TIME_STAMP_1000MS/20;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  自瞄射击控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_AUTO_Ctrl(void)
{
	static portTickType CurrentTime     = 0;
	static uint32_t RespondTime_Stop    = 0;//响应间隔计时，静止
	static uint32_t RespondTime_MobiPre = 0;//响应间隔计时，移动预测
	CurrentTime = xTaskGetTickCount();

/***********************************************************************/
	if( GIMBAL_IfAuto_MobPre_Yaw() == TRUE)	//开启了预测			
	{
		Shoot_Interval = TIME_STAMP_1000MS/15;//TIME_STAMP_50MS;//确定射频
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE				//自己算预测到了位置
				&& RespondTime_MobiPre < CurrentTime
					&& Key_ShootNum == 0 
						&& IF_MOUSE_PRESSED_LEFT)//左键按下
		{
			RespondTime_MobiPre = CurrentTime + Shoot_Interval;
			Key_ShootNum ++;
		}
		else//开启了预测但预测不到位，禁止打弹
		{
			Key_ShootNum = 0;
		}
	}
	else if(GIMBAL_IfAuto_MobPre_Yaw() == FALSE)	//没开预测
	{
		Shoot_Interval = TIME_STAMP_1000MS/5;//确定射频
		if(GIMBAL_MOBPRE_YAW_FIRE()==TRUE		//自己算预测到了位置
				&& RespondTime_Stop < CurrentTime
					&& Key_ShootNum == 0
						&& IF_MOUSE_PRESSED_LEFT)//左键按下				
		{
			RespondTime_Stop = CurrentTime + Shoot_Interval;//TIME_STAMP_500MS;//每隔0.5s三连发一次		
//			Key_ShootNum = 3;
			Key_ShootNum ++;
		}
	}
}

/**
  * @brief  打符射击控制,仅用于摄像头位于底盘
  * @param  void
  * @retval void
  * @attention  每隔500ms判断一次到位，到位开火
  */
uint32_t buff_shoot_time = 500;
uint32_t New_Armor_Time = 0;//稳定时间
int buff_time = 750;//500;//1200;
int shootbuff_flag = 0;
void SHOOT_BUFF_Ctrl(void)
{
		   portTickType CurrentTime = 0;
	static uint32_t		RespondTime = 0;//响应间隔计时
//	static uint32_t 	New_Armor_Time = 0;//稳定时间
	
	CurrentTime = xTaskGetTickCount();
	
	if(!IF_MOUSE_PRESSED_RIGH)//右键松开自动打弹
	{
		//子弹飞过去要600ms左右，所以600后还在瞄，说明没打中，补一发
		Shoot_Interval = buff_time;//TIME_STAMP_1000MS/1;//最快一秒1发，BUG,会在REVOLVER_KeyPosiCtrl()里计时错乱
		
		if( Vision_If_Armor() == TRUE )//刚换装甲板，给一定时间后再连续发弹
		{
			Vision_Clean_Ammor_Flag();//等待下次更新装甲板
			New_Armor_Time = 0;//装甲稳定重新计时
			RespondTime = CurrentTime - 1;
			Revol_Posit_RespondTime = xTaskGetTickCount();//重置响应
		}

		if( VisionRecvData.identify_buff == TRUE || VisionRecvData.identify_buff == 2 )
		{
			New_Armor_Time++;
			if( New_Armor_Time > buff_shoot_time  //换装甲后稳定500ms才开始打弹
					&& RespondTime < CurrentTime
						&& Key_ShootNum == 0)
			{
				RespondTime = CurrentTime + Shoot_Interval;
				Key_ShootNum++;//打一发
			}
		}
		else if(VisionRecvData.identify_buff == 0)//没识别到
		{
			New_Armor_Time = 0;//装甲稳定重新计时
		}
	}
	else//按住右键接管自动打弹
	{
		/*------ 左键抬起后才能打下一颗 -------*/
		if (IF_MOUSE_PRESSED_LEFT)
		{
			if (Revol_Switch_Left == 1)
			{
				Revol_Switch_Left = 2;
			}
			else if (Revol_Switch_Left == 2)
			{
				Revol_Switch_Left = 0;
			}
		}
		else if(!IF_MOUSE_PRESSED_LEFT)		
		{
			Revol_Switch_Left = 1;
		}
		
		/*------------------------------------*/
		if(IF_MOUSE_PRESSED_LEFT)//左键发弹
		{
			SHOOT_SINGLE_Ctrl();
		}
	}
}

/**
  * @brief  打符射击控制，摄像头在云台
  * @param  void
  * @retval void
  * @attention  每隔500ms判断一次到位，到位开火
  */
uint32_t buff_lost_time = 0;//掉帧延时，超过时间才认为掉帧
uint32_t buff_change_lost_time = 0;//切装甲掉帧延时
uint32_t buff_shoot_close = 0;
float buff_stamp = 800;//1100;//隔0.8秒补一发
float buff_lost_stamp = 200;//连续200ms丢失目标
float Armor_Change_Delay = 0;
float dy = 80;//1;//130;
void SHOOT_BUFF_Ctrl_Gimbal(void)
{
		   portTickType CurrentTime = 0;
	static uint32_t		RespondTime = 0;//响应间隔计时
	static uint32_t lockon_time = 0;//稳定瞄准一段时间后可击打
	
	CurrentTime = xTaskGetTickCount();
	
	if(!IF_MOUSE_PRESSED_RIGH)//右键松开自动打弹
	{	
		//掉帧统计，掉帧太严重不打弹
		if(VisionRecvData.identify_buff == FALSE)//没识别到目标
		{
			buff_lost_time++;
			if(buff_lost_time > buff_lost_stamp)
			{
				buff_fire = FALSE;
			}
		}
		else
		{
			buff_lost_time = 0;
			buff_fire = TRUE;//可以开火
		}
		
		Shoot_Interval = 200;//控制最高射频,不给太大防止疯狂打弹
		
		//识别到目标且接近目标
		if( buff_fire == TRUE //非长时间掉帧
				&& Vision_If_Armor() == FALSE)//非装甲切换
		{
//			buff_shoot_close = 0;//重新计算未瞄准目标时间
			Armor_Change_Delay++;
			if(Armor_Change_Delay > 50)
			{
				if(GIMBAL_BUFF_YAW_READY() && GIMBAL_BUFF_PITCH_READY())
				{
					buff_change_lost_time = 0;//刷新到位判断时间
					buff_change_fire = TRUE;//可以做切换后的稳定计时
				}
				else
				{
					buff_change_lost_time++;//稳定到位
					if(buff_change_lost_time > 50)//连续掉帧50ms，认为没到位
					{
						buff_change_fire = FALSE;//不给切换稳定计时
					}
				}
				
				if(buff_change_fire == TRUE)
				{
					lockon_time++;
				}
				else
				{
					lockon_time = 0;
				}
				
				if( RespondTime < CurrentTime
						&& Key_ShootNum == 0 
							&& lockon_time > dy//80
								&& (VisionRecvData.yaw_angle != 0 && VisionRecvData.pitch_angle != 0))//稳定到位30ms
				{
					RespondTime = CurrentTime + buff_stamp;//Shoot_Interval;
					Key_ShootNum++;//打一发
				}
			}
		}
		else//长时间掉帧或者切换了装甲
		{
			lockon_time = 0;
			buff_change_fire = FALSE;//不给切换稳定计时
			
			if( Vision_If_Armor() == TRUE )//切换装甲板
			{
				Armor_Change_Delay = 0;
				Vision_Clean_Ammor_Flag();//等待下次更新装甲板
			}
			
			RespondTime = CurrentTime-1;//立马刷新射击时间
			Key_ShootNum = 0;
			
//			buff_shoot_close++;
//			lockon_time = 0;
//			if(buff_shoot_close > 100)//连续100ms没瞄到目标
//			{
//				RespondTime = CurrentTime-1;//立马刷新射击时间
//				Key_ShootNum = 0;
//			}
		}
	}
	else//按住右键接管自动打弹
	{
		/*------ 左键抬起后才能打下一颗 -------*/
		if (IF_MOUSE_PRESSED_LEFT)
		{
			if (Revol_Switch_Left == 1)
			{
				Revol_Switch_Left = 2;
			}
			else if (Revol_Switch_Left == 2)
			{
				Revol_Switch_Left = 0;
			}
		}
		else if(!IF_MOUSE_PRESSED_LEFT)		
		{
			Revol_Switch_Left = 1;
		}
		
		/*------------------------------------*/
		if(IF_MOUSE_PRESSED_LEFT)//左键发弹
		{
			SHOOT_SINGLE_Ctrl();
		}
	}
}

/**
  * @brief  键盘模式拨盘速度环控制
  * @param  void
  * @retval void
  * @attention 推家模式超高射频,后续记得加入一个标志位用来告诉摩擦轮调低射速
  */
void REVOLVER_KeySpeedCtrl(void)
{
	REVOL_SpeedStuck();//卡弹判断及倒转
}

/**
  * @brief  键盘模式拨盘位置环控制
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_KeyPosiCtrl(void)
{
	static portTickType  CurrentTime = 0;
//	static uint32_t  RespondTime = 0;//响应间隔计时
	
	CurrentTime = xTaskGetTickCount();
	
	if(Key_ShootNum != 0 && Revol_Posit_RespondTime < CurrentTime)
	{
		Revol_Posit_RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum--;//发弹计数减
		Revolver_Buff_Target_Sum += AN_BULLET;//拨盘位置加
		
		posishoot_time = xTaskGetTickCount();//单发指令下达时的系统时间,用于发射延时测试
	}		
	
	if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//缓慢转过去
	{
		Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
	}
	
	REVOL_PositStuck();//卡弹判断及倒转,放前面更合理一点
}

/**
  * @brief  发送拨盘电流值
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_CANbusCtrlMotor(void)
{	
	CAN2_Revolver_QueueSend(Revolver_Final_Output);
}


/*******************拨盘电机数据更新*********************/

/**
  * @brief  获取电机角度
  * @param  CAN数据
  * @retval void
  * @attention  CAN2中断中调用
  */
void REVOLVER_UpdateMotorAngle(int16_t angle)
{
    Revolver_Angle_Measure = angle;
}

/**
  * @brief  获取电机转速
  * @param  CAN数据
  * @retval void
  * @attention  CAN2中断中调用
  */
void REVOLVER_UpdateMotorSpeed(int16_t speed)
{
	  Revolver_Speed_Measure = speed;
}

/**
  * @brief  统计转过角度总和
  * @param  void
  * @retval void
  * @attention 切换了模式之后记得清零 
  */
void REVOL_UpdateMotorAngleSum(void)
{		 
	//临界值判断法
	if (abs(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;
		}
		else
		{
			//超过了一圈
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;
	}

	//记录此时电机角度,下一次计算转过角度差用,用来判断是否转过1圈
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}

/***********************PID控制**********************/

/**
  * @brief  速度环PID控制
  * @param  void
  * @retval void
  * @attention  遥控只有速度环
  */
void REVOL_SpeedLoop(void)
{  
	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//典型单级PID算法
	pTermRevolSpeed   = Revolver_Speed_Error * Revolver_Speed_kpid[KP];
	iTermRevolSpeed  += Revolver_Speed_Error * Revolver_Speed_kpid[KI];
	iTermRevolSpeed   = constrain( iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax );

	Revolver_Final_Output = constrain_float( pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max );
}

/**
  * @brief  位置环PID控制
  * @param  void
  * @retval void
  * @attention  键盘模式
  */
void REVOL_PositionLoop(void)
{
	//获取转过的总角度值
	REVOL_UpdateMotorAngleSum( );
	
	//外环计算
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];

	//内环计算
	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];		
	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );

	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);

}


/*********************射频热量限制****************************/

/**
  * @brief  枪管热量限制
  * @param  void
  * @retval 热量是否超限
  * @attention  超限要重置一下拨盘,根据剩余可发弹量来做闭环
  *             如果做双枪管则此函数不适用
  */
bool Revolver_Heat_Limit(void)
{
	static uint16_t  usShootNumAllow  = 0;
	static uint16_t  usHeatBuffer     = 0;
	static bool_t  IfShootAllow  =  FALSE;

	static  uint16_t  usShootNumBuffer  = 0;
	static  portTickType  ulShootTimeRecord = 0;
	static  uint16_t  usShootHeatRecord = 0;
	static  uint16_t  usShootNumPrev    = 0;
	static  uint16_t  usHeatPrev        = 0;

	static  uint32_t  ulOfflineCnt      = 0;
			uint16_t  usHeatReal		= 0;
			uint16_t  usShootNumReal	= 0;
			uint16_t  usHeatOneShoot	= 30;
			uint16_t  usHeatLimit;
	static  uint32_t  ShootNumBuffer_Error = 0;//计数长时间过大，认为出错，清零

	/* 读取热量 */
	usHeatReal = JUDGE_usGetRemoteHeat17();
	
	  
	  /* 离线处理 */
	if (usHeatReal == usHeatPrev)
	{
		
	}
	else
	{
		ulOfflineCnt = 0;
	}

	/* 读取射击次数 */
	usShootNumReal  =  JUDGE_usGetShootNum( );
	
	/* 只要打了弹就增加 */
	if (usShootNumReal > usShootNumPrev)
	{
		usShootNumBuffer  += usShootNumReal - usShootNumPrev;
		ulShootTimeRecord  = xTaskGetTickCount( );
		usShootHeatRecord  = usHeatReal;
	}
  	
	/* */
	usHeatOneShoot = Fric_GetHeatInc( );
	if(usHeatOneShoot <= 1)//防止出错，否则直接自瞄打弹不会动
	{
		usHeatOneShoot = 30;
	}
	usHeatLimit    = JUDGE_usGetHeatLimit( );
	if(usHeatLimit <= 30)
	{
		usHeatLimit = 240;//防止数据出错
	}

	/* 剩余热量 */
	if (usHeatReal <= usHeatLimit)
	{
		usHeatBuffer = usHeatLimit - usHeatReal;
	}
	else
	{
		usHeatBuffer = 0;
	}

	if (usHeatBuffer > usHeatOneShoot)//剩余热量大于打一发所需热量
	{
		/* 还能打出的子弹数目 */
		usShootNumAllow = (uint16_t)(usHeatBuffer / usHeatOneShoot);// - 1;
	}
	else
	{
		usShootNumAllow = 0;//剩余热量过低，不给打
	}


	/**/
	if ( abs(Revolver_Speed_Measure) <= 100	
			&& xTaskGetTickCount( ) - ulShootTimeRecord > TIME_STAMP_30MS/*TIME_STAMP_100MS*/ 
				&& (usHeatReal > usShootHeatRecord + 3 || usHeatReal == 0) )
	{
		/* 清零 */
		usShootNumBuffer = 0;
	}

	if (usShootNumAllow <= 1)
	{
		IfShootAllow = FALSE;
	}
	else if (usShootNumBuffer < usShootNumAllow)
	{
		IfShootAllow = TRUE;
	}
	else
	{
		IfShootAllow = FALSE;
	}
	
    /* 记录此时的射击次数 */  
	usShootNumPrev  =  usShootNumReal;
	usHeatPrev 		=  usHeatReal;

	if(usShootNumBuffer > 6)//防止计数出错，时间过长则清零
	{
		ShootNumBuffer_Error++;
	}
	else
	{
		ShootNumBuffer_Error = 0;
	}
	
	if(ShootNumBuffer_Error > TIME_STAMP_1000MS*5)//连续5秒大于6颗
	{
		usShootNumBuffer = 0;
		ShootNumBuffer_Error = 0;
	}
	
	
	if (ulOfflineCnt < 100)
	{
		return  IfShootAllow;
	}
	else
	{
		return  TRUE;	
	}

}


/*****************************卡弹处理**************************************/

/**
  * @brief  速度环式卡弹处理
  * @param  void
  * @retval void
  * @attention  卡住就反转n格
  */
void REVOL_SpeedStuck(void)
{
	static uint16_t  stuck_time    = 0;//卡弹计时
	static uint16_t  turnback_time = 0;//倒转计时
	static bool Revol_Speed_ifStuck = FALSE;//卡弹判断

	if (Revol_Speed_ifStuck == TRUE)//已确认卡弹,开始倒转计时
	{
		Revolver_Speed_Target = -4000;//倒转
		turnback_time++;//倒转一定时间

		if (turnback_time > Stuck_TurnBack_Time)//倒转完成
		{
			turnback_time  = 0;
			Revol_Speed_ifStuck = FALSE;//可以正转
		}			
	}
	else
	{
		if ( abs(Revolver_Final_Output) >= Stuck_Revol_PIDTerm //PID输出过大
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//速度过低
		{
			stuck_time++;//卡弹计时
		}
		else
		{
			stuck_time = 0;//没有长时间卡弹,及时清零
		}

		if (stuck_time > Stuck_SpeedPID_Time)//卡了超过60ms
		{
			Stuck_Speed_Sum++;//卡弹计数,给机械组的大兄弟点面子
			stuck_time = 0;
			Revol_Speed_ifStuck = TRUE;//标记可以进入倒转计时
		}
	}
}

/**
  * @brief  位置环式卡弹处理
  * @param  void
  * @retval void
  * @attention  卡住就反转n格
  */
void REVOL_PositStuck(void)
{
	static uint16_t  stuck_time      = 0;//卡弹计时
	static uint16_t  turnback_time   = 0;//倒转计时
	static bool Revol_Posit_ifStuck = FALSE;//卡弹判断
	
	if (Revol_Posit_ifStuck == TRUE)//卡弹后开始倒转计时
	{
		//卡弹了则在判断是否卡弹这段时间内鼠标按下的指令都清零
		Key_ShootNum = 0;
		
		turnback_time++;//倒转计时,1ms一次
		if (turnback_time > Stuck_TurnBack_Time)//倒转时间够了
		{
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum;//正常旋转,旋转回本来想要它转到的位置
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			Revol_Posit_ifStuck = FALSE;//认为此时不再卡弹了
			turnback_time = 0;//倒转时间清零,为下次倒转做准备	
		}
	}
	else
	{
		if ( abs(Revolver_Final_Output)  >= Stuck_Revol_PIDTerm //PID输出过大
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//速度过低
		{
			stuck_time++;//统计卡了多长时间
		}
		else
		{
			stuck_time = 0;//不卡了,时间清零
		}
		
		if (stuck_time > Stuck_SpeedPID_Time)//卡太久了,提示要倒转
		{
			//倒转不能放在Revol_Posit_ifStuck == TRUE中,否则就不是堵一次倒转1/2格了
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum - AN_BULLET ;//倒转 1格  
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			
			Stuck_Posit_Sum++;//卡弹计数,给机械组的大兄弟点面子
			stuck_time = 0;
			Revol_Posit_ifStuck = TRUE;//用来标记倒转计时开启	
		}
	}
}


/***************拨盘辅助函数***************/

/**
  * @brief  拨盘角度数据重置
  * @param  void
  * @retval void
  * @attention  弹仓每开启一次执行一次
  */
void Revol_Angle_Clear(void)
{
	Key_ShootNum = 0;//发弹指令清零
	
	//速度环位置重置
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Target_Sum;
	
}

/**
  * @brief  点射时间获取
  * @param  void
  * @retval 位置环实时指令时间
  * @attention  用于发射延时测试
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}
