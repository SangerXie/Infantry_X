#include "magazine.h"

#include "control.h"
#include "remote.h"
#include "judge.h"
#include "Task_Revolver.h"
#include "Task_Gimbal.h"

/* 弹仓pwm --> 23~127范围可动 */ 


/*--------------------------------------------------------*/
//弹仓开关角度对应的PWM值
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
			#define Magazine_Close_Angle   53
			#define Magazine_Open_Angle    75//105
			
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
			#define Magazine_Close_Angle   25
			#define Magazine_Open_Angle    63
			
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
			#define Magazine_Close_Angle   88
			#define Magazine_Open_Angle    50
			
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
			#define Magazine_Close_Angle   88
			#define Magazine_Open_Angle    50
			
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
			#define Magazine_Close_Angle   89
			#define Magazine_Open_Angle    52
			
#endif
/*--------------------------------------------------------*/

//弹仓开关标志位
#define MAGA_STEP0    0		//失能标志
#define MAGA_STEP1    1		//SW1复位标志
#define MAGA_STEP2    2		//弹仓开关标志

#define MAGA_KEY_CLOSE    0		//弹仓键盘模式开关标志位
#define MAGA_KEY_OPEN     1		//弹仓键盘模式开关标志位
uint8_t	Magazine_Switch = 0;//弹仓遥控模式开关标志位转换
uint8_t Magazine_Key_Switch = 0;//弹仓键盘模式开关标志位转换

int16_t Magazine_Target;//PWM目标值
int16_t Magazine_Actual;//PWM真实值
int16_t Magazine_ServoRamp = 100;//弹仓斜坡,控制变化速度,实测好像没什么用


u8 Maga_Switch_R = 1;
u8 Maga_Key_R_Change = 0;
u8 Maga_Times = 0;


bool Senty_Run_Flag = FALSE;

/**
  * @brief  弹仓舵机停止转动
  * @param  void
  * @retval void
  * @attention 失控保护,置0弹仓不动,此时可手动打开弹仓
  */
void Magazine_StopCtrl(void)
{
	Magazine_Servo(0);
}

/******************弹仓控制,loop循环调用*************************/

/**
  * @brief  弹仓舵机控制
  * @param  void
  * @retval void
  * @attention 
  */
uint8_t maga_remot_change = TRUE;
void Magazine_Ctrl(void)
{
	if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//舵机角度系统初始化
	{
		//角度初始化,使目标值与测量值都为关闭值
		Magazine_Target = Magazine_Close_Angle;
		Magazine_Actual = Magazine_Close_Angle;
		Maga_Switch_R = 1;
		Maga_Key_R_Change = 0;
		Maga_Times = 0;
	}
	else
	{
		if (Magezine_Rc_Switch( ) == TRUE)//判断是否要弹仓改变当前状态
		{
			//改变当前状态的判断
			if (Magazine_Target == Magazine_Open_Angle)//弹仓开启此处始终进入
			{
				Magazine_Target = Magazine_Close_Angle;//若之前打开,则现在关闭
			}
			else			//在弹仓关闭之后此处始终进入
			{
				Magazine_Target = Magazine_Open_Angle;//若之前关闭,则现在打开
			}
		}
				
		if (SYSTEM_GetRemoteMode() == KEY)
		{
			Magazine_Key_Ctrl();
		}
		else
		{
			Magazine_Key_Switch = MAGA_KEY_CLOSE;
			Maga_Switch_R = 1;
			Maga_Key_R_Change = 0;
			Maga_Times = 0;
			maga_remot_change = TRUE;//标记切回了遥控模式
		}
	}
	

	//使舵机实际值逐步逼近目标值,斜坡输出
	if (Magazine_Actual < Magazine_Target)
	{
		Magazine_Actual += Magazine_ServoRamp;
		Magazine_Actual = constrain_int16_t( Magazine_Actual, Magazine_Actual, Magazine_Target );
	}
	else if (Magazine_Actual > Magazine_Target)
	{
		Magazine_Actual -= Magazine_ServoRamp;
		Magazine_Actual = constrain_int16_t( Magazine_Actual, Magazine_Target, Magazine_Actual );
	}
	

	Magazine_Servo(Magazine_Actual);
	
	
	/*********************/
	if(IF_KEY_PRESSED_X && IF_KEY_PRESSED_G)
	{
		Senty_Run_Flag = TRUE;
	}
}

/**
  * @brief  遥控模式,判断是否下达了状态转换指令,进入一次之后立刻变成FALSE
  * @param  void
  * @retval 是否下达了改变状态的指令
  * @attention 逻辑较复杂,好好想想
  */
bool Magezine_Rc_Switch(void)
{
	if (IF_RC_SW2_MID)//遥控模式
	{
		if (IF_RC_SW1_UP)//开启弹仓条件1
		{
			if (Magazine_Switch == MAGA_STEP1)//开启弹仓条件2
			{
				Magazine_Switch = MAGA_STEP2;
			}
			else if (Magazine_Switch == MAGA_STEP2)//弹仓关闭
			{
				Magazine_Switch = MAGA_STEP0;//切断联系
			}
		}
		else		//标志SW1是否有复位的情况,在复位的情况下才能再次进入STERP2
		{
			Magazine_Switch = MAGA_STEP1;//保障SW1在下次变换之前一直不能用
		}
	}
	else//s2不在中间,不允许弹仓开启
	{
		Magazine_Switch = MAGA_STEP0;//可能是摩擦轮开启也可能是切换成键盘模式
	}
	
	
	if (Magazine_Switch == MAGA_STEP2)
	{
		return TRUE;//只有SW1重新变换的时候才为TRUE
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  键盘模式
  * @param  void
  * @retval void
  * @attention 
  */
void Magazine_Key_Ctrl(void)
{
	static uint32_t ulTimePressS   = 0;
//	static uint32_t ulTimeOpen     = 0;
	       portTickType ulTimeCurrent  = 0;
	static uint32_t PressR_Gap = 0;//关弹仓情况下，R按一下之后长时间不再按一次则忽略此次
	
	if(maga_remot_change == TRUE)//刚从遥控模式切过来
	{
		Magazine_Key_Switch = MAGA_KEY_CLOSE;
		Magazine_Target = Magazine_Close_Angle;
		maga_remot_change = FALSE;
	}
	
	ulTimeCurrent = xTaskGetTickCount();
	
	switch (Magazine_Key_Switch)
	{
		case MAGA_KEY_CLOSE:	
			if(!IF_KEY_PRESSED_R)//R松开
			{
				Maga_Switch_R = 1;
				if(ulTimeCurrent - PressR_Gap > TIME_STAMP_500MS)//500ms内没按下R
				{
					Maga_Times = 0;//重新记次
				}
			}
			
			if (IF_KEY_PRESSED_R && Maga_Switch_R == 1
					&& GIMBAL_IfBuffHit() != TRUE)//R按下
			{
				PressR_Gap = ulTimeCurrent;//记录按下时间
				Maga_Switch_R = 0;	
				Maga_Times++;	
			}	
			
			if(Maga_Times >= 2)//500ms内连按2次
			{
				Magazine_Key_Switch = MAGA_KEY_OPEN;//开弹仓
				Magazine_Target = Magazine_Open_Angle;
				if(JUDGE_usGetShootNum()>0)
				{
					JUDGE_ShootNum_Clear();//发弹量清零
					Revol_Angle_Clear();//拨盘角度清零
				}
				ulTimePressS = ulTimeCurrent;
			}
			else
			{
				Magazine_Target = Magazine_Close_Angle;
			}
		break;
				
		case MAGA_KEY_OPEN:	
			if(!IF_KEY_PRESSED_R)//R松开
			{
				Maga_Switch_R = 1;
			}
			
			if (!IF_KEY_PRESSED_S)
			{
				ulTimePressS = ulTimeCurrent;//刷新S按下的时间
			}
			
			if ( ulTimeCurrent - ulTimePressS >  (TIME_STAMP_500MS + TIME_STAMP_300MS)  //连按S超过800ms
						|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V )	
			{
				Magazine_Key_Switch = MAGA_KEY_CLOSE;
			}
			else
			{
				Magazine_Target = Magazine_Open_Angle;
			}
			Maga_Times = 0;
		break;		
	}
	
}

/**************弹仓键盘模式各类小函数****************/

/**
  * @brief  弹仓舵机伺服
  * @param  目标PWM值
  * @retval void
  * @attention 28最小
  */
void Magazine_Servo(int16_t pwm)
{
	pwm = abs(pwm);

	TIM1->CCR2 = pwm;
}


/*******************弹仓辅助函数*************************/

/**
  * @brief  弹仓是否已经打开完成
  * @param  void
  * @retval TRUE打开,FALSE未打开
  * @attention 
  */
bool Magazine_IfOpen(void)
{
	if (Magazine_Actual == Magazine_Open_Angle)//用实际输出判断
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  弹仓是否正在打开
  * @param  void
  * @retval TRUE打开中,false未打开
  * @attention 
  */
bool Magazine_IfWait(void)
{
	if (Magazine_Target == Magazine_Open_Angle)//用目标输出判断
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/****************控制哨兵按键判断，高刷新率，防止检测不到按键*********************/

bool Senty_Run(void)
{
	return Senty_Run_Flag;
}

void Senty_Run_Clean_Flag(void)
{
	Senty_Run_Flag = FALSE;
}

