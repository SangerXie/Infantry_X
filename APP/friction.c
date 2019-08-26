#include "friction.h"

#include "control.h"
#include "pwm.h"
#include "friction.h"
#include "remote.h"
#include "Task_Gimbal.h"
#include "laser.h"
#include "judge.h"
#include "vision.h"
#include "magazine.h"

/*****************高射频必须降低射速，拨弹必须和摩擦轮配合***********************/

//此文件有很多不太合理的地方,只是为了做测试
/**************************摩擦轮控制***********************************/


//[0, 13~13.5, 17.5~18, 22~23, 根据弹道选择, 哨兵27]
//摩擦轮速度选择,影响子弹速度,只是测试,系数要后面定
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
/*snail*/
//	float Friction_PWM_Output[6]     = {0, 450, 450, 550, 550, 600};//关闭  低速  中速  高速  狂暴,打符650，速度25.5m/s
//550->23.5~25.5   450->20~21

///*小蜜蜂*/
	float Friction_PWM_Output[6]     = {0, 480, 550, 650, 730/*730*/, 730};//关闭  低速  中速  高速  狂暴  哨兵
//600->23.7~24.1    630->24~25   550->20~21    450->12.5~13.5
//710会干扰陀螺仪角加速度读取	

/*好盈窄版40A*/
//	float Friction_PWM_Output[6]     = {0, 500, 600, 700, 710, 730};//关闭  低速  中速  高速  狂暴,打符650，速度25.5m/s
////600->21    650->22~23   700->23.5~24.5    750->25~27

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	float Friction_PWM_Output[6]     = {0, 450, 550, 650, 720/*685*//*690*/, 730};//关闭  低速  中速  高速  狂暴
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	float Friction_PWM_Output[6]     = {0, 455, 510, 598/*570*/, 695, 685};//关闭  低速  中速  高速  狂暴  哨兵
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE    //600抖
	float Friction_PWM_Output[6]     = {0, 465, 520, 592, 715/*675*/, 675};//关闭  低速  中速  高速  狂暴  哨兵
	//打符27 哨兵25.5
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	float Friction_PWM_Output[6]     = {0, 460, 505, 583, 695, 685};//关闭  低速  中速  高速  狂暴  哨兵
	
#endif

//摩擦轮不同pwm下对应的热量增加值(射速),最好比实际值高5
uint16_t Friction_PWM_HeatInc[5] = {0,  20,  26,  34,  36};//测试时随便定的速度,后面测试更改


//遥控模式下的一些标志位
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2

//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1		//Z键射速
#define FRI_MID  	2		//B键射速
#define FRI_HIGH 	3		//左键
#define FRI_MAD  	4		//打符射速
#define FRI_SENTRY  5		//哨兵射速

//速度等级选择
uint16_t Fric_Speed_Level;

//遥控模式下的开启标志位
uint8_t Friction_Switch = 0;//与弹仓开关判断类似

//摩擦轮目标速度
float Friction_Speed_Target;

//摩擦轮等级目标转速
float Frict_Speed_Level_Target;

//摩擦轮实际输出速度,用来做斜坡输出
float Friction_Speed_Real;




/**
  * @brief  摩擦轮失控保护
  * @param  void
  * @retval void
  * @attention 缓慢减小PWM输出,否则会报警
  */
void FRICTION_StopMotor(void)
{
	Friction_Speed_Target = 0;

	if (Friction_Speed_Real > 0)
	{
		Friction_Speed_Real -= 1;//3
	}

	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}

	TIM4_FrictionPwmOutp( Friction_Speed_Real, Friction_Speed_Real );
}

/************************摩擦轮总控制*****************************/

/**
  * @brief  摩擦轮控制函数
  * @param  void
  * @retval void
  * @attention 摩擦轮停止转动,红外关闭,摩擦轮从关闭到开启,云台抬头归中
  */
float fric_rc_debug = 300;
void FRICTION_Ctrl( void )
{
//	uint8_t level;//当前机器人等级
	
	if(SYSTEM_GetSystemState( ) == SYSTEM_STARTING)
	{
		Fric_Speed_Level = FRI_OFF;
		Friction_Speed_Target = 0;
		Friction_Speed_Real   = 0;
		Laser_Off;
	}
	else
	{
		if (SYSTEM_GetRemoteMode( ) == RC)//遥控模式
		{
			Fric_Speed_Level = FRI_LOW;//遥控模式下的速度选择，低射速，方便检录发光弹
			
			if (FRIC_RcSwitch( ) == TRUE)//判断状态切换,跟弹仓开关逻辑相同
			{	//切换为关
				if (Friction_Speed_Target > Friction_PWM_Output[FRI_OFF])
				{
					Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];	
				}
				else//切换为开
				{
					Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];//摩擦轮目标值大于0,标志着遥控模式下开启,告诉pitch要抬头
				}
			}
			else
			{
				if(Friction_Speed_Target > Friction_PWM_Output[Fric_Speed_Level])
				{
					Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];
				}
			}
		}
		else				//键盘模式,可调射速
		{
//			level = JUDGE_ucGetRobotLevel();//读取等级
			FRIC_KeyLevel_Ctrl();
		}
	}
	
	//摩擦轮输出斜坡,注意要先抬头才能进入斜坡
	Friction_Ramp();
	
	//激光开关
	if(Friction_Speed_Real > 0 && GIMBAL_IfBuffHit() == FALSE && Magazine_IfOpen() == FALSE)//用实际输出合理一点
	{
		Laser_On;
	}
	else
	{
		Laser_Off;
		//Laser_On;
	}
	
	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
}

/**
  * @brief  摩擦轮遥控控制
  * @param  void
  * @retval 是否转换当前状态
  * @attention 跟弹仓开关逻辑相同
  */
bool FRIC_RcSwitch( void )
{
	if (IF_RC_SW2_DOWN)
	{
		if (IF_RC_SW1_UP)
		{
			if (Friction_Switch == FRIC_STEP1)
			{
				Friction_Switch = FRIC_STEP2;
			}
			else if (Friction_Switch == FRIC_STEP2)
			{
				Friction_Switch = FRIC_STEP0;
			}
		}
		else
		{
			Friction_Switch = FRIC_STEP1;
		}
	}
	else
	{
		Friction_Switch = FRIC_STEP0;
	}


	if (Friction_Switch == FRIC_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  摩擦轮等级控制,键盘专用,根据等级自动设置射速
  * @param  当前等级
  * @retval void
  * @attention 键盘模式下不关摩擦轮
  */
float debug_fric = 450;//760;//650;
float Fric_Dist_Far = 3;//大于此距离射速拉满
float Fric_Dist_Near = 1.2;//小于此距离射速最低
void FRIC_KeyLevel_Ctrl(void)
{
	static int16_t fric_auto_delay = 0;//防止闪灭频繁换速
	float fric_percent = 1;
	float Fric_Dist_Now = 5;
	
	Fric_Dist_Now = VisionRecvData.distance/100;
	
	if (GIMBAL_IfBuffHit( ) == TRUE)
	{
		Fric_Speed_Level = FRI_MAD;//打符模式,最高射速
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
	else if (IF_MOUSE_PRESSED_LEFT && GIMBAL_IfAutoHit() == FALSE)//非自瞄，正常打击
	{
		Fric_Speed_Level = FRI_HIGH;//一般用高射速
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
	else if (IF_KEY_PRESSED_Z)//推家射速
	{
		Fric_Speed_Level = FRI_LOW;				
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择		
	}
	else if (IF_KEY_PRESSED_B)
	{
		Fric_Speed_Level = FRI_MID;//高射频低射速模式,推家肉搏用
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}
	else if (GIMBAL_IfAutoHit() == TRUE)//自瞄模式
	{
		if(VisionRecvData.identify_target == TRUE)
		{
			fric_auto_delay = 0;
			
			fric_percent = (Fric_Dist_Now - Fric_Dist_Near) / (Fric_Dist_Far - Fric_Dist_Near);
			fric_percent = constrain_float(fric_percent, 0, 1);
			
			Frict_Speed_Level_Target = Friction_PWM_Output[FRI_LOW] 
										+ (Friction_PWM_Output[FRI_HIGH] - Friction_PWM_Output[FRI_LOW]) * fric_percent;
		}
		else//没识别到，过一段时间后再切换速度
		{
			fric_auto_delay++;
			if(fric_auto_delay >= 20)//连续200ms没识别到
			{
				fric_auto_delay = 200;//防止溢出
				Fric_Speed_Level = FRI_HIGH;//一般用高射速
				Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
			}
		}
		
		if(GIMBAL_AUTO_PITCH_SB() == TRUE)//自瞄打哨兵，射速提高
		{
			Fric_Speed_Level = FRI_SENTRY;
			Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
		}
	}
	else//防止出错
	{
		Fric_Speed_Level = FRI_HIGH;	
		Frict_Speed_Level_Target = Friction_PWM_Output[Fric_Speed_Level];//射速选择
	}

	if( IF_MOUSE_PRESSED_LEFT || Friction_Speed_Target>0 )//不用中间量处理会有BUG——抬头会自动开枪
	{
		//Friction_Speed_Target = debug_fric;
		Friction_Speed_Target = Frict_Speed_Level_Target;
	}
}


/***********摩擦轮启动云台抬头判断函数*************/

/**
  * @brief  摩擦轮是否已经开启
  * @param  void
  * @retval TRUE已开启   FALSE未开启
  * @attention 
  */
uint8_t FRIC_IfWait( void )
{
    if (Friction_Speed_Target > 0)//通过改变摩擦轮目标值来标志遥控模式下pitch是否要抬头
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  摩擦轮是否正在开启
  * @param  void
  * @retval TRUE正在开启   FALSE开启到目标速度
  * @attention 正在开启不准改变云台PITCH
  */
uint8_t FRIC_IfOpen( void )
{
	static  uint8_t  status = FALSE;
	static uint32_t  ulWait = 0;

	static portTickType ulCurrent = 0;

	ulCurrent = xTaskGetTickCount( );
	
    if (Friction_Speed_Real > 0)
	{
		if (ulCurrent >= ulWait + 1500)//抬头时长,1.5S
		{
			status = TRUE;
		}
	}
	else
	{
		ulWait = ulCurrent;
		
		status = FALSE;
	}
	
	return status;
}

/*************摩擦轮辅助函数****************/

/**
  * @brief  摩擦轮输出斜坡
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//开启
	{
		Friction_Speed_Real += 5;
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//关闭
	{
		Friction_Speed_Real -= 5;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
}

/**
  * @brief  当前PWM对应单发子弹热量增加值(射速)
  * @param  void
  * @retval 当前热量递增值
  * @attention 不适用于42mm,可用于客户端数据显示
  */
uint16_t Fric_GetHeatInc(void)
{
	if(GIMBAL_IfAutoHit() == TRUE && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
	{
		return JUDGE_usGetSpeedHeat17()*1.4f;
	}
	else
	{
		return Friction_PWM_HeatInc[Fric_Speed_Level];
	}
}

/**
  * @brief  获取当前摩擦轮PWM输出值
  * @param  void
  * @retval 实际PWM值
  * @attention 用来禁止摩擦轮速度过低的情况下拨盘的转动
  */
float Fric_GetSpeedReal(void)
{
	return Friction_Speed_Real;
}


