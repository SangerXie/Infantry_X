#include "control.h"

#include "remote.h"
#include "laser.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Revolver.h"

#include "magazine.h"
#include "friction.h"
#include "super_cap.h"
#include "adda.h"

/**********************系统控制判断及保护******************/


//控制模式
eRemoteMode remoteMode = RC;

//系统状态
eSystemState systemState = SYSTEM_STARTING;


/**
  * @brief  系统重置
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}

/**
  * @brief  失控保护
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_OutCtrlProtect(void)
{
    SYSTEM_Reset();//系统恢复至重启状态
	REMOTE_vResetData();//遥控数据恢复至默认状态
	
	
	Laser_Off;//激光关
	CHASSIS_StopMotor();//底盘关
	GIMBAL_StopMotor();//云台关
	Magazine_StopCtrl();//舵机停止转动
	REVOLVER_StopMotor();//拨盘停止转动
	FRICTION_StopMotor();//摩擦轮关
	Super_Cap_StopCtrl();//电容关闭充放电
}

/**
  * @brief  更新系统状态
  * @param  void
  * @retval void
  * @attention 1kHz,在LOOP循环调用
  */
void SYSTEM_UpdateSystemState(void)
{
	static uint32_t  ulInitCnt  =  0;
	
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;

		if (ulInitCnt > 2500)//启动延时,1ms*2k=2s,为了给MPU启动时间
		{
			ulInitCnt = 0;

			systemState = SYSTEM_RUNNING;//启动完成,转换成普通模式
		}
	}
}

/**
  * @brief  拨杆模式选择
  * @param  void
  * @retval void
  * @attention 键盘或鼠标,可在此自定义模式选择方式,1ms执行一次
  */
void SYSTEM_UpdateRemoteMode( void )
{ 
    if (IF_RC_SW2_UP)
	{
		remoteMode = KEY;
	}
	
	else
	{
		remoteMode = RC;
	}
}


//返回控制模式
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//返回系统状态
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}




