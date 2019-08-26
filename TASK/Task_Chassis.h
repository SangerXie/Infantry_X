#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H

#include "system.h"


//底盘模式选择
typedef enum
{
	CHASSIS_MECH_MODE = 0,//机械
	CHASSIS_GYRO_MODE = 1,//陀螺仪,底盘跟随云台
	
} eChassisCtrlMode;


//键盘模式时,底盘状态选择
typedef enum
{
	CHASSIS_NORMAL   = 0,//普通模式,正常前进
	CHASSIS_CORGI    = 1,//扭屁股模式
	CHASSIS_ROSHAN   = 2,//打符模式
	CHASSIS_SLOW     = 3,//补弹低速模式
	CHASSIS_SZUPUP   = 4,//爬坡模式
	CHASSIS_MISS     = 5,//自动闪避模式
	CHASSIS_PISA     = 6,//45°模式
	
} eChassisAction;

//底盘电机ID
typedef enum
{
	LEFT_FRON_201 = 0,  // 左前
	RIGH_FRON_202 = 1,  // 右前
	LEFT_BACK_203 = 2,  // 左后
	RIGH_BACK_204 = 3,  // 右后
	
}eChassisWheel;

//转速测量
typedef enum
{
	Chassis_ANGLE = 0,  // 机械角度
	Chassis_SPEED = 1,  // 转速
	
}eChassis_MotorReturnValueType;


void CHASSIS_InitArgument(void);
void CHASSIS_StopMotor(void);
void CHASSIS_REST(void);

/************************底盘总控制,loop中调用***************/
void Task_Chassis(void *pvParameters);
void CHAS_Rc_Ctrl(void);
void CHAS_Key_Ctrl(void);

/*******************底盘键盘模式各类模式小函数*******************/
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );
void Chassis_NORMAL_Mode_Ctrl(void);
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_SZUPUP_Mode_Ctrl(void);
void CHASSIS_MISS_Mode_Ctrl(void);
void CHASSIS_PISA_Mode_Ctrl(void);

/********************底盘电机数据更新及发送*****************************/
void CHASSIS_UpdateMotorAngle( eChassisWheel eWheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( eChassisWheel eWheel, int16_t speed );
void CHASSIS_UpdateMotorCur( eChassisWheel eWheel, int16_t current );

void CHASSIS_CANbusCtrlMotor(void);

/**********************底盘输出计算**************************/
void Chassis_Omni_Move_Calculate(void);
void Chassis_Motor_Speed_PID( eChassisWheel eWheel );
void Chassis_MotorOutput(void);
float Chassis_Z_Speed_PID(void);

/*****************底盘功率*************************/
void Chassis_Power_Limit(void);

/**************键盘模式辅助函数********************/
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
bool CHASSIS_IfActiveMode(void);

bool Chassis_IfSZUPUP(void);
bool Chassis_IfCORGI(void);
bool Chassis_IfPISA(void);

/************底盘数据接口***************/

float Chassis_All_Speed(void);
float Chassis_All_Speed_Target(void);

float Get_Chass_X(void);
float Get_Chass_Y(void);
float Get_Chass_Z(void);

float Chassis_Z_Corgi(float get, float set);
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);

#endif
