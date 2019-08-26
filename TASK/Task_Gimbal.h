#ifndef _TASK_GIMBAL_H
#define _TASK_GIMBAL_H

#include "system.h"


#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

#define NOW  0
#define LAST 1

/*       临界值处理结构体        */
typedef struct 
{

	float LastAngle;       //上一次读取的角度
	float CurAngle;	//当前读取的角度
	float AngleSum;	//角度累加值
	
}Critical_t;

typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;

void GIMBAL_InitArgument(void);
void GIMBAL_kPID_Init(void);
void GIMBAL_StopMotor(void);
void GIMBAL_InitCtrl(void);

/***********云台总控制,loop中调用************/
void Task_Gimbal(void *pvParameters);
void GIMBAL_Rc_Ctrl(void);
void GIMBAL_Key_Ctrl(void);
void Gimbal_Chass_Separ_Limit(void);

/***********云台键盘模式各类模式小函数*************/
void GIMBAL_NORMAL_Mode_Ctrl(void);
void GIMBAL_HIGH_Mode_Ctrl(void);
void GIMBAL_LEVEL_Mode_Ctrl(void);
void GIMBAL_AUTO_Mode_Ctrl(void);
void GIMBAL_BUFF_Mode_Ctrl_Chassis(void);
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);
void GIMBAL_BASE_Mode_Ctrl(void);
void GIMBAL_MANUAL_Mode_Ctrl(void);

/************云台测量值更新及发送****************/
void GIMBAL_UpdateAngle( char eAxis, int16_t angle );
void GIMBAL_UpdatePalstance(void);
void GIMBAL_CanbusCtrlMotors(void);

/*****************************云台位置PID控制***********************************/
void GIMBAL_PositionLoop(void);
void vPitch_Mech_PositionLoop(void);
void vPitch_Gyro_PositionLoop(void);
void vYaw_Mech_PositionLoop(void);
void vYaw_Gyro_PositionLoop(void);


/****************************辅助函数**************************************/
void Critical_Handle_Init(Critical_t *critical, float get);
float Gimbal_Yaw_Gryo_AngleSum(Critical_t *critical, float get);
int16_t GIMBAL_GetOffsetAngle(void);//计算YAW中心偏差
uint8_t GIMBAL_IfPitchLevel(void);//判断Pitch是否回到水平位置
uint8_t GIMBAL_IfPitchHigh(void);//云台是否抬头
bool GIMBAL_IfGIMBAL_LEVEL(void);
//视觉
bool GIMBAL_IfBuffHit(void);//打符
bool GIMBAL_If_Big_Buff(void);//大符
bool GIMBAL_If_Small_Buff(void);//小符
bool GIMBAL_If_Base(void);
bool GIMBAL_IfManulHit(void);//手动打符
bool GIMBAL_IfAutoHit(void);//自瞄
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
bool Gimb_If_Small_Top(float angle);
bool GIMBAL_IfAuto_MobPre_Yaw(void);
bool GIMBAL_MOBPRE_YAW_FIRE(void);

bool GIMBAL_BUFF_YAW_READY(void);
bool GIMBAL_BUFF_PITCH_READY(void);
bool GIMBAL_AUTO_PITCH_SB(void);
bool GIMBAL_AUTO_PITCH_SB_SK(void);
float GIMBAL_PITCH_Judge_Angle(void);
int Base_Angle_Measure(void);

#endif
