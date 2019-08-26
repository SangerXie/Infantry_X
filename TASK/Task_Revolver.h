#ifndef _TASK_REVOLVER_H
#define _TASK_REVOLVER_H

#include "system.h"


void REVOLVER_StopMotor(void);
void REVOLVER_InitArgument(void);
void REVOLVER_Rest(void);
void Revolver_Angle_Rest(void);

/*********拨盘总控制*************/
void Task_Revolver(void *pvParameters);

void REVOLVER_Rc_Ctrl(void);
bool REVOLVER_Rc_Switch(void);
/*******键盘模式************/
void REVOLVER_Key_Ctrl(void);

/******底盘键盘模式各类模式小函数*******/
void SHOOT_NORMAL_Ctrl(void);
void SHOOT_SINGLE_Ctrl(void);
void SHOOT_TRIPLE_Ctrl(void);
void SHOOT_HIGHTF_LOWS_Ctrl(void);
void SHOOT_MIDF_HIGHTS_Ctrl(void);
void SHOOT_AUTO_Ctrl(void);
void SHOOT_BUFF_Ctrl(void);
void SHOOT_BUFF_Ctrl_Gimbal(void);

void REVOLVER_KeySpeedCtrl(void);
void REVOLVER_KeyPosiCtrl(void);

void REVOLVER_CANbusCtrlMotor(void);

/****拨盘电机数据更新,CAN2中断中调用****/
void REVOLVER_UpdateMotorAngle( int16_t angle );
void REVOLVER_UpdateMotorSpeed( int16_t speed );
void REVOL_UpdateMotorAngleSum( void );

/*****PID控制*******/
void REVOL_SpeedLoop( void );
void REVOL_PositionLoop( void );

/***射频热量限制***/
bool Revolver_Heat_Limit(void);

/****卡弹处理*****/
void REVOL_SpeedStuck(void);
void REVOL_PositStuck(void);

/******拨盘辅助函数******/
void Revol_Angle_Clear(void);
portTickType REVOL_uiGetRevolTime(void);

#endif
