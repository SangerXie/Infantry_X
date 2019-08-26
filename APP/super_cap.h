#ifndef _SUPER_CAP_H
#define _SUPER_CAP_H

#include "system.h"


//void CAP_Ctrl(void);
//void CAP_Ctrl_Charge_On(void);

//void CAP_Ctrl_Charge_Off(void);
//void CAP_Ctrl_Output(void);
//void Super_Cap_StopCtrl(void);
//void Cap_Current_PID(void);

//bool Cap_Out_Can_Open(void);









void CAP_Ctrl(void);
void SuperCap_PID_Parameter_Init(void);
void Super_Cap_StopCtrl(void);
void Get_SuperCap_Mode(void);
void SuperCap_Power_PID(void);
void Super_Charging_Control(void);
void SuperCap_Giveout_Control(void);

void SuperCap_Out(void);
void SuperCap_In(void);

bool Cap_Out_Can_Open(void);

#endif
