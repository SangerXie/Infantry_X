#ifndef _MAGAZINE_H
#define _MAGAZINE_H

#include "system.h"

void Magazine_StopCtrl(void);

void Magazine_Ctrl(void);
bool Magezine_Rc_Switch(void);
void Magazine_Key_Ctrl(void);
void Magazine_Servo(int16_t pwm);

/*******************µ¯²Ö¸¨Öúº¯Êý*************************/
bool Magazine_IfOpen(void);
bool Magazine_IfWait(void);


bool Senty_Run(void);
void Senty_Run_Clean_Flag(void);


#endif
