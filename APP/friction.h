#ifndef _FRICTION_H
#define _FRICTION_H

#include "system.h"

void FRICTION_StopMotor(void);

/*********Ä¦²ÁÂÖÖ÷¿Ø************/
void FRICTION_Ctrl( void );
bool FRIC_RcSwitch( void );
void FRIC_KeyLevel_Ctrl(void);

/***Ä¦²ÁÂÖÆô¶¯ÔÆÌ¨Ì§Í·ÅÐ¶Ïº¯Êý***/
uint8_t FRIC_IfWait( void );
uint8_t FRIC_IfOpen( void );

/****Ä¦²ÁÂÖ¸¨Öúº¯Êý*****/
void Friction_Ramp(void);
uint16_t Fric_GetHeatInc(void);
float Fric_GetSpeedReal(void);

#endif

