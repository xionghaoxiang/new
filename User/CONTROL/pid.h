#ifndef __PID_H
#define	__PID_H

#include "stm32f10x.h"
#include "Header.h"
extern float Velocity_KP,Velocity_KI;					//增量式PI参数


extern float CCD_KP_Diff,CCD_KD_Diff,CCD_KI_Diff;



int Incremental_PI_Left(float Encoder,float Target);//左电机pid控制
int Incremental_PI_Right(float Encoder,float Target);//右电机pid控制

float CCD_PID(float Current_Value,float Target_Value );


#endif
