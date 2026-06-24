#ifndef __SHOW_H
#define	__SHOW_H

#include "stm32f10x.h"
#include "./OLED/bsp_oled.h"


#define CHANNEL_NUMBER    3				//示波器通道数，更改这个显示通道数


void Show(void);


u16 myabs(int Input);
void DataScope(void);
void APP_Show(void);


#endif
