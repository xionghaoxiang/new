#ifndef __KEY_H
#define	__KEY_H


#include "stm32f10x.h"
#include "Header.h"

//  引脚定义
#define    KEY1_GPIO_CLK     RCC_APB2Periph_GPIOA
#define    KEY1_GPIO_PORT    GPIOA			   
#define    KEY1_GPIO_PIN	 GPIO_Pin_0

#define    KEY2_GPIO_CLK     RCC_APB2Periph_GPIOC
#define    KEY2_GPIO_PORT    GPIOC		   
#define    KEY2_GPIO_PIN     GPIO_Pin_13



#define KEY2_STATE  		 PCin(13)

 /** 按键按下标置宏
	*  按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
	*  若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
	*/
#define KEY_ON	1
#define KEY_OFF	0


//用户按键返回值状态
#define No_Action 					0
#define Click 						1
#define Long_Press 					2
#define Double_Click				3



void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
uint8_t User_Key_Scan(void);
void Get_KeyVal(void);


#endif /* __KEY_H */

