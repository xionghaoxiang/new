#ifndef __KEY_H
#define	__KEY_H


#include "stm32f10x.h"
#include "Header.h"

//  ���Ŷ���
#define    KEY1_GPIO_CLK     RCC_APB2Periph_GPIOA
#define    KEY1_GPIO_PORT    GPIOA			   
#define    KEY1_GPIO_PIN	 GPIO_Pin_0

#define    KEY2_GPIO_CLK     RCC_APB2Periph_GPIOC
#define    KEY2_GPIO_PORT    GPIOC		   
#define    KEY2_GPIO_PIN     GPIO_Pin_13



#define KEY2_STATE  		 PCin(13)

 /** �������±��ú�
	*  ��������Ϊ�ߵ�ƽ������ KEY_ON=1�� KEY_OFF=0
	*  ����������Ϊ�͵�ƽ���Ѻ����ó�KEY_ON=0 ��KEY_OFF=1 ����
	*/
#define KEY_ON	1
#define KEY_OFF	0


//�û���������ֵ״̬
#define No_Action 					0
#define Click 						1
#define Long_Press 					2
#define Double_Click				3



void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
uint8_t User_Key_Scan(void);
void Get_KeyVal(void);
static uint8_t Target_Laps;   // 目标圈数，默认1圈
static uint8_t Target_Turns = 0;   // 目标圈数，默认1圈


#endif /* __KEY_H */

