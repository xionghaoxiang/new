#include "./Led/bsp_led.h"   

/**************************************************************************
Function: LED_GPIO_Config
Input   : none
Output  : none
函数功能：LED配置
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd( LED1_GPIO_CLK | LED2_GPIO_CLK , ENABLE);
	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;

	/*调用库函数，初始化GPIO*/
	GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);


	/* 关闭所有led灯	*/
	GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);

	/* 关闭所有led灯	*/
	GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);	 
    
}

/**************************************************************************
Function: LED_Flash
Input   : Indicates the count of frequencies
Output  : none
函数功能：LED闪烁频率
入口参数: 指示频率的计数 
返回  值：无
**************************************************************************/	 	
//在中断函数调用
void LED_Flash(u16 count)
{
	static int count_time;
	if(0 == count)
	{
		LED1_OFF;
	}
	else if(++count_time >= count)
	{
		LED1_TOGGLE;
		count_time = 0;	
	}
}

