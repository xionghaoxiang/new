#include "Header.h"

uint8_t startflag = 0;
volatile u8 delay_50,delay_flag; 		//延时变量			
u16 Voltage;							//电压变量，放大100倍储存
u8 Flag_Stop;							//电机启停标志位
u8 PS2_ON_Flag = 0,APP_ON_Flag = 0,ROS_ON_Flag = 0,Remote_ON_Flag;		//默认所有方式不控制
u8 Car_Num=Diff_Car;								//车型号码选择
u8 Flag_Show = 1;						//显示标志位，默认开启，长按切换到上位机模式，此时关闭
float Perimeter; 						//轮子的周长
float Wheelspacing; 					//轮子的轮距
u16 DISTANCE=0,ANGLE=0;
float test1,test2;
//volatile uint32_t sys_ms = 0;   // 毫秒计数器
uint32_t brake_timer = 0;
uint8_t brake_done = 0;
uint8_t racing_state1=2;		//racing_state的平替

void USART2_DisableAllInterrupts(void)
{
    // 1. 清除串口控制寄存器的所有中断使能位
    USART2->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_PEIE);
    USART2->CR2 &= ~USART_CR2_LBDIE;    // LIN中断
    USART2->CR3 &= ~(USART_CR3_EIE | USART_CR3_CTSIE);  // 错误中断和CTS中断
    
    // 2. 在NVIC中禁用USART1中断
    NVIC_DisableIRQ(USART2_IRQn);
    
    // 3. 可选：清除所有挂起的中断标志
    USART2->SR = 0;
}

/**
 * @brief  完全打开USART1的所有中断
 * @note   这个函数与USART1_DisableAllInterrupts()完全对称
 */
void USART2_EnableAllInterrupts(void)
{
    // 1. 在NVIC中启用USART1中断（必须先启用，否则中断无法触发）
    NVIC_EnableIRQ(USART2_IRQn);
    
    // 2. 启用串口控制寄存器的所有中断使能位
    // 启用CR1寄存器中的中断
    USART2->CR1 |= (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_PEIE);
    
    // 启用CR2寄存器中的LIN中断
    USART2->CR2 |= USART_CR2_LBDIE;    // LIN中断
    
    // 启用CR3寄存器中的错误中断和CTS中断
    USART2->CR3 |= (USART_CR3_EIE | USART_CR3_CTSIE);  // 错误中断和CTS中断
    
    // 3. 可选：清除所有挂起的中断标志（确保开始时没有误触发）
    USART2->SR = 0;
}
/**************************************************************************  
Function: Main function
Input   : none
Output  : none
函数功能：主函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
int main(void)
{	
	u32 Voltage_Sum = 0;
	u8 Voltage_Count = 0;				//电压计算相关变量
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级分组
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	LED_GPIO_Config();					//LED初始化，PC2和PC3
	Key_GPIO_Config();					//按键初始化，PA0和PC13
	BEEP_GPIO_Config();					//蜂鸣器初始化，PA15
	OLED_Init();						//OLED初始化
	Encoder_Init();						//编码器初始化，TIM4和TIM8
	Motor_Init(7199,0);					//电机PWM初始化，TIM3
	DEBUG_USART_Init();					//调试串口设置，串口1，波特率115200
	BLUETOOTH_USART_Init();				//蓝牙串口设置，串口3，波特率9600
    CountSensor_Init();
	Voltage_ADC_Init();					//用于测量电压

	ROS_USART_Init();
	ROS1_USART_Init();
	
	MPU6050_Init();										//MPU6050初始化

	BEEP_ON;													//按键按下蜂鸣器提示
	delay_ms(200);
	BEEP_OFF;

	Car_Perimeter_Init();										//初始化轮子周长和轮距
	TIMING_TIM_Init(7199,49);									//5ms中断控制，大部分控制逻辑在里面
	
	while(1)
	{	
		if(get_rotate_done()==1)
		{
				GraySensor_TrackSimple(1000);
		}
		if(stop_flag<=2)
		{
//			if(Turn_Off()==Normal && startflag==0)
//			{
//				BEEP_ON;													//按键按下蜂鸣器提示
//				delay_ms(20);
//				BEEP_OFF;
//				startflag=1;
//			}

	
		
		if(Flag_Show)											//正常显示，非上位机模式
		{
			LED2_OFF;
			Show();												//显示屏
			data_transition();
			USART3_SEND();
  			Voltage_Sum += Get_Voltage();						//电压测量，每5次取一次平均
			if(++Voltage_Count == 5)
			Voltage = Voltage_Sum/5,Voltage_Count = 0,Voltage_Sum = 0;
		}
		else													//进入上位机模式
		{
			LED2_ON;											//LED2常亮指示进入了上位机模式
			DataScope();										//上位机示波器显示
		}
		delay_flag=1;	//使用50ms延时时，雷达数据会出现异常
		delay_50 = 0;
		while(delay_flag);	     								//通过定时中断实现的50ms延时，主要用于示波器				
	}
}
	
}


/***********************************END OF FILE********************************/

