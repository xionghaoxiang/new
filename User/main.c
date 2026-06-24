#include "Header.h"

uint8_t startflag = 0;
volatile u8 delay_50,delay_flag; 		//��ʱ����			
u16 Voltage;							//��ѹ�������Ŵ�100������
u8 Flag_Stop;							//�����ͣ��־λ
u8 PS2_ON_Flag = 0,APP_ON_Flag = 0,ROS_ON_Flag = 0,Remote_ON_Flag;		//Ĭ�����з�ʽ������
u8 Car_Num=Diff_Car;								//���ͺ���ѡ��
u8 Flag_Show = 1;						//��ʾ��־λ��Ĭ�Ͽ����������л�����λ��ģʽ����ʱ�ر�
float Perimeter; 						//���ӵ��ܳ�
float Wheelspacing; 					//���ӵ��־�
u16 DISTANCE=0,ANGLE=0;
float test1,test2;
//volatile uint32_t sys_ms = 0;   // ���������
uint32_t brake_timer = 0;
uint8_t brake_done = 0;
uint8_t racing_state1=2;		//racing_state��ƽ��

void USART2_DisableAllInterrupts(void)
{
    // 1. ������ڿ��ƼĴ����������ж�ʹ��λ
    USART2->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_PEIE);
    USART2->CR2 &= ~USART_CR2_LBDIE;    // LIN�ж�
    USART2->CR3 &= ~(USART_CR3_EIE | USART_CR3_CTSIE);  // �����жϺ�CTS�ж�
    
    // 2. ��NVIC�н���USART1�ж�
    NVIC_DisableIRQ(USART2_IRQn);
    
    // 3. ��ѡ��������й�����жϱ�־
    USART2->SR = 0;
}

/**
 * @brief  ��ȫ��USART1�������ж�
 * @note   ���������USART1_DisableAllInterrupts()��ȫ�Գ�
 */
void USART2_EnableAllInterrupts(void)
{
    // 1. ��NVIC������USART1�жϣ����������ã������ж��޷�������
    NVIC_EnableIRQ(USART2_IRQn);
    
    // 2. ���ô��ڿ��ƼĴ����������ж�ʹ��λ
    // ����CR1�Ĵ����е��ж�
    USART2->CR1 |= (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_PEIE);
    
    // ����CR2�Ĵ����е�LIN�ж�
    USART2->CR2 |= USART_CR2_LBDIE;    // LIN�ж�
    
    // ����CR3�Ĵ����еĴ����жϺ�CTS�ж�
    USART2->CR3 |= (USART_CR3_EIE | USART_CR3_CTSIE);  // �����жϺ�CTS�ж�
    
    // 3. ��ѡ��������й�����жϱ�־��ȷ����ʼʱû���󴥷���
    USART2->SR = 0;
}
/**************************************************************************  
Function: Main function
Input   : none
Output  : none
�������ܣ�������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
int main(void)
{	
	u32 Voltage_Sum = 0;
	u8 Voltage_Count = 0;				//��ѹ������ر���
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�ж����ȼ�����
	JTAG_Set(JTAG_SWD_DISABLE);     //�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //��SWD�ӿ� �������������SWD�ӿڵ���
	LED_GPIO_Config();					//LED��ʼ����PC2��PC3
	Key_GPIO_Config();					//������ʼ����PA0��PC13
	BEEP_GPIO_Config();					//��������ʼ����PA15
	OLED_Init();						//OLED��ʼ��
	Encoder_Init();						//��������ʼ����TIM4��TIM8
	Motor_Init(7199,0);					//���PWM��ʼ����TIM3
	DEBUG_USART_Init();					//���Դ������ã�����1��������115200
	BLUETOOTH_USART_Init();				//�����������ã�����3��������9600
    CountSensor_Init();
	Voltage_ADC_Init();					//���ڲ�����ѹ

	ROS_USART_Init();
	ROS1_USART_Init();
	
	MPU6050_Init();										//MPU6050��ʼ��

	BEEP_ON;													//�������·�������ʾ
	delay_ms(200);
	BEEP_OFF;

	Car_Perimeter_Init();										//��ʼ�������ܳ����־�
	TIMING_TIM_Init(7199,49);									//5ms�жϿ��ƣ��󲿷ֿ����߼�������
	
	while(1)
	{	
		if(Turn_Off()==Normal)
		{
			if(get_rotate_done()==1)
			{
				GraySensor_TrackSimple(1000);
			}
			// if (Target_Turns>=4*Target_Laps)
			// {
			// 	SetMotor(0,0);
			// }
		}
			
		else
				{
					SetMotor(0,0);
				}

		

		if(stop_flag<=2)
		{
//			if(Turn_Off()==Normal && startflag==0)
//			{
//				BEEP_ON;													//�������·�������ʾ
//				delay_ms(20);
//				BEEP_OFF;
//				startflag=1;
//			}

	
		
		if(Flag_Show)											//������ʾ������λ��ģʽ
		{
			LED2_OFF;
			Show();												//��ʾ��
			data_transition();
			USART3_SEND();
  			Voltage_Sum += Get_Voltage();						//��ѹ������ÿ5��ȡһ��ƽ��
			if(++Voltage_Count == 5)
			Voltage = Voltage_Sum/5,Voltage_Count = 0,Voltage_Sum = 0;
		}
		else													//������λ��ģʽ
		{
			LED2_ON;											//LED2����ָʾ��������λ��ģʽ
			DataScope();										//��λ��ʾ������ʾ
		}
		delay_flag=1;	//ʹ��50ms��ʱʱ���״����ݻ�����쳣
		delay_50 = 0;
		while(delay_flag);	     								//ͨ����ʱ�ж�ʵ�ֵ�50ms��ʱ����Ҫ����ʾ����				
	}
}
	
}


/***********************************END OF FILE********************************/

