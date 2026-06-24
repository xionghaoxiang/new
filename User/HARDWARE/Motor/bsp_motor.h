#ifndef __MOTOR_H
#define	__MOTOR_H


#include "stm32f10x.h"
#include "./GeneralTim/bsp_GeneralTim.h" 

//引脚定义

//#define    MOTOR1_GPIO_CLK     			RCC_APB2Periph_GPIOA
//#define    MOTOR1_GPIO_PORT    			GPIOA			   
//#define    MOTOR1_IN1_GPIO_PIN	 		GPIO_Pin_6
//#define    MOTOR1_IN2_GPIO_PIN	 		GPIO_Pin_7


//#define    MOTOR2_GPIO_CLK     			RCC_APB2Periph_GPIOB
//#define    MOTOR2_GPIO_PORT    			GPIOB			   
//#define    MOTOR2_IN1_GPIO_PIN	 		GPIO_Pin_0
//#define    MOTOR2_IN2_GPIO_PIN	 		GPIO_Pin_1







//这里是电机输出的PWM
#define PWMA_IN1 TIM3->CCR1
#define PWMA_IN2 TIM3->CCR2
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4


void Motor_Init(u16 arr,u16 psc);
void Set_Pwm(int motor_a,int motor_b);
void Stop_Pwm(void);
void SetMotor(int left, int right);
void Motor_Brake(void);

#endif
