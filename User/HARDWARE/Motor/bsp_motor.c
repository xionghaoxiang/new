#include "./Motor/bsp_motor.h"   

/**************************************************************************
Function: Motor_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
函数功能：控制电机的PWM初始化
入口参数: 预装载值和预分频器 
返回  值：无
**************************************************************************/	 	
void Motor_Init(u16 arr,u16 psc)
{
	PWM_OutPut_TIM_Init(arr,psc);//初始化定时器
}



/**************************************************************************
Function: Drive_Motor
Input   : motor_a,motor_b:Indicates the left/right motor PWM duty cycle
Output  : none
函数功能：驱动电机
入口参数: motor_a和motor_b数值指示左右电机PWM占空比，值越大速度越快
返回  值：无
**************************************************************************/	 	
void Set_Pwm(int motor_a,int motor_b)
{
	
	if(motor_a<0)		PWMA_IN1=7200,PWMA_IN2=7200+motor_a;
	else 	            PWMA_IN2=7200,PWMA_IN1=7200-motor_a;

	if(motor_b<0)		PWMB_IN1=7200,PWMB_IN2=7200+motor_b;
	else 	            PWMB_IN2=7200,PWMB_IN1=7200-motor_b;
}
void Stop_Pwm(void)
{
	
			PWMA_IN1=7200,PWMA_IN2=7200;
			PWMB_IN1=7200,PWMB_IN2=7200;
}

void SetMotor(int left, int right)
{
    // 限制最大 PWM 保护电机
    if(left > 7200)  left = 7200;
    if(left < -7200) left = -7200;

    if(right > 7200)  right = 7200;
    if(right < -7200) right = -7200;

    Set_Pwm(left, right);   // 使用你现有的驱动函数
}

void Motor_Brake(void)
{
    // 两边全部进入短路制动
    // 两个通道 PWM 输入相同即为刹车
    PWMA_IN1 = 7200;
    PWMA_IN2 = 7200;
    
    PWMB_IN1 = 7200;
    PWMB_IN2 = 7200;
}





