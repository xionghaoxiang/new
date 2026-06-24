#include "pid.h"

//float Velocity_KP = 0.18f,Velocity_KI = 0.17f;	//增量式PI参数，用于电机速度控制，光电编码器参数
float Velocity_KP = 200,Velocity_KI = 140;			//增量式PI参数，用于电机速度控制，霍尔编码器参数


float CCD_KP_Diff = 0.05,CCD_KD_Diff = 5.0,CCD_KI_Diff = 0.0001;

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/


int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
//	 if(Pwm>7200)Pwm=7200;
//	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
//	 if(Pwm>7200)Pwm=7200;
//	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}

/**************************************************************************
Function: CCD_PID
Input   : Current_Value;Target_Value
Output  : OutPut
函数功能：CCD巡线PID
入口参数: 当前CCD的值和目标值
返回  值：电机目标速度
**************************************************************************/	 	
float CCD_PID(float Current_Value,float Target_Value )
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Value-Current_Value;                         	 	//计算偏差
	Integral_bias+=Bias;	                                 		//求出偏差的积分
	if(Integral_bias>5000) Integral_bias=5000;
	else if(Integral_bias<-5000) Integral_bias=-5000;
	OutPut=CCD_KP_Diff*Bias+CCD_KI_Diff*Integral_bias+CCD_KD_Diff*(Bias-Last_Bias);
	Last_Bias=Bias;                                       		//保存上一次偏差
	if(MotorA.Motor_Pwm == 0 && MotorB.Motor_Pwm == 0)			//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                       
}



