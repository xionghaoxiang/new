#include "show.h"
#include "usartx.h"

/**************************************************************************
Function: OLED_Show
Input   : none
Output  : none
函数功能：显示屏显示函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Show(void)
{
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏
		//巡线模式显示
		//=============第二行显示CCD中值=======================//	
//		OLED_ShowString(00,10,"line_error:");
//		if( line_error<0)		OLED_ShowString(90,10,"-"),
//		OLED_ShowNumber(100,10,line_error,3,12);
//		else                 	OLED_ShowString(90,10,"+"),
//		OLED_ShowNumber(100,10,line_error,3,12);
    OLED_ShowNumber(50,10,racing_state1,2,12);
	OLED_ShowNumber(00,10,turn,2,12);
	
//		  OLED_ShowNumber(20,10,GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3),2,12);
//		  OLED_ShowNumber(40,10,GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8),2,12);
//		  OLED_ShowNumber(60,10,GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11),2,12);
		
		//=============第三行显示CCD阈值=======================//	
//		OLED_ShowString(00,20,"Threshold:");
//		if( CCD_Threshold<0)	OLED_ShowString(90,20,"-"),
//		OLED_ShowNumber(100,20,-CCD_Threshold,3,12);
//		else                 	OLED_ShowString(90,20,"+"),
//								OLED_ShowNumber(100,20, CCD_Threshold,3,12);
                      OLED_ShowString(00,20,"CIR:");
                      OLED_ShowNumber(30,20,stop_flag,2,12);
					  OLED_ShowString(50,20,"TOT:");
					  OLED_ShowNumber(90,20,total_distance*10,2,12);
					  OLED_ShowString(110,20,"m");
		//=============第四行显示左电机PWM与速度数据=======================//		
								OLED_ShowString(00,40,"V:");
//		if(MotorA.Motor_Pwm<0)	OLED_ShowString(20,40,"-"),
//								OLED_ShowNumber(36,40,MotorA.Motor_Pwm,4,12);
//		if(MotorA.Motor_Pwm>=0)	OLED_ShowString(30,40,"+"),
//								OLED_ShowNumber(36,40,MotorA.Motor_Pwm,4,12);
//		if(MotorB.Motor_Pwm<0)	OLED_ShowString(30,40,"-"),
//								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
//		if(MotorB.Motor_Pwm>=0)	OLED_ShowString(30,40,"+"),
//								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
        if(MotorA.Target_Encoder<0)	  	OLED_ShowString(15,40,"-");
		if(MotorA.Target_Encoder>=0)  	OLED_ShowString(15,40,"+");
		OLED_ShowNumber(30,40,((MotorA.Current_Encoder+MotorB.Current_Encoder)/2)*100,2,12);
		OLED_ShowString(55,40,"m/S");
//		if(MotorB.Target_Encoder<0)		OLED_ShowString(55,40,"-");
//		if(MotorB.Target_Encoder>=0)  	OLED_ShowString(55,40,"+");
//		OLED_ShowNumber(65,40,MotorB.Current_Encoder*1000,2,12);		
//								OLED_ShowString(66,30,"L-E:");
//OLED_ShowNumber(00,30,stop_flag,2,12);
//		if(MotorA.Target_Encoder<0)	  	OLED_ShowString(98,30,"-");
//		if(MotorA.Target_Encoder>=0)  	OLED_ShowString(98,30,"+");
//									OLED_ShowNumber(104,30,MotorA.Target_Encoder*1000,4,12);
        OLED_ShowString(0,30,"TIM ");				//模式显示
		OLED_ShowString(54,30,".");
		OLED_ShowString(78,30,"S");
		OLED_ShowNumber(40,30,RunTimer_GetMs()/1000,1,12);//显示电压
		OLED_ShowNumber(48,30,RunTimer_GetMs()/100%10,1,12);//显示电压
		OLED_ShowNumber(62,30,RunTimer_GetMs()/10%10,1,12);//显示电压
		OLED_ShowNumber(70,30,RunTimer_GetMs()%10,1,12);
		//=============第五行显示右电机PWM与速度数据=======================//		
//								OLED_ShowString(00,40,"R-P:");
//		if(MotorB.Motor_Pwm<0)	OLED_ShowString(30,40,"-"),
//								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
//		if(MotorB.Motor_Pwm>=0)	OLED_ShowString(30,40,"+"),
//								OLED_ShowNumber(36,40,MotorB.Motor_Pwm,4,12);
//										
//								OLED_ShowString(66,40,"R-E:");

//		if(MotorB.Target_Encoder<0)		OLED_ShowString(98,40,"-");
//		if(MotorB.Target_Encoder>=0)  	OLED_ShowString(98,40,"+");
//									OLED_ShowNumber(104,40,MotorB.Target_Encoder*1000,4,12);



		//=============第六行显示模式，电压与电机开关=======================//
				OLED_ShowNumber(00,50,Serial_RxFlag2,3,12);	   
//		OLED_ShowString(0,50,"NUM:23 ");				//模式显示
//		OLED_ShowString(54,50,".");
//		OLED_ShowString(78,50,"V");
//		//OLED_ShowNumber(0,50,Read_Encoder(Encoder1),2,12);
////		OLED_ShowNumber(0,50,Read_Encoder(Encoder1),2,12);
////    OLED_ShowNumber(20,50,Read_Encoder(Encoder2),2,12);
//		OLED_ShowNumber(62,50,Voltage/10%10,1,12);//显示电压
//		OLED_ShowNumber(70,50,Voltage%10,1,12);
//		if(Flag_Stop)          	OLED_ShowString(105,50, "OFF");
//		if(!Flag_Stop)        	OLED_ShowString(105,50, "ON ");

	//==================刷新==================//
	OLED_Refresh_Gram();	
}

/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	static u8 flag;
	int Velocity_Left_Show,Velocity_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1000)*2/3;			if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
	Velocity_Right_Show=MotorB.Velocity*1.1; 	if(Velocity_Right_Show<0) Velocity_Right_Show=-Velocity_Right_Show;			  //对编码器数据就行数据处理便于图形化
	Velocity_Left_Show= MotorA.Velocity*1.1;  	if(Velocity_Left_Show<0) Velocity_Left_Show=-Velocity_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//发送PID参数,在APP调参界面显示
	{
//		 printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)(CCD_KP_STank*100000),(int)(CCD_KD_STank*100000),(int)0,(int)0,(int)0,(int)0,(int)0,0,0);//打印到APP上面
		 PID_Send=0;		
  }
   else	if(flag==0)		// 发送电池电压，速度，角度等参数，在APP首页显示
	   printf("{A%d:%d:%d:%d}$",(int)(MotorA.Current_Encoder*100),(int)(MotorB.Current_Encoder*100),(int)Voltage_Show,(int)0); //打印到APP上面
	 else				//发送小车姿态角，在波形界面显示
	    printf("{B%d:%d}$",(int)(MotorA.Current_Encoder*100),(int)(MotorB.Current_Encoder*100)); //显示左右编码器的速度																	   //可按格式自行增加显示波形，最多可显示五个
}


//上位机示波器
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{   
	u8 i;			//计数变量
	u8 Send_Count;	//串口需要发送的数据个数
	float Vol;		//电压变量
	Vol=(float)Voltage/100;
	DataScope_Get_Channel_Data( MotorA.Velocity, 1 );     	//显示左轮速度，单位mm/s
	DataScope_Get_Channel_Data( MotorB.Velocity, 2 );    	//显示右轮速度
	DataScope_Get_Channel_Data( Vol, 3 );               	//显示电池电压 单位：V
//	DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0, 7 );
//	DataScope_Get_Channel_Data( 0, 8 ); 
//	DataScope_Get_Channel_Data(0, 9 );  
//	DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(CHANNEL_NUMBER);//CHANNEL_NUMBER可改变通道数量，目前是4
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：uint
作    者：平衡小车之家
**************************************************************************/
u16 myabs(int Input)
{
	int Output;
	if(Input > 0)
		Output = Input;
	else
		Output = -Input;
	return Output;
}


