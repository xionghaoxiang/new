#include "control.h"

#include "MPU6050.h"

static uint8_t needturn=0;

float Move_X =0,Move_Z = 0;						//目标速度和目标转向速度
float PWM_Left,PWM_Right;					//左右电机PWM值
float RC_Velocity,RC_Turn_Velocity;			//遥控控制的速度
volatile uint8_t stop_flag = 0;
Motor_parameter MotorA,MotorB;				//左右电机相关变量
u8 turn_flag=0;
 u8 turn=0;
float CCD_Move_X = 0.3;						//CCD巡线速度

u8 Ros_count=0;
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据   
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;

// ????
Odometry_t odometry = {0, 0, 0, 0, 0};
float total_distance = 0;         // 总的行驶距离
float segment_distance = 0;       // 当前段距离
float current_speed = 0;          // 当前速度

u32 timeren =0 ;
#define WHEEL_BASE          0.15f   // 轮子间距
#define ROTATE_B_TO_D_DELTA       (-95.0f)
#define ROTATE_D_TO_C_DELTA       (105.0f)

#define LINE_KP      30.0f
#define LINE_KI      0.0f
#define LINE_KD      3.0f
#define LINE_PWM_MAX 3000
#define LINE_MIN_PWM 200   // 死区补偿

float yaw_target = 0;          // 要保持的目标角度
float yaw_error_last = 0;
float yaw_integral = 0;

#define YAW_KP   25.0f
#define YAW_KI    0.0f
#define YAW_KD    2.0f

// 全局 PID 状态
float line_last_error = 0;
float line_integral = 0;

// 目标中心线
#define TARGET_POS 80

// 左右轮基础前进速度
#define BASE_SPEED 1500

// 一个小的重置函数（放 motor 模块）
void Reset_All_Control(void)
{
    // 直接停止 PWM 输出（确保立刻无输出）
    SetMotor(0, 0);

    // 清除积分 / 误差（根据你变量名）
    line_integral = 0;
    line_last_error = 0;


    // reset timers/counters relevant
    timeren = 0;
}


// ------------------------
// PID 控制函数，每 5ms 调用
// ------------------------
void Heading_Hold_Run(int base_speed)
{
    static uint8_t init_flag = 0;  // 自动初始化标志
    static float yaw_target = 0;
    static float yaw_error_last = 0;
    static float yaw_integral = 0;

    Read_DMP();    // 更新 Yaw

    // --- 第一次进入，记录当前方向 ---
    if(init_flag == 0)
    {
        yaw_target = Yaw;      // 记住当时角度作为直行参考
        yaw_error_last = 0;
        yaw_integral = 0;
        init_flag = 1;
    }

    // --- 计算误差 ---
    float error = yaw_target - Yaw;

    // 处理跨越 -180 ~ 180
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // --- PID ---
    yaw_integral += error;
    float derivative = error - yaw_error_last;
    yaw_error_last = error;

    float output = YAW_KP * error + YAW_KI * yaw_integral + YAW_KD * derivative;

    // --- 左右轮速度 ---
    int left_pwm  = base_speed - (int)output;
    int right_pwm = base_speed + (int)output;

    // --- 限幅 ---
    if(left_pwm > 7200) left_pwm = 7200;
    if(left_pwm < -7200) left_pwm = -7200;
    if(right_pwm > 7200) right_pwm = 7200;
    if(right_pwm < -7200) right_pwm = -7200;

    SetMotor(-left_pwm, right_pwm);
}
void Heading_Hold_Run1(int base_speed)
{
    static uint8_t init_flag = 0;  // 自动初始化标志
    static float yaw_target = 0;
    static float yaw_error_last = 0;
    static float yaw_integral = 0;

    Read_DMP();    // 更新 Yaw

    // --- 第一次进入，记录当前方向 ---
    if(init_flag == 0)
    {
        yaw_target = Yaw;      // 记住当时角度作为直行参考
        yaw_error_last = 0;
        yaw_integral = 0;
        init_flag = 1;
    }

    // --- 计算误差 ---
    float error = yaw_target - Yaw;

    // 处理跨越 -180 ~ 180
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // --- PID ---
    yaw_integral += error;
    float derivative = error - yaw_error_last;
    yaw_error_last = error;

    float output = YAW_KP * error + YAW_KI * yaw_integral + YAW_KD * derivative;

    // --- 左右轮速度 ---
    int left_pwm  = base_speed - (int)output;
    int right_pwm = base_speed + (int)output;

    // --- 限幅 ---
    if(left_pwm > 7200) left_pwm = 7200;
    if(left_pwm < -7200) left_pwm = -7200;
    if(right_pwm > 7200) right_pwm = 7200;
    if(right_pwm < -7200) right_pwm = -7200;

    SetMotor(-left_pwm, right_pwm);
}

void Heading_Hold_Run2(int base_speed)
{
    static uint8_t init_flag = 0;  // 自动初始化标志
    static float yaw_target = 0;
    static float yaw_error_last = 0;
    static float yaw_integral = 0;

    Read_DMP();    // 更新 Yaw

    // --- 第一次进入，记录当前方向 ---
    if(init_flag == 0)
    {
        yaw_target = Yaw;      // 记住当时角度作为直行参考
        yaw_error_last = 0;
        yaw_integral = 0;
        init_flag = 1;
    }

    // --- 计算误差 ---
    float error = yaw_target - Yaw;

    // 处理跨越 -180 ~ 180
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // --- PID ---
    yaw_integral += error;
    float derivative = error - yaw_error_last;
    yaw_error_last = error;

    float output = YAW_KP * error + YAW_KI * yaw_integral + YAW_KD * derivative;

    // --- 左右轮速度 ---
    int left_pwm  = base_speed - (int)output;
    int right_pwm = base_speed + (int)output;

    // --- 限幅 ---
    if(left_pwm > 7200) left_pwm = 7200;
    if(left_pwm < -7200) left_pwm = -7200;
    if(right_pwm > 7200) right_pwm = 7200;
    if(right_pwm < -7200) right_pwm = -7200;

    SetMotor(-left_pwm, right_pwm);
}

void Heading_Hold_Run3(int base_speed)
{
    static uint8_t init_flag = 0;  // 自动初始化标志
    static float yaw_target = 0;
    static float yaw_error_last = 0;
    static float yaw_integral = 0;

    Read_DMP();    // 更新 Yaw

    // --- 第一次进入，记录当前方向 ---
    if(init_flag == 0)
    {
        yaw_target = Yaw;      // 记住当时角度作为直行参考
        yaw_error_last = 0;
        yaw_integral = 0;
        init_flag = 1;
    }

    // --- 计算误差 ---
    float error = yaw_target - Yaw;

    // 处理跨越 -180 ~ 180
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // --- PID ---
    yaw_integral += error;
    float derivative = error - yaw_error_last;
    yaw_error_last = error;

    float output = YAW_KP * error + YAW_KI * yaw_integral + YAW_KD * derivative;

    // --- 左右轮速度 ---
    int left_pwm  = base_speed - (int)output;
    int right_pwm = base_speed + (int)output;

    // --- 限幅 ---
    if(left_pwm > 7200) left_pwm = 7200;
    if(left_pwm < -7200) left_pwm = -7200;
    if(right_pwm > 7200) right_pwm = 7200;
    if(right_pwm < -7200) right_pwm = -7200;

    SetMotor(-left_pwm, right_pwm);
}

void Line_Follow_PID(int line_pos)
{
    float error = TARGET_POS - line_pos;
    line_integral += error;  // 积分项，可加限幅防止积分饱和
    float derivative = error - line_last_error;
    line_last_error = error;

    // PID 输出
    float output = LINE_KP * error + LINE_KI * line_integral + LINE_KD * derivative;

    // 限幅
    if(output > LINE_PWM_MAX) output = LINE_PWM_MAX;
    if(output < -LINE_PWM_MAX) output = -LINE_PWM_MAX;

    // 最小输出死区补偿
    if(output > 0 && output < LINE_MIN_PWM) output = LINE_MIN_PWM;
    if(output < 0 && output > -LINE_MIN_PWM) output = -LINE_MIN_PWM;

    // 根据 PID 输出调整左右轮
    // output > 0 → 偏左 → 右轮加速，左轮减速
    // output < 0 → 偏右 → 左轮加速，右轮减速
    int left_speed = BASE_SPEED - (int)output;
    int right_speed = BASE_SPEED + (int)output;

    // 调用 SetMotor 控制电机
    SetMotor(-left_speed, right_speed);
}

// PD 参数
#define Kpp 40.0f
#define Kdd 4.0f
#define PWM_LIMIT 7000
#define MIN_PWM 300   // 死区补偿，避免输出太小不起作用

volatile uint8_t rotate_done = 1;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done1 = 0;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done2 = 0;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done3 = 0;   // 1 = 完成（默认不在旋转）

volatile uint8_t rotate_done4 = 0;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done5 = 0;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done6 = 0;   // 1 = 完成（默认不在旋转）
volatile uint8_t rotate_done7 = 0;   // 1 = 完成（默认不在旋转）

float   target_rotate_angle = 0; // 当前旋转目标角度
float   target_rotate_angle1 = 0; // 当前旋转目标角度
float   target_rotate_angle2 = 0; // 当前旋转目标角度
float   target_rotate_angle3 = 0; // 当前旋转目标角度

float   target_rotate_angle4 = 0; // 当前旋转目标角度
float   target_rotate_angle5 = 0; // 当前旋转目标角度
float   target_rotate_angle6 = 0; // 当前旋转目标角度
float   target_rotate_angle7 = 0; // 当前旋转目标角度


static float Normalize_Angle(float angle)//转换，标准化
{
	while (angle > 180.0f)
	{
		angle -= 360.0f;
	}
	while (angle < -180.0f)
	{
		angle += 360.0f;
	}
	return angle;
}

uint8_t get_rotate_done(void)
{
	return rotate_done;
}

void set_rotate_done(void)
{
	rotate_done=0;
}
//// 固定角度旋转函数，每次 5ms 调用一次


void Rotate_Fixed_Angle6(float delta_angle)
{
    static uint8_t init_flag = 0;
    static float start_yaw = 0;      // ??????
    static float accumulated = 0;    // ??????(???)
		
		Read_DMP();
    // ?????:???????????
    if (!init_flag)
    {
        start_yaw = Yaw;                     // ??????
        target_rotate_angle = Normalize_Angle(Yaw + delta_angle);
        accumulated = 0;
        init_flag = 1;
    }

    // ????????????(???,??? 0?)
    float diff = Yaw - start_yaw;
    if (diff > 180)  diff -= 360;
    if (diff < -180) diff += 360;

    // ??????????(?? < 5?)
    float error = target_rotate_angle - Yaw;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < 3.0f)
    {
        SetMotor(0, 0);
        rotate_done = 1;
			init_flag=0;
        return;
    }

    // ----------------- ?????? -----------------
    // ????????????(? = ??,? = ??)
    int8_t direction = (delta_angle > 0) ? 1 : -1;

    // ?? PWM ?(??),??????????????
    const float FIXED_SPEED_PWM1 = 300.0f;   // ???,????
		const float FIXED_SPEED_PWM2 = 800.0f;
		
    // ?????(????)
    float left_pwm  = -direction * FIXED_SPEED_PWM1;
    float right_pwm =  direction * FIXED_SPEED_PWM2;

    SetMotor(left_pwm, right_pwm);
}

//void set_rotate_done()
//{
//	rotate_done = 0;
//}
void Rotate_Fixed_Angle1(float delta_angle)
{
    static uint8_t init_flag = 0;
    static float last_error = 0;
    Read_DMP();
    float error;

    // 第一次进入初始化目标角度
    if(!init_flag)
    {
        rotate_done1 = 0; // 开始旋转
        target_rotate_angle1 = Yaw + delta_angle;

        // 归一化到 0~360
        if(target_rotate_angle1 >= 180) target_rotate_angle1 -= 360;
        if(target_rotate_angle1 < -180)    target_rotate_angle1 += 360;

        last_error = 0;
        init_flag = 1;
    }

    // 误差计算（处理跨 0°）
    error = target_rotate_angle1 - Yaw;
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // 判断是否到位
    if(fabs(error) < 15.0f)
    {
        SetMotor(0, 0);
        rotate_done1 = 1;
//        init_flag=0;
        last_error = 0;
		return ;
    }

    // PD 控制
    float d_error = error - last_error;
    last_error = error;
    float output = Kpp * error + Kdd * d_error;

    // 限幅
    if(output > PWM_LIMIT) output = PWM_LIMIT;
    if(output < -PWM_LIMIT) output = -PWM_LIMIT;

    // 最小输出死区补偿，保证一开始就有旋转
    if(output > 0 && output < MIN_PWM) output = MIN_PWM;
    if(output < 0 && output > -MIN_PWM) output = -MIN_PWM;

    // 根据误差方向生成左右轮差速（兼容你 SetMotor）
    // 左轮反，右轮正 → 小车左转
 
    SetMotor(output, output);

}
void Rotate_Fixed_Angle2(float delta_angle)
{
    static uint8_t init_flag = 0;
    static float last_error = 0;
    Read_DMP();
    float error;

    // 第一次进入初始化目标角度
    if(!init_flag)
    {
        rotate_done2 = 0; // 开始旋转
        target_rotate_angle2 = Yaw + delta_angle;

        // 归一化到 0~360
        if(target_rotate_angle2 >= 180) target_rotate_angle2 -= 360;
        if(target_rotate_angle2 < -180)    target_rotate_angle2 += 360;

        last_error = 0;
        init_flag = 1;
    }

    // 误差计算（处理跨 0°）
    error = target_rotate_angle2 - Yaw;
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // 判断是否到位
    if(fabs(error) < 15.0f)
    {
        SetMotor(0, 0);
        rotate_done2 = 1;
//        init_flag=0;
        last_error = 0;
		return ;
    }

    // PD 控制
    float d_error = error - last_error;
    last_error = error;
    float output = Kpp * error + Kdd * d_error;

    // 限幅
    if(output > PWM_LIMIT) output = PWM_LIMIT;
    if(output < -PWM_LIMIT) output = -PWM_LIMIT;

    // 最小输出死区补偿，保证一开始就有旋转
    if(output > 0 && output < MIN_PWM) output = MIN_PWM;
    if(output < 0 && output > -MIN_PWM) output = -MIN_PWM;

    // 根据误差方向生成左右轮差速（兼容你 SetMotor）
    // 左轮反，右轮正 → 小车左转
 
    SetMotor(output, output);

}

void Rotate_Fixed_Angle3(float delta_angle)
{
    static uint8_t init_flag = 0;
    static float last_error = 0;
    Read_DMP();
    float error;

    // 第一次进入初始化目标角度
    if(!init_flag)
    {
        rotate_done3 = 0; // 开始旋转
        target_rotate_angle3 = Yaw + delta_angle;

        // 归一化到 0~360
        if(target_rotate_angle3 >= 180) target_rotate_angle3 -= 360;
        if(target_rotate_angle3 < -180)    target_rotate_angle3 += 360;

        last_error = 0;
        init_flag = 1;
    }

    // 误差计算（处理跨 0°）
    error = target_rotate_angle3 - Yaw;
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;

    // 判断是否到位
    if(fabs(error) < 15.0f)
    {
        SetMotor(0, 0);
        rotate_done3 = 1;
//        init_flag=0;
        last_error = 0;
		return ;
    }

    // PD 控制
    float d_error = error - last_error;
    last_error = error;
    float output = Kpp * error + Kdd * d_error;

    // 限幅
    if(output > PWM_LIMIT) output = PWM_LIMIT;
    if(output < -PWM_LIMIT) output = -PWM_LIMIT;

    // 最小输出死区补偿，保证一开始就有旋转
    if(output > 0 && output < MIN_PWM) output = MIN_PWM;
    if(output < 0 && output > -MIN_PWM) output = -MIN_PWM;

    // 根据误差方向生成左右轮差速（兼容你 SetMotor）
    // 左轮反，右轮正 → 小车左转
 
    SetMotor(output, output);

}

void Calculate_Distance_5ms(void)
	{
		//计算当前速度
		current_speed = (fabs(MotorA.Current_Encoder) + fabs(MotorB.Current_Encoder)) * 0.5f;
		
		// 5ms????????
		float distance_increment = current_speed * 0.005f;  // 5ms = 0.005?
		
		// 累计距离
		total_distance += distance_increment;
		segment_distance += distance_increment;
		

	}
/**************************************************************************
Function: Control Function
Input   : none
Output  : none
函数功能：5ms定时中断控制函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
int TIMING_TIM_IRQHandler(void)
{
	static int ct;
	if(TIM_GetITStatus( TIMING_TIM, TIM_IT_Update) != RESET ) 
	{			
		TIM_ClearITPendingBit(TIMING_TIM , TIM_IT_Update);
		
		Get_Velocity_From_Encoder();								//读取左右编码器的值且转换成速度
    Calculate_Distance_5ms();
		Get_KeyVal();		 		
//		Update_Odometry();
		 Read_DMP();
	OLED_ShowString(50,0,"Yaw");//fabs(error)
		OLED_ShowNumber(0,0,(u32)get_rotate_done(),1,12);
        OLED_ShowNumber(30,0,(u32)Target_Laps,1,12);
    OLED_ShowNumber(70, 0, (u32)Yaw, 2, 12); 
    Turn_Off();
		if(delay_flag)												//50ms延时
		{
			if(++delay_50==10) delay_50=0,delay_flag=0;            	//给主函数提供50ms的精准延时，主要是用于上位机示波器
			else if(delay_50>10)	delay_50=0,delay_flag=0;
		}			
			if(APP_ON_Flag == RC_ON)								//开启蓝牙控制时，需上拉轮盘直到显示屏出现 bluetooth 字样
				Bluetooth_Control();								
	
			if(rotate_done==0)
			{
			Rotate_Fixed_Angle6(85);
			}
//			line_error=Serial_RxPacket[0];
		
	}
	return 0;
}







/**************************************************************************
Function: Bluetooth_Control
Input   : none
Output  : none
函数功能：手机蓝牙控制
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Bluetooth_Control(void)
{
	if(Flag_Direction==0) Move_X=0,Move_Z=0;  			 						//停止
	else if(Flag_Direction==1) Move_X=RC_Velocity,Move_Z=0;  					//前进
	else if(Flag_Direction==2) Move_X=RC_Velocity,Move_Z=Pi/4;  	//右前
	else if(Flag_Direction==3) Move_X=0,Move_Z=Pi/2;   				//向右
	else if(Flag_Direction==4) Move_X=-RC_Velocity,Move_Z=Pi/4; 	//右后
	else if(Flag_Direction==5) Move_X=-RC_Velocity,Move_Z=0;    				//后退
	else if(Flag_Direction==6) Move_X=-RC_Velocity,Move_Z=-Pi/4; 	//左后
	else if(Flag_Direction==7) Move_X=0,Move_Z=-Pi/2;      		 	//向左
	else if(Flag_Direction==8) Move_X=RC_Velocity,Move_Z=-Pi/4;  	//左前
	else Move_X=0,Move_Z=0; 
	
	Move_X=Move_X/1000;     Move_Z=-Move_Z;
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(void)
{
	
	 //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr; 
		OriginalEncoder.A=Read_Encoder(Encoder1);	
		OriginalEncoder.B=Read_Encoder(Encoder2);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		Encoder_A_pr=OriginalEncoder.A; 
		Encoder_B_pr=-OriginalEncoder.B;
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MotorA.Current_Encoder= Encoder_A_pr*Frequency*Perimeter/1560.0f;  
		MotorB.Current_Encoder= Encoder_B_pr*Frequency*Perimeter/1560.0f; 	//1560=4*13*30=2（两路脉冲）*2（上下沿计数）*霍尔编码器13线*电机的减速比
        //OLED_ShowNumber(20,50,MotorA.Current_Encoder,2,12);
	//		MotorA.Current_Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Akm_wheelspacing//(4*13*30);  
//		MotorB.Current_Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Akm_wheelspacing/Encoder_precision;  
	
}
/**************************************************************************
Function: Drive_Motor
Input   : none
Output  : none
函数功能：运动学逆解
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	Move_X = target_limit_float(Move_X,-1.2,1.2);
	Move_Z = target_limit_float(Move_Z,-Pi/3,Pi/3);

		  if(Vx<0) Vz=-Vz;
	      else     Vz=Vz;
			//Inverse kinematics //运动学逆解
		   MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
		   MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		   MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
	     MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
	
}
/**************************************************************************
Function: Get_Motor_PWM
Input   : none
Output  : none
函数功能：转换成驱动电机的PWM
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Motor_PWM(void)
{
	//计算左右电机对应的PWM
	MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
	MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);

		//滤波，使起步和停止稍微平滑一些
		MotorA.Motor_Pwm  = Mean_Filter_Left(MotorA.Motor_Pwm);
		MotorB.Motor_Pwm  = Mean_Filter_Right(MotorB.Motor_Pwm);

	//限幅
	MotorA.Motor_Pwm  = PWM_Limit(MotorA.Motor_Pwm,PWM_MAX,PWM_MIN);
	MotorB.Motor_Pwm  = PWM_Limit(MotorB.Motor_Pwm,PWM_MAX,PWM_MIN);
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: Check whether it is abnormal
Input   : none
Output  : 1:Abnormal;0:Normal
函数功能：异常关闭电机
入口参数: 无 
返回  值：1：异常  0：正常
**************************************************************************/	 	
u8 Turn_Off(void)
{
	u8 temp = Normal;
	Flag_Stop = KEY2_STATE;			//读取按键2状态，按键2控制电机的开关
	if(Voltage<1000)				//电池电压低于10V关闭电机,LED灯快速闪烁
		LED_Flash(50),temp=Abnormal;
	else
		LED_Flash(200);				//每一秒闪一次，正常运行
	if(Flag_Stop)
		temp=Abnormal;
	return temp;			
}

/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
函数功能：数据滑动滤波
入口参数：数据
返回  值：滤波后的数据
**************************************************************************/
float Mean_Filter_Left(float data)
{
	u8 i;
	float Sum_Data = 0; 
	float Filter_Data;
	static  float Speed_Buf[FILTERING_TIMES]={0};
	for(i = 1 ; i<FILTERING_TIMES; i++)
	{
		Speed_Buf[i - 1] = Speed_Buf[i];
	}
	Speed_Buf[FILTERING_TIMES - 1] =data;

	for(i = 0 ; i < FILTERING_TIMES; i++)
	{
		Sum_Data += Speed_Buf[i];
	}
	Filter_Data = (s32)(Sum_Data / FILTERING_TIMES);
	return Filter_Data;
  
}
/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
函数功能：数据滑动滤波
入口参数：数据
返回  值：滤波后的数据
**************************************************************************/
float Mean_Filter_Right(float data)
{
	  u8 i;
	  float Sum_Data = 0; 
	  float Filter_Data;
	  static  float Speed_Buf[FILTERING_TIMES]={0};
	  for(i = 1 ; i<FILTERING_TIMES; i++)
	  {
		Speed_Buf[i - 1] = Speed_Buf[i];
	  }
	  Speed_Buf[FILTERING_TIMES - 1] =data;

	  for(i = 0 ; i < FILTERING_TIMES; i++)
	  {
		Sum_Data += Speed_Buf[i];
	  }
	  Filter_Data = (s32)(Sum_Data / FILTERING_TIMES);
	  return Filter_Data;
}

/**************************************************************************
Function: Car_Perimeter_Init
Input   : none
Output  : none
函数功能：计算小车各轮子的周长
入口参数：无
返回  值：无
**************************************************************************/
void Car_Perimeter_Init(void)
{
		Perimeter = Diff_Car_Wheel_diameter*Pi;
		Wheelspacing = Diff_wheelspacing;
}

/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      //读取加速度、角速度、倾角
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
//		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
//		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
//		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/Pi;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/Pi;     //计算倾角，转换单位为度
		Gyro_X=Gyro_X/65.5;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
		Gyro_Y=Gyro_Y/65.5;                              //陀螺仪量程转换	
		if(way==2)		  	
		{
			Roll= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波
			Pitch = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(way==3) 
		{  
			Roll = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
			Pitch= -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
	}
}


/**
  * @brief  7?????????(????,???)
  * @param  ?
  * @retval ?
  */
/**
  * @brief  8?????????(????)
  * @param  ?
  * @retval ?
  */
void CountSensor_Init(void)
{
    /* 1. ???? */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // ??GPIOA??
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // ??GPIOB??
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    // ??AFIO??(??!)
    
    /* 2. ??!??SWD????,??PA13?PA14????IO */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);    // ????????
    
    /* 3. GPIO?????? */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           // ????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* ========== GPIOA ???? ========== */
    // PA13 - GS0 (???)
    // PA14 - GS1 (??2)  ? ??????PA3
    // PA11 - GS3 (??4)  ? ??:????PA2?
    // PA8  - GS4 (????)
    // PA12 - GS5 (????)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_11 | GPIO_Pin_8 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* ========== GPIOB ???? ========== */
    // PB13 - GS6 (??2)
    // PB12 - GS7 (???)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void Read_All_Sensors(uint8_t *sensor_states)
{
    sensor_states[0] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13);  // GS0 - ???
    sensor_states[1] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);   // GS1 - ??2
    sensor_states[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14);   // GS2 - ??3
    sensor_states[3] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);  // GS3 - ??4
    sensor_states[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);   // GS4 - ????
    sensor_states[5] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);  // GS5 - ????
    sensor_states[6] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);  // GS6 - ??2
    sensor_states[7] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);  // GS7 - ???
			OLED_ShowNumber(0,60,(u32)sensor_states[2],1,12);
}

void GraySensor_TrackSimple(int base_speed)
{
    uint8_t sensors[8];
		uint8_t a,b,c,d;
	a=(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 1 && sensors[6] == 1 && sensors[7] == 1);
	b=(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 1 && sensors[5] == 1 && sensors[6] == 1 && sensors[7] == 1);
	c=(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1 && sensors[6] == 1 && sensors[7] == 1);
	d=(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1 && sensors[6] == 1 && sensors[7] == 1);
    Read_All_Sensors(sensors);
    
    // ????4????: GS3(??3), GS4(??4), GS5(??5), GS6(??6)
    // ????: ?1(GS3), ???(GS4), ???(GS5), ?1(GS6)
    
    // ?????? (GS4?GS5)
    if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)
    {
        // 中间两个
        SetMotor(-base_speed, base_speed);
    }
    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 1 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)  // ?????????? ? ??,????
    {
        // 右乙?
        SetMotor(-(base_speed + 150), base_speed - 150);
    }
    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 1 && sensors[5] == 1 && sensors[6] == 0 && sensors[7] == 0)  // ?????????? ? ??,????
    {
        // 左一
        SetMotor(-(base_speed - 150), base_speed + 150);
    }
		    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)  // ?1?????,????
    {
        //2
        SetMotor(-(base_speed + 200), base_speed - 200);
    }
				    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 1 && sensors[6] == 0 && sensors[7] == 0)  // ?1?????,????
    {
        //5
        SetMotor(-(base_speed + 200), base_speed - 200);
    }
    else if(sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 1 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)  // ?1?????,????
    {
        // 右二
        SetMotor(-(base_speed + 300), base_speed - 300);
    }
    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 1 && sensors[6] == 1 && sensors[7] == 0)  // ?1?????,????
    {
        // 左二
        SetMotor(-(base_speed - 300), base_speed + 300);
    }
		else if(sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)  // ?1?????,????
    {
        // 右三
        SetMotor(-(base_speed + 500), base_speed - 500);
    }
    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 1 && sensors[7] == 1)  // ?1?????,????
    {
        // 左三
        SetMotor(-(base_speed - 500), base_speed + 500);
    }
		else if(sensors[0] == 1 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 0)  // ?1?????,????
    {
        // 右四
        SetMotor(-(base_speed + 800), base_speed - 800);
    }
    else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0 && sensors[5] == 0 && sensors[6] == 0 && sensors[7] == 1)  // ?1?????,????
    {
        // 左四
        SetMotor(-(base_speed - 800), base_speed + 800);
    }
		else if((( sensors[4] +  sensors[5] + sensors[6] + sensors[7] ) >=3)||(total_distance>0.9)) // ?1?????,????
    {
				set_rotate_done();
                total_distance=0;
                Target_Turns++;

    }
    else
    {
        // ???????,?????
        SetMotor(-base_speed, base_speed);
    }
}
uint8_t get_needturn(void)
{
	return needturn;

}
uint16_t CountSensor_Get(void)
{
	return count_times;
}

volatile uint8_t blackline_trigger = 0;
volatile uint8_t count_times = 0;

/*关键修改4：修改中断服务函数名称*/
void EXTI15_10_IRQHandler(void)  // EXTI15~10共享同一个中断服务函数
{
    /*关键修改5：检查EXTI_Line11中断标志*/
    if(EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        blackline_trigger = 1;
        
        // 你可以在这里添加读取其他传感器引脚（如PA8, PA12, PA13）的逻辑
        // 注意：PA11现在是中断触发引脚，可能不适合同时作为普通输入引脚使用
        
        // 例如，读取其他三个传感器：
        // uint8_t s8  = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
        // uint8_t s12 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
        // uint8_t s13 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13);
        
        /*关键修改6：清除EXTI_Line11中断标志*/
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}