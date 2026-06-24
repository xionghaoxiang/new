/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/


#ifndef __CONTROL_H
#define __CONTROL_H

#include "Lidar.h"
#include "Header.h"

//PWM限制最大最小值
#define PWM_MAX  6900
#define PWM_MIN  -6900


#define Default_Velocity					350			//默认遥控速度，单位mm/s
#define Default_Turn_Bias					Pi/4			//默认遥控速度，单位度/s
#define Bluetooth_Turn_Angle				Pi/20		//阿克曼车蓝牙默认转向角度，单位rad
#define PS2_Turn_Angle						Pi/4		//阿克曼车手柄默认转向角度，单位rad

#define forward_velocity 0.25  //Move_X速度
#define Along_Angle      282    //参照物的方向

#define FILTERING_TIMES 					10			//滑动滤波

//小车各模式定义
#define Normal_Mode							0
#define Lidar_Avoid_Mode					1
#define Lidar_Follow_Mode					2
#define Lidar_Along_Mode				    3		
#define ELE_Line_Patrol_Mode				4
#define CCD_Line_Patrol_Mode				5
#define ROS_Mode				            6
#define Measure_Distance_Mode				7			//默认没有使用

#define Follow_Distance 1600  //跟随距离
#define Keep_Follow_Distance 400  //跟随保持距离

#define Aovid_Speed 0.3        //避障速度

//指示遥控控制的开关
#define RC_ON								1	
#define RC_OFF								0
//遥控控制前后速度最大值
#define MAX_RC_Velocity						800
//遥控控制转向速度最大值
#define	MAX_RC_Turn_Bias					1
//遥控控制前后速度最小值
#define MINI_RC_Velocity					210
//遥控控制转向速度最小值
#define	MINI_RC_Turn_Velocity			    Pi/20

//前进加减速幅度值，每次遥控加减的步进值
#define X_Step								25
//转弯加减速幅度值
#define Z_Step								0.1

//车轮直径
#define Diff_Car_Wheel_diameter				0.0670f			//差速车和阿克曼车，单位m
#define Small_Tank_WheelDiameter			0.0430f			//小履带车
#define Big_Tank_WheelDiameter				0.0440f			//大履带车

//车轮轮距
#define Diff_wheelspacing					0.177			//差速车轮距
#define Akm_wheelspacing					0.162		//阿克曼车轮距
#define Small_Tank_wheelspacing				0.1350		//小履带车轮距
#define Big_Tank_wheelspacing				0.2330			//大履带车轮距

#define Akm_axlespacing           			0.144f			//阿克曼车轴距
#define Diff_axlespacing                    0.155f           //差速车轴距


#define Gear_Ratio							30.0f			//电机的减速比


#define Pi									3.14159265358979f	//圆周率
#define Angle_To_Rad						57.295779513f	//角度制转弧度制，除以这个参数
#define Frequency							200.0f			//每5ms读取一次编码器的值
#define SERVO_INIT 							1500  			//舵机零点PWM值


#define Encoder_resolution_Photoelectric	500.0f			//光电编码器500线
#define Encoder_resolution_Hall 			13.0f			//霍尔编码器13线
#define Encoder_resolution 					Encoder_resolution_Hall		//使用13线霍尔编码器

#define Angle_TO_PWM						640.62f			//用于计算pwm和角度的关系

#define Normal								0				//检测异常状态，0为正常
#define Abnormal							1


#define Lidar_Detect_ON						1				//电磁巡线是否开启雷达检测障碍物
#define Lidar_Detect_OFF					0

//#define forward_velocity 250 //小车初始速度

//编码器数据读取频率
#define   CONTROL_FREQUENCY 100
#define   Encoder_precision 4*Encoder_resolution_Hall*Gear_Ratio
//电机速度控制相关参数结构体
typedef struct  
{
	float Current_Encoder;     	//编码器数值，读取电机实时速度
	float Motor_Pwm;     		//电机PWM数值，控制电机实时速度
	float Target_Encoder;  		//电机目标编码器速度值，控制电机目标速度
	float Velocity; 	 		//电机速度值
}Motor_parameter;

//编码器结构体
typedef struct  
{
  int A;      
  int B;  
}Encoder;


extern u16 mini_distance1;
extern float angle1;
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern float Move_X,Move_Z;
extern float PWM_Left,PWM_Right;
extern float RC_Velocity,RC_Turn_Velocity;

extern Motor_parameter MotorA,MotorB;	


extern float CCD_Move_X;	
extern Encoder OriginalEncoder;
extern float total_distance;      // ?????
extern float segment_distance;    // ?????  
extern float current_speed;       // ????


void Bluetooth_Control(void);

void Get_Target_Encoder(float Vx,float Vz);
void Get_Motor_PWM(void);
void Get_Velocity_From_Encoder(void);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

float PWM_Limit(float IN,float max,float min);
u8 Turn_Off(void);
float Mean_Filter_Left(float data);
float Mean_Filter_Right(float data);

// ?????????
typedef struct {
    float total_distance;    // 总距离
    float segment_distance;  // 段距离
    float lateral_error;     // 横向偏差
    float heading_angle;     // 航向角
    u8 current_segment;      //阶段
} Odometry_t;

extern Odometry_t odometry;


// ????
void Update_Odometry(void);
float Keep_Straight_PID(void);
void Go_Distance(float target_distance);
void Reset_Odometry(void);


float Rotate_To_Angle(float target_angle);

void Car_Perimeter_Init(void);
void Calculate_Distance_5ms(void);
void Get_Angle(u8 way);
extern u8 turn_flag;
extern u8 turn;
void CountSensor_Init(void);

void EXTI2_IRQHandler(void);
extern volatile uint8_t stop_flag;
extern volatile uint8_t count_times ;
extern volatile uint8_t blackline_trigger;

extern volatile uint8_t rotate_done ;
extern volatile uint8_t rotate_done1 ;
extern volatile uint8_t rotate_done2 ;
extern volatile uint8_t rotate_done3 ;
extern volatile uint8_t rotate_done4 ;
extern volatile uint8_t rotate_done5 ;
extern volatile uint8_t rotate_done6 ;
extern volatile uint8_t rotate_done7 ;

extern u32 timeren;

void Read_All_Sensors(uint8_t *sensor_states);
void GraySensor_TrackSimple(int base_speed);
uint8_t get_needturn(void);
uint8_t get_rotate_done(void);

void set_rotate_done(void);

uint16_t CountSensor_Get(void);
void Heading_Hold_Init(void);
void Heading_Hold_Run(int base_speed);
float Straight_Line_Keep(void);

void Rotate_Fixed_Angle(float delta_angle);
void Rotate_Fixed_Angle1(float delta_angle);

extern float line_last_error;
extern float line_integral;

#endif
