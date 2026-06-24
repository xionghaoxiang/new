#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"



//单片机最大测量电压3.3V
#define Max_Voltage   				3.3f
//ADC读取最大数值4095
#define Max_Voltage_ADC				4095
//电池电压与读取电压的比例，11:1
#define Ratio 						11
//车型选择ADC电压最大是3.3/2V 
#define Max_Car_ADC					2047
//一共四个车
#define Num_Of_Car					4


// ADC 编号选择
// 可以是 ADC1/2，如果使用ADC3，中断相关的要改成ADC3的

/**************************电压检测ADC，ADC2*****************************/
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADCx                          ADC2
#define    ADC_CLK                       RCC_APB2Periph_ADC2

// ADC GPIO宏定义
// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    ADC_GPIO_CLK                  RCC_APB2Periph_GPIOC  
#define    ADC_PORT                      GPIOC
#define    ADC_PIN                       GPIO_Pin_1
// ADC 通道宏定义
#define    ADC_CHANNEL                   ADC_Channel_11
#define Battery_Ch 11



extern __IO uint16_t ADC_ConvertedValue;



void Voltage_ADC_Init(void);
u16 Get_Voltage(void);
u16 Get_Adc2(u8 ch);  

#endif /* __ADC_H */

