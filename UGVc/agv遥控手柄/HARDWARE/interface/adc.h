#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

/*********油门，方向遥感及电压采集端口宏定义*********/
 
 #define ADC_APB2_GPIOA      RCC_APB2Periph_GPIOA

 
 #define ADC_APB2_ADC1       RCC_APB2Periph_ADC1
 #define ADC_APB2_DMA1       RCC_AHBPeriph_DMA1
 
 #define ADC_GPIOA           GPIOA
 #define ADC_GPIOB           GPIOB
 
 /*******GPIOA******/
 #define ADC_YAW_PIN         GPIO_Pin_2
 #define ADC_THRUST_PIN      GPIO_Pin_3
 
 /*****GPIOA*******/
 #define ADC_BAT_PIN         GPIO_Pin_0
 #define ADC_ROLL_PIN        GPIO_Pin_1  
 #define ADC_PITCH_PIN       GPIO_Pin_2
 
 
 
 #define ADC_ADC1            ADC1



#define  ADC_BAT		   0
#define  ADC_ROLL		   1
#define  ADC_PITCH	 	 2
#define  ADC_YAW		   3
#define  ADC_THRUST		 4




//初始化ADC，使用DMA传输
void Adc_Init(void);
void ADC_Filter(uint16_t* adc_val);	//ADC均值滤波
uint16_t getAdcValue(uint8_t axis);

#endif 
