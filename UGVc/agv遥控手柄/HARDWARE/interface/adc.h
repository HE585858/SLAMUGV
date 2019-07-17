#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

/*********���ţ�����ң�м���ѹ�ɼ��˿ں궨��*********/
 
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




//��ʼ��ADC��ʹ��DMA����
void Adc_Init(void);
void ADC_Filter(uint16_t* adc_val);	//ADC��ֵ�˲�
uint16_t getAdcValue(uint8_t axis);

#endif 
