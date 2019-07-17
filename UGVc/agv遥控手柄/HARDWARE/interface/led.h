#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * LED��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/6/1
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
#define LED_APB2_GPIOB  RCC_APB2Periph_GPIOB
#define LED_APB2_GPIOC  RCC_APB2Periph_GPIOC

#define LED_GPIOB       GPIOB
#define LED_GPIOC       GPIOC



#define LED_BLUE_PIN 		 GPIO_Pin_13
#define LED_RED_PIN	   GPIO_Pin_9

#define LED_BLUE 		PBout(9)
#define LED_RED 		PCout(13)

void ledInit(void);/* LED��ʼ�� */
			    
#endif
