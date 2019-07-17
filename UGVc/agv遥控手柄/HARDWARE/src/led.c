#include "led.h"

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

/* LED��ʼ�� */
void ledInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	/* ��ֹJTAʹ��SWD�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	/* ��ʼ��LED_BLUE(PB9)*/
	RCC_APB2PeriphClockCmd(LED_APB2_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN  ;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_GPIOB, &GPIO_InitStructure);
	
		/* ��ʼ�� LED_RED(PC13) */
	RCC_APB2PeriphClockCmd(LED_APB2_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN  ;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_GPIOC, &GPIO_InitStructure);
	
	/* �ر�����LED */
	GPIO_SetBits(LED_GPIOB,LED_BLUE);			
	GPIO_SetBits(LED_GPIOC,LED_RED);
}
 
