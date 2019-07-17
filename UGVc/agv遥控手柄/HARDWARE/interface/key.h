#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/6/1
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/   	 

#define KEY_APB2_GPIOB  RCC_APB2Periph_GPIOB



#define KEY_GPIOB  GPIOB



#define KEY_1  		GPIO_Pin_14
#define KEY_2  		GPIO_Pin_15
#define KEY_3   	GPIO_Pin_13
#define KEY_4   	GPIO_Pin_12

#define READ_KEY_1()  	GPIO_ReadInputDataBit(GPIOB,KEY_1)	//��ȡKEY1����
#define READ_KEY_2() 	  GPIO_ReadInputDataBit(GPIOB,KEY_2)	//��ȡKEY2����
#define READ_KEY_3()  	GPIO_ReadInputDataBit(GPIOB,KEY_3)	//��ȡKEY3����
#define READ_KEY_4()  	GPIO_ReadInputDataBit(GPIOB,KEY_4)	//��ȡKEY4����

//IO��ʼ��
void keyInit(void);

 //����ɨ�躯��		
void KEY_Scan(void);

#endif



