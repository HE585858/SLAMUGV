#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly_Remotor
 * LED驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/6/1
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
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

void ledInit(void);/* LED初始化 */
			    
#endif
