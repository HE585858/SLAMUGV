#include <stdbool.h>
#include "keyTask.h"
/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ����������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/6/1
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define LONG_PRESS_COUNT 	500	/*�ж�Ϊ����ʱ�䣨ms��*/

#define PRESSED		0
#define RELEASED	1

static bool keyL_pressed;
static bool keyR_pressed;
static bool keyJ1_pressed;
static bool keyJ2_pressed;
static u8 keyState;
static u32 pressedTime;

void keyTask(void* param)
{
	while(1)
	{
		vTaskDelay(20);
		
		if(keyL_pressed==false && READ_KEY_1()==PRESSED)
		{
			keyL_pressed = true;
			pressedTime = xTaskGetTickCount();
		}
		if(keyR_pressed==false && READ_KEY_2()==PRESSED)
		{
			keyR_pressed = true;
			pressedTime = xTaskGetTickCount();
		}
		if(keyJ1_pressed==false && READ_KEY_3()==PRESSED)
		{
			keyJ1_pressed = true;
			pressedTime = xTaskGetTickCount();
		}
		if(keyJ2_pressed==false && READ_KEY_4()==PRESSED)
		{
			keyJ2_pressed = true;
			pressedTime = xTaskGetTickCount();
		}
		
		if(keyL_pressed==true)
		{
			if(READ_KEY_1()==RELEASED)
				keyL_pressed = false;
			if((xTaskGetTickCount() - pressedTime) > LONG_PRESS_COUNT)
				keyState = KEY_L_LONG_PRESS;
			else if(READ_KEY_1()==RELEASED)
				keyState = KEY_L_SHORT_PRESS;
		}
		if(keyR_pressed==true)
		{
			if(READ_KEY_2()==RELEASED)
				keyR_pressed = false;
			if((xTaskGetTickCount() - pressedTime) > LONG_PRESS_COUNT)
				keyState = KEY_R_LONG_PRESS;
			else if(READ_KEY_2()==RELEASED)
				keyState = KEY_R_SHORT_PRESS;
		}
		if(keyJ1_pressed==true)
		{
			if(READ_KEY_3()==RELEASED)
				keyJ1_pressed = false;
			if((xTaskGetTickCount() - pressedTime) > LONG_PRESS_COUNT)
				keyState = KEY_J1_LONG_PRESS;
			else if(READ_KEY_3()==RELEASED)
				keyState = KEY_J1_SHORT_PRESS;
		}
		if(keyJ2_pressed==true)
		{
			if(READ_KEY_4()==RELEASED)
				keyJ2_pressed = false;
			if((xTaskGetTickCount() - pressedTime) > LONG_PRESS_COUNT)
				keyState = KEY_J2_LONG_PRESS;
			else if(READ_KEY_4()==RELEASED)
				keyState = KEY_J2_SHORT_PRESS;
		}

	}
}

//��ȡ����״̬
u8 getKeyState(void)
{
	u8 temp;
	temp = keyState;
	keyState = 0;//��ȡ����֮������
	return temp;
}


















