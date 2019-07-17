#include "sys.h"

#define USART1_TX		GPIO_Pin_6 //PB6 复用
#define USART1_RX   GPIO_Pin_7 //PB7


#define USART2_TX		GPIO_Pin_5 //PD5 复用
#define USART2_RX   GPIO_Pin_6 //PD6

#define USART3_TX		GPIO_Pin_10 //PB10
#define USART3_RX   GPIO_Pin_11 //PB11

#define USART6_TX		GPIO_Pin_6 //PC6
#define USART6_RX   GPIO_Pin_7 //PC7

#define LMOTORF_PWM  GPIO_Pin_0 //PB0
#define LMOTORB_PWM  GPIO_Pin_1 //PB1
#define RMOTORF_PWM  GPIO_Pin_5 //PE5
#define RMOTORB_PWM  GPIO_Pin_6 //PE6

#define LMOTORF_ENCONDER1		GPIO_Pin_9 //PA9
#define LMOTORF_ENCONDER2		GPIO_Pin_9 //PE9
#define LMOTORB_ENCONDER1		GPIO_Pin_0 //PA0
#define LMOTORB_ENCONDER2		GPIO_Pin_1 //PA1
#define RMOTORF_ENCONDER1		GPIO_Pin_6 //PA6
#define RMOTORF_ENCONDER2		GPIO_Pin_7 //PA7
#define RMOTORB_ENCONDER1		GPIO_Pin_12 //PD12
#define RMOTORB_ENCONDER2		GPIO_Pin_13 //PD13

#define LMOTORF_GPIO1		GPIO_Pin_12 //PA12  左前电机前进
#define LMOTORF_GPIO2		GPIO_Pin_8 	//PA8   左前电机后退 


#define LMOTORB_GPIO1		GPIO_Pin_10	//PA10 左后电机前进
#define LMOTORB_GPIO2		GPIO_Pin_11 //PA11 左后电机后退

#define RMOTORF_GPIO1		GPIO_Pin_13 //PE13 右前电机前进
#define RMOTORF_GPIO2		GPIO_Pin_12 //PE12 右前电机后退

#define RMOTORB_GPIO1		GPIO_Pin_11 //PE11    右后电机前进
#define RMOTORB_GPIO2		GPIO_Pin_10 //PE10    右后电机后退




#define LBP1_duty	  5000		//占空比为5000/10000 = 50%		频率：7.2KHz
#define LFP2_duty		5000		//占空比为5000/10000 = 50%
#define RBP1_duty		5000		//占空比为5000/10000 = 50%
#define RFP2_duty		5000		//占空比为5000/10000 = 50%

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为定时器是16位的�




