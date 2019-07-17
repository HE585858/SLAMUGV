#include "Hardware.h"
#include "IOConfig.h"


void GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);//

  //USART1 
  GPIO_InitStructure.GPIO_Pin = USART1_TX | USART1_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����                    
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB
	
  //USART2 ������
  GPIO_InitStructure.GPIO_Pin = USART2_TX | USART2_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOD
	
  //USART3 
  GPIO_InitStructure.GPIO_Pin = USART3_TX | USART3_RX;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB
	
	//USART6 
  GPIO_InitStructure.GPIO_Pin = USART6_TX | USART6_RX;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC
	
	//�����������
  GPIO_InitStructure.GPIO_Pin = LMOTORF_GPIO1	 | LMOTORF_GPIO2	|	LMOTORB_GPIO1	 | LMOTORB_GPIO2 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA
	GPIO_ResetBits(GPIOA,LMOTORF_GPIO1	|	LMOTORF_GPIO2	|	LMOTORB_GPIO1	|	LMOTORB_GPIO2	);
	
	GPIO_InitStructure.GPIO_Pin = RMOTORF_GPIO1	 | RMOTORF_GPIO2	|	RMOTORB_GPIO1	 | RMOTORB_GPIO2 ;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE  
  GPIO_ResetBits(GPIOE,RMOTORF_GPIO1	|	RMOTORF_GPIO2	|	RMOTORB_GPIO1	|	RMOTORB_GPIO2	);
	
	//���PWM TIM3_CH3 TIM3_CH4
	GPIO_InitStructure.GPIO_Pin = LMOTORF_PWM	 | LMOTORB_PWM ;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOE  
  GPIO_ResetBits(GPIOB,LMOTORF_PWM	 | LMOTORB_PWM	);
	
	
  //���PWM TIM9_CH1 TIM9_CH2
	GPIO_InitStructure.GPIO_Pin = RMOTORF_PWM	 | RMOTORB_PWM ;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//500MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE  
  GPIO_ResetBits(GPIOE,RMOTORF_PWM	 | RMOTORB_PWM	);
  
  //��������� PA0 PA1 PA6 PA7	PA9
	GPIO_InitStructure.GPIO_Pin = LMOTORF_ENCONDER1	 
															| LMOTORB_ENCONDER1	
															| LMOTORB_ENCONDER2	
															|	RMOTORF_ENCONDER1	
															|	RMOTORF_ENCONDER2;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA 
//  GPIO_ResetBits(GPIOA,LMOTORF_ENCONDER1	 
//															| LMOTORB_ENCONDER1	
//															| LMOTORB_ENCONDER2	
//															|	RMOTORF_ENCONDER1	
//															|	RMOTORF_ENCONDER2	);
															
	GPIO_PinAFConfig(GPIOA,LMOTORF_ENCONDER1,GPIO_AF_TIM1); 	//GPIOA9���ó�TIM1_CH1
  GPIO_PinAFConfig(GPIOA,LMOTORB_ENCONDER1,GPIO_AF_TIM2); 	//GPIOA9���ó�TIM2_CH1
	GPIO_PinAFConfig(GPIOA,LMOTORB_ENCONDER2,GPIO_AF_TIM2); 	//GPIOA9���ó�TIM2_CH2
	GPIO_PinAFConfig(GPIOA,RMOTORF_ENCONDER1,GPIO_AF_TIM3); 	//GPIOA9���ó�TIM3_CH1
	GPIO_PinAFConfig(GPIOA,RMOTORF_ENCONDER2,GPIO_AF_TIM3); 	//GPIOA9���ó�TIM3_CH2
	
	//��������� PE9
	GPIO_InitStructure.GPIO_Pin = LMOTORF_ENCONDER2 ;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE  
//  GPIO_ResetBits(GPIOE,LMOTORF_ENCONDER2	);
	
	GPIO_PinAFConfig(GPIOE,RMOTORF_ENCONDER2,GPIO_AF_TIM1); 	//GPIOE9���ó�TIM1_CH2
	
	//��������� PD12 PD13
	GPIO_InitStructure.GPIO_Pin = RMOTORB_ENCONDER1	 | RMOTORB_ENCONDER2 ;															;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����    
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOD  
//  GPIO_ResetBits(GPIOD,RMOTORB_ENCONDER1	 | RMOTORB_ENCONDER2	);

  GPIO_PinAFConfig(GPIOD,RMOTORB_ENCONDER1,GPIO_AF_TIM4); 	//GPIOD12���ó�TIM4_CH1
	GPIO_PinAFConfig(GPIOD,RMOTORB_ENCONDER2,GPIO_AF_TIM4); 	//GPIOD13���ó�TIM4_CH2
}

void TIMX_Config(u16 arr,u16 psc)
{ 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); 
		/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = arr;//��ʱ������
	TIM_TimeBaseStructure.TIM_Prescaler = psc;//Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	 /*LB PWMģʽ����*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = LBP1_duty	;      //���ô�װ�벶��ȽϼĴ���������ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��3
	
	/*LF PWMģʽ����*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = LFP2_duty	;      //���ô�װ�벶��ȽϼĴ���������ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��4
	
	/*RB PWMģʽ����*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = RBP1_duty	;    //���ô�װ�벶��ȽϼĴ���������ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
  TIM_OC1Init(TIM9, &TIM_OCInitStructure);	 //ʹ��ͨ��1
	
	/*RF PWMģʽ����*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = RFP2_duty	;    //���ô�װ�벶��ȽϼĴ���������ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);	 //ʹ��ͨ��2
	
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE �����ʹ��
  TIM_CtrlPWMOutputs(TIM9,ENABLE);	//MOE �����ʹ��


	/*ʹ��ͨ������*/
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIM3��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM9, ENABLE); //ʹ��TIM9��ARR�ϵ�Ԥװ�ؼĴ���
	
	
	
	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM3, ENABLE);	
	TIM_Cmd(TIM9, ENABLE);	
}
void Encoder_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ�ܶ�ʱ��1��ʱ��	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ�ܶ�ʱ��3��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ�ܶ�ʱ��4��ʱ��
	
	//TIM1 �����ڸ߼���ʱ��
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 			// Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ��� 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//�ظ���������	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM1,0);
	TIM_Cmd(TIM1, ENABLE); 
 
 //TIM2
 
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);		//
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 			// Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); 
 
 //TIM3
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE); 

//TIM4
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		 case 1:  Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;
		 case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/**************************************************************************
�������ܣ���ʱ���жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM1_IRQHandler(void) 
{ 	    	  	     
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //���TIM1���жϴ�����λ:TIM �ж�Դ 
 }	     
} 

void TIM2_IRQHandler(void)
{ 		    		  			    
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //���TIM2���жϴ�����λ:TIM �ж�Դ 
 }	     
}

void TIM3_IRQHandler(void)
{ 		    		  			    
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //���TIM3���жϴ�����λ:TIM �ж�Դ 
 }	     
}


void TIM4_IRQHandler(void)
{ 		    		  			    
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //���TIM4���жϴ�����λ:TIM �ж�Դ 
 }	   
}



//void USART_Config(USART_TypeDef* USARTx, u32 rate)
//{ 
//	
//  USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate = rate;						//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
//	USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ
//	/* Configure USARTx */
//	USART_Init(USARTx, &USART_InitStructure);							//���ô��ڲ�������
//	/* Enable USART1 Receive and Transmit interrupts */
//	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                      //ʹ�ܽ����ж�
//	USART_Cmd(USARTx, ENABLE);
//	USART_ClearFlag(USARTx, USART_FLAG_TC);
//}

//void NVIC_Configuration(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure;

//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //Ƕ�������жϿ�������ѡ�� 

//  
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//���ô���1�ж�
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//�������ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ж�
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			     	//���ô���2�ж�
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//���ô���3�ж�
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;			     	//���ô���5�ж�
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;			     	//���ô���4�ж�
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//	//NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռʽ�ж����ȼ�����Ϊ1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //��Ӧʽ�ж����ȼ�����Ϊ1
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        //ʹ���ж�
//	NVIC_Init(&NVIC_InitStructure);

//}


