#include "PWM_output.h"

void PWM_out_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_TIM9, ENABLE);

	/* GPIOC and GPIOB clock enable */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM3 pins to AF2 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 20000;		//20ms
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) / 1000000) - 1;		//1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 40000;		//20ms		//to maintain 20ms period
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	/* PWM Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	/* PWM Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_ARRPreloadConfig(TIM9, ENABLE);

	TIM_CtrlPWMOutputs(TIM1,ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM9, ENABLE);
}

void check_servo_limits()
{
	uint8_t i;
	for(i=0;i<12;i++)
	{
		if(PWM_out_channel[i] > SERVO_MAX)
			PWM_out_channel[i] = SERVO_MAX;
		else if(PWM_out_channel[i] < SERVO_MIN)
			PWM_out_channel[i] = SERVO_MIN;
	}
}

void PWM_Set_Duty_Cycle(u8 channel,u16 duty_cycle)
{
	if(duty_cycle > SERVO_MAX)
		duty_cycle = SERVO_MAX;
	else if(duty_cycle < SERVO_MIN)
		duty_cycle = SERVO_MIN;

	if(channel == 1)
	{
		PWM_out_channel[0] = duty_cycle;
		TIM_SetCompare1(TIM3, PWM_out_channel[0]);
	}
	else if(channel == 2)
	{
		PWM_out_channel[1] = duty_cycle;
		TIM_SetCompare2(TIM3, PWM_out_channel[1]);
	}
	else if(channel == 3)
	{
		PWM_out_channel[2] = duty_cycle;
		TIM_SetCompare1(TIM9, PWM_out_channel[2]*2);
	}
	else if(channel == 4)
	{
		PWM_out_channel[3] = duty_cycle;
		TIM_SetCompare2(TIM9, PWM_out_channel[3]*2);
	}
	else if(channel == 5)
	{
		PWM_out_channel[4] = duty_cycle;
		TIM_SetCompare1(TIM1, PWM_out_channel[4]*2);
	}
	else if(channel == 6)
	{
		PWM_out_channel[5] = duty_cycle;
		TIM_SetCompare2(TIM1, PWM_out_channel[5]*2);
	}
	else if(channel == 7)
	{
		PWM_out_channel[6] = duty_cycle;
		TIM_SetCompare3(TIM1, PWM_out_channel[6]*2);
	}
	else if(channel == 8)
	{
		PWM_out_channel[7] = duty_cycle;
		TIM_SetCompare4(TIM1, PWM_out_channel[7]*2);
	}
	else if(channel == 9)
	{
		PWM_out_channel[8] = duty_cycle;
		TIM_SetCompare1(TIM4, PWM_out_channel[8]);
	}
	else if(channel == 10)
	{
		PWM_out_channel[9] = duty_cycle;
		TIM_SetCompare1(TIM4, PWM_out_channel[9]);
	}
	else if(channel == 11)
	{
		PWM_out_channel[10] = duty_cycle;
		TIM_SetCompare1(TIM4, PWM_out_channel[10]);
	}
	else if(channel == 12)
	{
		PWM_out_channel[11] = duty_cycle;
		TIM_SetCompare1(TIM4, PWM_out_channel[11]);
	}
}
