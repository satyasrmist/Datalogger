#include "RPM.h"

volatile uint16_t pulse_cnt = 0;		//max 16000 counts in 100ms, so about 16000*10*60 = 9600000RPM
volatile uint8_t done_flag = 0;
uint32_t rpm = 0;

void RPM_config()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_Init(GPIOF, &GPIO_InitStructure);

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource15);

	  /* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	  EXTI_ClearITPendingBit(EXTI_Line15);

	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line15) != RESET )
	{
		pulse_cnt++;
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}
