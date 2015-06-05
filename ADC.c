#include "ADC.h"

uint16_t ADCConvertedValue[13];		//store ADC1 converted values
uint16_t ADC_3_ConvertedValue[8];	//store ADC3 converted values

//Configure ADC1
void ADC_config()
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Enable ADC1, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 20000; 			// 20ms
	TIM_TimeBaseStructure.TIM_Prescaler = (84000000 / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);

	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)& ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 13;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Configure ADC1 pins as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

	ADC_InitStructure.ADC_ScanConvMode = ENABLE;

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 13;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 channels configuration *************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 9, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 10, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 11, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 12, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 13, ADC_SampleTime_84Cycles);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
}

//Configure ADC3
void ADC_3_config()		///needs regular ADC config for timer
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	/* DMA2 Stream1 channel2 configuration **************************************/
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)& ADC_3_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream1, ENABLE);

	/* Configure ADC3 pins as analog input *****************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC3 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 8;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 channels configuration *************************************/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 2, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 3, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 4, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 5, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_6, 6, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 7, ADC_SampleTime_84Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 8, ADC_SampleTime_84Cycles);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);
}
