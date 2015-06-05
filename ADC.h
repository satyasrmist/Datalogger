#ifndef ADC_H
#define ADC_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"

#define ADC1_DR_Address 	((uint32_t)0x4001204C)		//ADC1 address, refer datasheet
#define ADC3_DR_Address		((uint32_t)0x4001224C)		//ADC1 address, refer datasheet

#define VREF				3.38						//ADC reference voltage
#define ADC_5_SCALE			1200/(1200+620)				//Voltage divider network
#define ADC_5_SCALE_INV		(22000+10000)/22000

extern uint16_t ADCConvertedValue[13];		//store ADC1 converted values
extern uint16_t ADC_3_ConvertedValue[8];	//store ADC3 converted values

//Configure ADC1
void ADC_config();

//Configure ADC3
void ADC_3_config();	//needs regular ADC config for timer


#endif
