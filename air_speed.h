#ifndef _AIR_SPEED_H
#define _AIR_SPEED_H

#include "stm32f4xx.h"
#include "math.h"
#include "ADC.h"
#include "delay.h"
#include "modem.h"
#include "flash.h"
#include "stm32f4xx_iwdg.h"

#define AIR_DENSITY 1.225
//#define ADC_5_SCALE	2200/(2200+10000)		//Voltage divider network

extern float air_speed,ASI_press;
extern uint16_t ASI_ZERO;

//Calaculate air speed offset
void zero_air_speed_calculate(void);

//Update air speed
void air_speed_indicator();

#endif
