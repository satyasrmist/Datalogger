#ifndef RPM_H
#define RPM_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#define MOTOR_POLES	12

extern volatile uint16_t pulse_cnt;		//max 16000 counts in 100ms, so about 16000*10*60 = 9600000RPM
extern volatile uint8_t done_flag;
extern uint32_t rpm;

void RPM_config();


#endif
