#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "RC.h"

#define AILERON		1
#define ELEVATOR	2
#define THROTTLE	3
#define RUDDER		4

#define SERVO_MAX 	2000
#define SERVO_MIN	1000

void PWM_out_config();
void check_servo_limits();
void PWM_Set_Duty_Cycle(u8 channel,u16 duty_cycle);

#endif
