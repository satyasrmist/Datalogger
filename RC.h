#ifndef RC_H_
#define RC_H_

#include "stm32f4xx.h"
#include "modem.h"
#include "loops.h"

extern uint16_t PWM_out_channel[12];
extern uint16_t RC_in_sticks[8];		//valid range 0-7, subtract 1 from PWM channel to obtain value
extern uint16_t RC_trim[8];

void update_sticks();
void set_trims();
uint16_t get_trim_val(uint8_t channel);	//VALID range 1-8


#endif
