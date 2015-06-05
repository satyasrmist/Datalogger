#ifndef LOOPS_H_
#define LOOPS_H_

#include "stm32f4xx.h"
#include "PWM_output.h"
#include "arduimu.h"

#define Kp_ROLL		7
#define Kd_ROLL 	0
#define Kscale_ROLL 	1

#define Kp_HEAD 	1
#define K_ROLL 		0

#define ROLL_LIMIT 40
extern float roll_target, heading_target;
extern uint8_t flight_mode;			//0-Manual, 1 -Stabilize/Loops

extern int16_t excitation_offset;
extern uint16_t excitation_time;	//ms
extern uint8_t excitation_state;
extern uint16_t excitation_time_elapsed;
extern uint32_t excitation_time_old;
extern uint8_t excitation_channel;	//valid range 1-8 corresponding to PWM channels
extern uint8_t excitation_type;		//0-undefined,1- undefined, 2- doublet

//Computes and sets the PWM servo outputs
void innerloop();

void set_target_heading();

//Estimates the roll correction/target to hold the set heading target
void heading_hold(int);

//Computes the Aileron output based on current roll and send corrective output to the Aileron
void roll_hold(float roll_target);

void set_excitation();

//Initiate excitation
void trigger_excitation();

void carry_out_excitation();
#endif
