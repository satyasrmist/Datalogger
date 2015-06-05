#ifndef POWER_PROPULSION_H_
#define POWER_PROPULSION_H_

#include "ADC.h"
#include "packets.h"

#define MOTOR_CURRENT_WEIGHT 	0.3
#define MOTOR_RPM_WEIGHT		0.8

extern float battery_V;
extern uint32_t motor_RPM;
extern float motor_current;

void update_battery();
void update_RPM();

#endif
