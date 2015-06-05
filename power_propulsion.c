#include "power_propulsion.h"

float battery_V;
uint32_t motor_RPM;
float motor_current = 0;

void update_battery()
{

/******************************ORIGINAL V/I PORTS******************************/
//	motor_current = (MOTOR_CURRENT_WEIGHT)*((ADCConvertedValue[1]*ADC_5_SCALE_INV*VREF*1000/4096.00)/18.30) + (1-MOTOR_CURRENT_WEIGHT)*motor_current;		//original current port. Current sensor sensitivity 18.30mV/A
//	battery_V = ADCConvertedValue[0]*ADC_5_SCALE_INV*VREF*1000/4096.00;			//original voltage port
/******************************************************************************/

//LIPO supply voltage
	battery_V = ADC_3_ConvertedValue[7]*7.8*VREF/4096.00;		//obtain raw ADC value and scale to obtain final value, voltage divider of 68/10K - 7.8 = ((68000+10000)/10000)

//ADC 3.3V spare channel 2
	motor_current = (MOTOR_CURRENT_WEIGHT)*((ADC_3_ConvertedValue[4]*VREF*1000/4096.00)/18.30) + (1-MOTOR_CURRENT_WEIGHT)*motor_current;		//current sensor sensitivity 18.30mV/A
}

void update_RPM()
{
	motor_RPM = MOTOR_RPM_WEIGHT*rpm + (1-MOTOR_RPM_WEIGHT)*motor_RPM;
}
