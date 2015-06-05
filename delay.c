#include "delay.h"


void delay(int time)		//count 2000 approx 250ms delay
{
	volatile uint32_t i=0;
	for(i=0; i<1000*time; i++);
}


