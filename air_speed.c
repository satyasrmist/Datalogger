#include "air_speed.h"

float air_speed = 0,ASI_press = 0;
uint16_t ASI_ZERO = 0;

//Calaculate air speed offset
void zero_air_speed_calculate(void)
{
	uint8_t count;
	uint32_t temp_val = 0;

    for(count = 0; count < 100; count++)			//sum up 250 samples
    {
    	IWDG_ReloadCounter();
    	delay(20);
    	temp_val += ADCConvertedValue[2]*ADC_5_SCALE_INV;
    }
    ASI_ZERO = temp_val/100;				//average the samples to obtain calibrated value
    store_offsets();						//write values to FLASH
    send_message(CALIBRATE_AIR_SPEED_ID);	//send calibrated message
}

//Update air speed
void air_speed_indicator()
{
	uint16_t ASI_RAW;													//maximum ~140 Kmph of airspeed
	ASI_RAW = ADCConvertedValue[2]*ADC_5_SCALE_INV;						//obtain raw ADC value and scale to make up for voltage divider network
	ASI_press = 0.2*(( ASI_RAW - ASI_ZERO)*0.806) + 0.8*ASI_press;		//subtract the offset, convert RAW value to voltage and apply a filter
	if(ASI_press<0)														//discard negative Air pressure and return
	{
		air_speed = 0;
		return;
	}
	air_speed = sqrt(ASI_press*2/AIR_DENSITY);							//convert pressure to air_speed
}
