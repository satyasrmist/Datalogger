#include "RC.h"

uint16_t PWM_out_channel[12];
uint16_t RC_in_sticks[8];		//valid range 0-7, subtract 1 from PWM channel to obtain value
uint16_t RC_trim[8];

void update_sticks()
{
	uint8_t count;
	union
	{
		uint16_t val;
		struct
		{
			uint8_t bytes[2];
		};
	}extract_uint16;

	for(count = 0; count <8; count++)
	{
		extract_uint16.bytes[0] = xbee_buff[count*2 + 11];
		extract_uint16.bytes[1] = xbee_buff[count*2 + 12];

		RC_in_sticks[count] = extract_uint16.val;
	}
	link_count = 0;		//reset counter
	modem_link_active = 1;
	if((RC_in_sticks[4] > 1710) || (RC_in_sticks[4] < 1000))
		flight_mode = 0;
	else if((RC_in_sticks[4] > 1400) && (RC_in_sticks[4] < 1600))
		flight_mode = 1;		//stabilize
	else
		flight_mode = 2;		//auto heading
}

void set_trims()
{
	uint8_t i;

	for(i=0;i<8;i++)
	{
		RC_trim[i] = RC_in_sticks[i];
	}
	store_offsets();
	send_message(SET_TRIMS_MSG_ID);
}

uint16_t get_trim_val(uint8_t channel)
{
	return RC_trim[channel -1];
}
