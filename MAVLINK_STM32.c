#include "MAVLINK_STM32.h"

uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
uint8_t data_start_index = 0;

//MAVLink wrapper function
int16_t mavlink_serial_send(mavlink_channel_t chan, uint8_t buf[], uint16_t len)
// Note: Channel Number, chan, is currently ignored.
{
	// Note at the moment, all channels lead to the one serial port
/*******************************************************************/

	data_start_index = 0;

/*******************************************************************/
		memcpy(&serial_buffer[data_start_index], buf, len);
/*************************Asteria Function**************************/

		empty_serial_buffer(len);

/*******************************************************************/
	return (1);
}

void empty_serial_buffer(uint8_t len)
{
	uint8_t temp;

	while(len)
	{
		temp = serial_buffer[data_start_index++];
		xbee_put_char(temp);
		len--;
	}
}
