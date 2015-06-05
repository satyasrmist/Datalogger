#ifndef MAVLINK_STM32_H
#define MAVLINK_STM32_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "../common/common.h"
#include "mavlink_types.h"
#include "modem.h"

//////////////////**************************************************************////////////////

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

//////////////////**************************************************************////////////////

#define	SERIAL_BUFFER_SIZE			MAVLINK_MAX_PACKET_LEN

extern uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
extern uint8_t data_start_index;

extern mavlink_system_t mavlink_system;

//MAVLink wrapper function
int16_t mavlink_serial_send(mavlink_channel_t chan, uint8_t buf[], uint16_t len);
void empty_serial_buffer(uint8_t len);

#endif
