#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"
#include "packets.h"

#define DEBUG_TX_BUFF_SIZE 	500

extern uint8_t debug_tx_buff[DEBUG_TX_BUFF_SIZE];
extern volatile uint16_t debug_tx_index_rd,debug_tx_index_wr,debug_tx_count;

void debug_config();
void queue_data();
void debug_put_char(char data);

#endif
