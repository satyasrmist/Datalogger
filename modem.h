#ifndef MODEM_H
#define MODEM_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "arduimu.h"
#include "RC.h"
#include "log.h"
#include "air_speed.h"

#define XBEE_RX_BUFF_SIZE 	1000
#define XBEE_TX_BUFF_SIZE 	1000
#define XBEE_BUFF_SIZE		100

//MESSAGES
#define CALIBRATE_AIR_SPEED_MSG_ID		36
#define CALIBRATE_IMU_MSG_ID			37
#define CALIBRATE_MAG_MSG_ID			38
#define CALIBRATE_ALTITUDE_MSG_ID		39
#define LOG_INIT_MSG_ID					40
#define SET_TRIMS_MSG_ID				41
#define EXCITATION_SET_MSG_ID			42

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW	35
#define CALIBRATE_AIR_SPEED_ID			36
#define CALIBRATE_IMU_ID				37
#define CALIBRATE_MAG_ID				38
#define CALIBRATE_ALTITUDE_ID			39
#define LOG_INIT_ID						40
#define SET_TRIMS_ID					41
#define TRIGGER_EXCITATION_ID			42
#define SET_HEADING_ID					43
#define SET_EXCITATION_ID				44
#define REBOOT_SYSTEM_ID				45
#define FIRMWARE_REQUEST_ID				46

extern char xbee_rx_buff[XBEE_RX_BUFF_SIZE];
extern uint8_t xbee_tx_buff[XBEE_TX_BUFF_SIZE];
extern volatile uint16_t xbee_rx_index_wr, xbee_rx_index_rd;
extern uint16_t xbee_rx_count;
extern volatile uint16_t xbee_tx_index_rd,xbee_tx_index_wr,xbee_tx_count;

extern uint16_t CRC_Accum_Rx;
extern char xbee_packet_set;
extern char xbee_packet_size0;
extern char xbee_fresh_data;
extern uint8_t xbee_buff_index;
extern volatile char xbee_buff[XBEE_BUFF_SIZE];
extern uint8_t modem_link_active,link_count;

void modem_config();
void xbee_put_char(char data);
void parse_modem_data();
void CRC_accumulate_rx(uint8_t data, uint16_t *crcAccum);
void calculate_xbee();

#endif
