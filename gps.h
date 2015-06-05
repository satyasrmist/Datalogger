#ifndef GPS_H
#define GPS_H

#include "diskio.h"
#include "ublox.h"
#include "modem.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"

#define GPS_RX_BUFF_SIZE 	512
#define GPS_BUFF_SIZE  		255
#define GPS_DATA_SIZE 		20
#define GPS_TX_BUFF_SIZE 	128

#define RE 					6378500   				// radius of earth in meters

extern signed int X, Y;
extern uint32_t CurrentTime;
extern float X1,X2,X3;

extern float X1,X2,X3;
extern long LAT_home, LONG_home;
extern uint32_t old_tow;
extern float nav_dt;
extern float old_speed;

typedef struct {
	int8_t 	gps_fresh_data;
	int8_t 	fix;
	int8_t 	satellites;
	float 	gps_alt;
	float 	gps_speed;
	float 	gps_air_speed;
	float 	gps_heading;
	float 	gps_acc;
	long	LAT,LONG;
	uint32_t mTOW;
	float vDOP;
	float hDOP;
	float pDOP;
}gps_parameters;
extern gps_parameters ublox;

extern char gps_rx_buff[];
extern unsigned int gps_data[];
extern char gps_tx_buff[];

typedef struct {
	uint16_t wr_index;
	uint16_t	rd_index;
	uint16_t	count;
}gps_comm_buffer;
extern gps_comm_buffer gps_tx,gps_rx;

void gps_comm_init();

//Configure GPS serial port
void ublox_init();
void gps_tx_int();
void gps_rx_int(char data);
void gps_put_char(char data);
void gps_puts(uint8_t *buff, uint8_t len);
char gps_get_char();

//Check if GPS data is received, is so, start parsing messages
void parse_gps_data();

//Assign GPS data to the variables used in the rest of the program
void calculate_ublox();		//hardware GPS


#endif
