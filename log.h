#ifndef LOG_H_
#define LOG_H_

#include "stm32f4_discovery_sdio_sd.h"
#include "ff.h"
#include "string.h"
#include "stdio.h"
#include "gps.h"
#include "packets.h"
#include "portmacro.h"

/*	One to read internal sensor and Zero to read data from ext. port					*/
#define dataFromInternalSensors	0

#define LOG_BUFF_SIZE 5120		//filled in about 250-260ms, 383bytes each

#define GPS_LOG_RATE		5
#define PRESS_TEMP_RATE		3

FRESULT res;
FILINFO fno;
FIL fil;
FATFS fs32;
char* path;
UINT bw;
extern char file_name[];

extern volatile DWORD cursor_pos;
extern volatile uint8_t write_buff_to_card;
extern char *LOG_BUFF;
extern volatile uint8_t log_buff_status;		//keep track of which buffer you are writing too
extern volatile uint8_t log_init_flag;
extern uint32_t log_loop_count;

typedef struct {
	char BUFFER[LOG_BUFF_SIZE];
	volatile uint16_t	write_count;
}log_buffer;
extern log_buffer SD_buffer1,SD_buffer2;

extern portTickType xTaskGetTickCount( void );

void create_log_file();

//Check which data is to be written and call the function to append it to the active buffer
void log_data();

//Append incoming data to the active buffer, check buffer boundary and switch accordingly
void append_byte_to_array(char array[],uint8_t bytes_to_append);

#endif
