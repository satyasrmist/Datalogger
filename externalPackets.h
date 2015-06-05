/*
 * externalPackets.h and externalpackets.c
 * purpose is to able to read external packets from serial port
 * and log data on SD card.
 *
 *
 * For the experiment purpose it is tested with x-plane sim. data via
 * ext. IMU port and log on to SD card.
 *
 *
 * Steps to comment this functionality
 * 1. comment #define dataFromInternalSensors from log.h
 * 2. comment externalDataPortInit() and externalPackets.h from main
 * 3. comment 1st else conditions log_data() in log.c
 */

#ifndef EXTERNALPACKETS_H
#define EXTERRNALPACKETS_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"


#define RXBUFFERSIZE 50
#define TXBUFFERSIZE 50
#define BUFFERSIZE 22

extern volatile uint8_t UART4RxBuffer[RXBUFFERSIZE], UART4TxBuffer[TXBUFFERSIZE];
extern volatile uint8_t UART4RxRdIndex, UART4TxRdIndex, UART4RxCount, UART4TxCount;
extern volatile uint8_t UART4RxWrIndex, UART4TxWrIndex, receivedBufferIndex, packetReadStart;
extern uint8_t maxBufferSize;
extern uint8_t extBuffer[BUFFERSIZE];


	extern float Roll;
	extern float Pitch;
	extern float Yaw;
	extern float RollRate;
	extern float PitchRate;
	extern float YawRate;
	extern float GPSLAT;
	extern float GPSLONG;
	extern float GPSAlt;
	extern float Speed;
	extern float Heading;
	extern uint8_t GPSFix;
	extern uint8_t GPSSAT;
	extern float GPSAirSpeed;
	extern float BaroALT;



void externalDataPortInit(void);
void UART4Init(void);
void UART4_IRQHandler(void);
void ParseExtDataByte(void);
char ExtGetChar(void);
void CalculateExtIMU(void);
void CalculateExtGPS(void);
#endif
