#include "gps.h"

gps_parameters ublox;
gps_comm_buffer gps_tx,gps_rx;

signed int X, Y;
uint32_t CurrentTime;
float X1,X2,X3;

float old_speed=0;
uint32_t old_tow=0;
long LAT_home, LONG_home;
float nav_dt=1;

char gps_rx_buff[GPS_RX_BUFF_SIZE];
char gps_tx_buff[GPS_TX_BUFF_SIZE];
unsigned int gps_data[GPS_DATA_SIZE];

void gps_comm_init()			//re-initialize buffer counters
{
	gps_tx.wr_index=0;
	gps_tx.rd_index=0;
	gps_tx.count=0;
	gps_rx.wr_index=0;
	gps_rx.rd_index=0;
	gps_rx.count=0;
}

//Configure GPS serial port
void ublox_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	gps_comm_init();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 38400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

    _configure_gps();		//set the packet rates and packet types to be sent
}

void USART2_IRQHandler(void)
{
	if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
		gps_rx_buff[gps_rx.wr_index++] = (char)(USART_ReceiveData(USART2));
		gps_rx.count++;
		if(gps_rx.wr_index==GPS_RX_BUFF_SIZE)
		{
			gps_rx.wr_index = 0;
		}
	}
	if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
	{
		if(gps_tx.count==0)
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		else
		{
			USART_SendData(USART2,gps_tx_buff[gps_tx.rd_index++]);
			gps_tx.count--;
			if(gps_tx.rd_index == GPS_TX_BUFF_SIZE)
				gps_tx.rd_index = 0;
		}
	}
}

void gps_put_char(char data)			//queue data to be transmitted in TX buffer
{
	gps_tx_buff[gps_tx.wr_index++]=data;
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	gps_tx.count++;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	if(gps_tx.wr_index == GPS_TX_BUFF_SIZE)
		gps_tx.wr_index = 0;
}

void gps_puts(uint8_t *buff, uint8_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)
	{
		gps_put_char(*buff);
		buff++;
	}
}

char gps_get_char()
{
    char data;
    if(gps_rx.count==0)
    	return 0;
    data = gps_rx_buff[gps_rx.rd_index];
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    gps_rx.count--;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if(++gps_rx.rd_index==GPS_RX_BUFF_SIZE)
    	gps_rx.rd_index=0;
    return data;
}

//Check if GPS data is received, is so, start parsing messages
void parse_gps_data()
{
	if(gps_rx.count)
	{
		if(UBLOX_read(gps_rx.count)==TRUE)
			calculate_ublox();					//extract the GPS data to variables used in the rest of the program
	}
}

//Assign GPS data to the variables used in the rest of the program
void calculate_ublox()			//assign GPS data from hardware GPS
{
	ublox.fix  = GPS.fix;
	if(ublox.fix==1)
	{
		nav_dt = (GPS.time - old_tow)/1000.0f;
		old_tow = GPS.time;
		ublox.LAT = GPS.latitude;
		ublox.LONG = GPS.longitude;
		ublox.gps_alt = GPS.altitude/100.0f;
		ublox.gps_speed = GPS.ground_speed/100.0f;    // m/s
		ublox.gps_heading = GPS.ground_course/100.0f; // degrees
		ublox.mTOW = GPS.mTOW;
		ublox.hDOP = GPS.hDOP*0.01;
		ublox.vDOP = GPS.vDOP*0.01;
		ublox.pDOP = GPS.pDOP*0.01;

		if(ublox.gps_heading>180)
			ublox.gps_heading-=360;

		if((ublox.gps_speed > 1) && (nav_dt != 0))
		{
			ublox.gps_acc = ublox.gps_acc*0.3 + 0.7*(ublox.gps_speed - old_speed)/nav_dt;		//check math error
		}
		else
			ublox.gps_acc = 0;

		old_speed = ublox.gps_speed;
		X1 = (float)(LAT_home)/10000000;
		X2 = ToRad(X1);
		X3 = cos(X2);
		X3 = RE*X3;
		X = X3*ToRad((float)(ublox.LONG-LONG_home)/10000000);
		Y = RE*ToRad((float)(ublox.LAT-LAT_home)/10000000);
	}
	else			//eliminated as is not needed, gps speed will borrow air speed if fix not presen when necessary, however it will not replace the value of gps speed
	{
		ublox.gps_acc = 0;
		ublox.gps_heading = 0;
		ublox.gps_speed = 0;
	}
	ublox.satellites = GPS.num_sats;
	CurrentTime = ((GPS.year - 170)<<25) + (GPS.month<<21) + (GPS.day<<16) + (GPS.hour<<11) + (GPS.min<<5) + (GPS.sec/2);
	ublox.gps_fresh_data = 1;

	DL_GPS.latitude = ublox.LAT;
	DL_GPS.longitude = ublox.LONG;
	DL_GPS.ground_speed = ublox.gps_speed;
	DL_GPS.ground_course = ublox.gps_heading;
	DL_GPS.altitude = ublox.gps_alt;
	DL_GPS.HDOP = ublox.hDOP;
	DL_GPS.VDOP = ublox.vDOP;
	DL_GPS.PDOP = ublox.pDOP;
	DL_GPS.time = ublox.mTOW;
	DL_GPS.satellites = ublox.satellites;
	DL_GPS.fix = ublox.fix;
}
