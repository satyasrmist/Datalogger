#include"externalPackets.h"

volatile uint8_t UART4RxBuffer[RXBUFFERSIZE], UART4TxBuffer[TXBUFFERSIZE];
volatile uint8_t UART4RxRdIndex =0, UART4TxRdIndex =0, UART4RxCount =0, UART4TxCount =0;
volatile uint8_t UART4RxWrIndex =0, UART4TxWrIndex = 0, receivedBufferIndex, packetReadStart= 0;
uint8_t maxBufferSize = 0;
uint8_t extBuffer[BUFFERSIZE];

	extern float Roll =0;
	extern float Pitch =0;
	extern float Yaw = 0;
	extern float RollRate =0;
	extern float PitchRate =0;
	extern float YawRate =0;
	extern float GPSLAT =0;
	extern float GPSLONG =0;
	extern float GPSAlt =0;
	extern float Speed =0;
	extern float Heading =0;
	extern uint8_t GPSFix =0;
	extern uint8_t GPSSAT =0;
	extern float GPSAirSpeed =0;
	extern float BaroALT =0;

/*
 * initialize USART3 before use
 * initialize GPIO on external IMU port
 * Initialize NVIC interrupt
 * initialize USART on PD 8 , 9 with 15200 8N1
 */
void externalDataPortInit()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx| USART_Mode_Rx;;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/*
 * initialize UART4 before use
 * not used here
 * Initialize NVIC interrupt
 * initialize USART on PA 0 , 1 with 15200 8N1
 */

void UART4Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef UART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	UART_InitStructure.USART_BaudRate = 115200;
	UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	UART_InitStructure.USART_StopBits = USART_StopBits_1;
	UART_InitStructure.USART_Parity = USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &UART_InitStructure);
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}

/*
 * interrupt handler
 * puts incoming data into buffer
 * sends outgoing data from buffer
 */

void USART3_IRQHandler(void)
{
	if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
		UART4RxBuffer[UART4RxRdIndex++] = (u8) USART_ReceiveData(USART3);
		UART4RxCount++;
		if(UART4RxRdIndex== RXBUFFERSIZE)
		{
			UART4RxRdIndex = 0;
		}
	}
	if(USART_GetITStatus(USART3, USART_IT_TXE) == SET)
	{
		if(UART4TxCount==0)
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

		else
		{
			USART_SendData(USART3, (uint16_t) UART4TxBuffer[UART4TxWrIndex]);
			UART4TxCount--;
			if(UART4TxWrIndex == TXBUFFERSIZE)
				UART4TxWrIndex= 0;
		}
	}
}

/*
 * parse byte array into a different data buffer
 * parse the incoming data from interrupt buffer
 * store data inside extBuffer for further use
 */
void ParseExtDataByte()
{

	char data;
	while (UART4RxCount)
	{
		data = ExtGetChar();
		if((data == 254)&& (packetReadStart == 0))
		{
			receivedBufferIndex = 0;
			packetReadStart = 1;
		}
		else
		{
			if(packetReadStart)
			{
				if (receivedBufferIndex == 0)
					maxBufferSize = data;
				extBuffer[receivedBufferIndex] = data;

				if(++receivedBufferIndex == maxBufferSize)
				{
					receivedBufferIndex = 0;
					packetReadStart = 0;
				}
			}
		}
	}
	CalculateExtIMU();
}

/*
 * returns a byte from a incoming data array
 * USART enable and disable is to make sure it doesn't override
 * while interrupr is in use
 */
char ExtGetChar()
{
	char data =0;
	if(UART4RxCount==0)
		return 0;
	data = UART4RxBuffer[UART4RxWrIndex++];
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	UART4RxCount--;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	if(UART4RxWrIndex == RXBUFFERSIZE)
		UART4RxWrIndex=0;
	return data;
}

/*
 *calculates the IMU data from extDataBuffer
 *differentiate IMU packet from GPS packet
 *Converts in to meaningful attitude information
 */
void CalculateExtIMU()			//sort out data from X-PLANE
{
	int i;
	static int extDataBuffer[6];
	if(extBuffer[1]==1)			//roll pitch yaw packet
	{
		for(i=1; i<(maxBufferSize)/2; i++)
			extDataBuffer[i-1] = 256*extBuffer[2*i+1] + extBuffer[2*i];

		Roll = (float)((signed int)extDataBuffer[0] - 30000)/100;
		Pitch = (float)((signed int)extDataBuffer[1] - 30000)/100;
		Yaw = (float)((signed int)extDataBuffer[2] - 30000)/100;
		//		yaw += (*MAG_DECLINATION);

		RollRate = (float)((signed int)extDataBuffer[3] - 30000)/10;
		PitchRate = (float)((signed int)extDataBuffer[4] - 30000)/10;
		YawRate = (float)((signed int)extDataBuffer[5] - 30000)/10;

		//		hi_pass(yaw_rate, 0.1);
	}
	else if(extBuffer[1]==2)	//gps packet
		CalculateExtGPS();
}
/*
 * parse GPS packets and stored into
 * external sensor data GPS packet strutcher
 */
void CalculateExtGPS(void)
{
	uint8_t i;

	GPSFix  = extBuffer[16];
	if(GPSFix==1)
	{
		GPSLAT = 0;
		GPSLONG = 0;
		for(i=5; i>1; i--)
			GPSLONG = (GPSLONG*256 + extBuffer[i]);
		for(i=9; i>5; i--)
			GPSLAT = (GPSLAT*256 + extBuffer[i]);

				GPSAlt = ((int)extBuffer[11]*256 + extBuffer[10])/10.0;	//GPS altitude
				GPSAirSpeed = ((int)extBuffer[13]*256 + extBuffer[12])*0.01;    // m/s
				Heading = ((int)extBuffer[15]*256 + extBuffer[14])/100.0; // degrees

		if(Heading>180)
			Heading-=360;

		GPSAirSpeed = ((int)extBuffer[19]*256 + extBuffer[18])*0.01;    // m/s
		BaroALT = ((int)extBuffer[21]*256 + extBuffer[20])/10.0;    // m/s	Barometric altitude

		GPSSAT = extBuffer[17];
	}
}
