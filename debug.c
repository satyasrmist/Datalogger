#include "debug.h"

uint8_t debug_tx_buff[DEBUG_TX_BUFF_SIZE];
volatile uint16_t debug_tx_index_rd = 0,debug_tx_index_wr = 0,debug_tx_count = 0;

//Configures the serial port hardware to function with interrupts, just like other serial ports
//Baud 115200, 8-N-1
//Only Tx, no RX active
void debug_config()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

//This function appends data to the global buffer which transmits data.
//Global buffer size is kept at 500bytes.
//This function allows appending a maximum of 100 bytes of data to global buffer at a time.
//'sprintf' has a limit a print limit of 255bytes at one time, it truncates the remaining bytes, so do not use sprintf to transfer more than 255 bytes to a buffer.
// It is recommended to follow the formatting as used in the 'log.c' file for various variables.
void queue_data()
{
	char temp_data[100];
	uint8_t byte_count,i=0;

	byte_count = sprintf(temp_data,"DBG - Roll: %08.2f, ACC_X: %06d, RC_1: %04d , Batt.:%08.2f \r\n",DL_attitude.roll, DL_attitude_raw.acc_X,RC_in_sticks[0],DL_power_propulsion.battery_V);

	for(i=0;i<byte_count;i++)
	{
		debug_put_char(temp_data[i]);
	}
}

//This function directly appends data to the global buffer.
void debug_put_char(char data)
{
	debug_tx_buff[debug_tx_index_wr++]=data;
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	debug_tx_count++;
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	if(debug_tx_index_wr == DEBUG_TX_BUFF_SIZE)
		debug_tx_index_wr = 0;
}

//ISR for debug serial port
//void USART3_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART3, USART_IT_TXE) == SET)		//if Tx interrupt is active
//	{
//		if(debug_tx_count==0)								//turn off Tx interrupt if no more data is to be sent, else the MCU will repeatedly call the Tx interrupt and thus hang.
//			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
//		else
//		{
//			USART_SendData(USART3, (uint16_t) debug_tx_buff[debug_tx_index_rd++]);	//send the data from the start of buffer read index.
//			debug_tx_count--;														//decrement the tx buffer count
//			if(debug_tx_index_rd == DEBUG_TX_BUFF_SIZE)								//wrap around if buffer limit has reached
//				debug_tx_index_rd = 0;
//		}
//	}
//}
