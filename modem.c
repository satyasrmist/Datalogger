#include "modem.h"

char xbee_rx_buff[XBEE_RX_BUFF_SIZE];
uint8_t xbee_tx_buff[XBEE_TX_BUFF_SIZE];
volatile uint16_t xbee_rx_index_wr=0, xbee_rx_index_rd=0;
uint16_t xbee_rx_count=0;
volatile uint16_t xbee_tx_index_rd=0,xbee_tx_index_wr=0,xbee_tx_count=0;

char xbee_packet_set = 0;
char xbee_packet_size = 0;
char xbee_fresh_data = 0;
uint8_t xbee_buff_index = 0;
volatile char xbee_buff[XBEE_BUFF_SIZE];

uint16_t CRC_Accum_Rx;
uint8_t modem_link_active = 0,link_count = 0;

void modem_config()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void xbee_put_char(char data)
{
	if(modem_link_active)		//start sending only if you receive stick packets from GIB, hence you will prevent bricking the XBees
	{
		xbee_tx_buff[xbee_tx_index_wr++]=data;
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		xbee_tx_count++;
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		if(xbee_tx_index_wr == XBEE_TX_BUFF_SIZE)
			xbee_tx_index_wr = 0;
	}
	else
	{
		xbee_tx_count = 0;
	}
}

char xbee_get_char()
{
    char data;
    if(xbee_rx_count==0)
    	return 0;
    data = xbee_rx_buff[xbee_rx_index_rd++];
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    xbee_rx_count--;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    if(xbee_rx_index_rd==XBEE_RX_BUFF_SIZE)
    	xbee_rx_index_rd=0;
    return data;
}

void USART1_IRQHandler(void)
{
	if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
		xbee_rx_buff[xbee_rx_index_wr++] = (u8) USART_ReceiveData(USART1);
		xbee_rx_count++;
		if(xbee_rx_index_wr==XBEE_RX_BUFF_SIZE)
		{
			xbee_rx_index_wr = 0;
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
	{
		if(xbee_tx_count==0)
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

		else
		{
			USART_SendData(USART1, (uint16_t) xbee_tx_buff[xbee_tx_index_rd++]);
			xbee_tx_count--;
			if(xbee_tx_index_rd == XBEE_TX_BUFF_SIZE)
				xbee_tx_index_rd = 0;
		}
	}
}

void parse_modem_data()
{
    char data;
    uint8_t ck[2];

    while(xbee_rx_count)
    {
        data = xbee_get_char();
        if(data == 255  && xbee_packet_set==0)
        {
            xbee_packet_set = 1;
            xbee_buff_index = 0;
            CRC_Accum_Rx = 0xFFFF;
        }
        else
        {
            if(xbee_packet_set==1)
            {
                if(xbee_buff_index==0)
                {
                    xbee_packet_size = data + 7;		//for header data
                }
                xbee_buff[xbee_buff_index++] = data;
                if(xbee_buff_index <= (xbee_packet_size - 2))	//do not accumulate checksums
                {
                	CRC_accumulate_rx(data,&CRC_Accum_Rx);
                }

                if(xbee_buff_index == xbee_packet_size)
                {
                  	xbee_packet_set=0;
                    xbee_buff_index=0;
                    crc_accumulate(xbee_buff[4], &CRC_Accum_Rx);
                	ck[0] = (uint8_t)(CRC_Accum_Rx & 0xFF);
                	ck[1] = (uint8_t)(CRC_Accum_Rx >> 8);
                    if(xbee_buff[xbee_packet_size-1]==ck[1] && xbee_buff[xbee_packet_size-2] == ck[0])
                    {
                        xbee_fresh_data=1;
                        return;
                    }
                }
            }
        }
    }
}

void CRC_accumulate_rx(uint8_t data, uint16_t *crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    uint8_t tmp;

    tmp = data ^ (uint8_t)(*crcAccum &0xff);
    tmp ^= (tmp<<4);
    *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

void calculate_xbee()
{
	switch(xbee_buff[4])
	{
	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
		update_sticks();
		break;
	case CALIBRATE_AIR_SPEED_ID:
		zero_air_speed_calculate();
		break;
	case CALIBRATE_IMU_ID:
		startup_ground();
		break;
	case CALIBRATE_MAG_ID:
		mag_calib();
		break;
	case CALIBRATE_ALTITUDE_ID:
		zero_altitude();
		break;
	case LOG_INIT_ID:
		if(!log_init_flag)
			create_log_file();
		else
			log_init_flag = 0;
		break;
	case SET_TRIMS_ID:
		set_trims();
		break;
	case TRIGGER_EXCITATION_ID:
		trigger_excitation();
		break;
	case SET_HEADING_ID:
		set_target_heading();
		break;
	case SET_EXCITATION_ID:
		set_excitation();
		break;
	case REBOOT_SYSTEM_ID:
		NVIC_SystemReset();
		break;
	case FIRMWARE_REQUEST_ID:
		send_firmware_version();
		break;
	}
}
