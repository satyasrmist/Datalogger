#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4_discovery.h"
#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"

/***************************ASTERIA FILES*************************************************/

#include "PWM_output.h"
#include "ADC.h"
#include "RPM.h"
#include "modem.h"
#include "arduimu.h"
#include "DCM.h"
#include "common.h"
#include "gps.h"
#include "air_speed.h"
#include "packets.h"
#include "MS5611.h"
#include "power_propulsion.h"
#include "log.h"
#include "flash.h"
#include "loops.h"
#include"externalPackets.h"
/////////////////////////////////////
#include "debug.h"
/////////////////////////////////////

/****************************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"

/*********************************CONFIG*************************************************/

//#define USB_MS_ENABLED			//Uncomment line to enable MASS_STORAGE mode - Datalogger will operate as card reader as soon as USB is plugged in

/****************************************************************************************/

void NVIC_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART2_Configuration(void);

#include "ff.h"
#include "diskio.h"

#define STACK_SIZE_MIN	1024	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

void vmisc_tasks();
void vMSC_tasks();
void vIMU_tasks();

void IWDT_config();
uint32_t GetLSIFrequency(void);

uint8_t WDT_status = 0;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;

const uint8_t firmware_version[3] =
{
		0,			//hardware version
		1,			//code version
		1			//sub-code version
};

int main(void)
{
	 /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
	SystemInit();

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

#ifdef USB_MS_ENABLED
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	delay(1000);										//wait for pin initialisation to take effect ~125ms

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8))			//if USB is plugged in
	{
		uint32_t i = 0;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//Activate USB line-pull up

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB, GPIO_Pin_2);

	    USBD_Init(&USB_OTG_dev,
	  #ifdef USE_USB_OTG_HS
	              USB_OTG_HS_CORE_ID,
	  #else
	              USB_OTG_FS_CORE_ID,
	  #endif
	              &USR_desc,
	              &USBD_MSC_cb,
	              &USR_cb);

	    while(1)													//stay in infinite loop, datalogger is not operational in USB mode
	    {
	        if (i++ == 0x100000)
	        {
	        	GPIOF->ODR ^= GPIO_Pin_11;
	        	GPIOF->ODR ^= GPIO_Pin_12;
	        	GPIOF->ODR ^= GPIO_Pin_13;
	        	i = 0;
	        }
	    }
	}
#endif

//////////////////////////****************************//////////////////////////////

	IWDT_config();		//Comment the watchdog timer when debugging

//////////////////////////****************************//////////////////////////////

	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
	DBGMCU_Config(DBGMCU_I2C1_SMBUS_TIMEOUT, ENABLE);
	restore_offsets();
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        GPIO_WriteBit(GPIOF, GPIO_Pin_11, Bit_SET);
        RCC_ClearFlag();
    }

	xTaskCreate( vmisc_tasks, (const signed char*)"Manage misc tasks",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );

	xTaskCreate( vIMU_tasks, (const signed char*)"Manage IMU tasks",
		STACK_SIZE_MIN*5, NULL, 2, NULL );

	vTaskStartScheduler();

  while(1)
  {
	  GPIOD->ODR ^= GPIO_Pin_12;
  }
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void IWDT_config()
{
	RCC_LSICmd(ENABLE);
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value.
	     Counter Reload Value = time(ms)/IWDG counter clock period
	                          = time(ms) / (LSI/32)
	                          = time(ms) / (LsiFreq/32)
	                          = time(ms)		//since LsiFreq ~= 32KHz
   */
	  IWDG_SetReload(3000);		//about 3seconds based on LSI deviation

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

void vmisc_tasks()
{
	uint8_t time_count,time_200ms;
	portTickType ticks_now,ticks_old = 0;
	uint16_t time_elapsed = 0;

	modem_config();
	ublox_init();
//	debug_config();			//configure debug serial port
	/*
	 * data from external sensors initialized, version M21
	 * can disable it by commenting here and changing dataFromInternalSensors from 0 to 1
	 * check externalPackets.h for more information
	 */
		if (!(dataFromInternalSensors))
	/*	initialize external sensors data UART3, debug port and this can't be enabled together		*/
	externalDataPortInit();
	RPM_config();

	ADC_config();
	ADC_SoftwareStartConv(ADC1);
	ADC_3_config();
	ADC_SoftwareStartConv(ADC3);
	PWM_out_config();

	for(;;)
	{
		if(WDT_status == 1)
		{
			IWDG_ReloadCounter();
			WDT_status = 0;		//clear it so that it can be set again if all is OK
		}
		if(xbee_tx_count>0)								//enable TX interrupts if transmit data is present
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		if(gps_tx.count >0)								//enable TX interrupts if transmit data is present
			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		if(debug_tx_count >0)								//enable TX interrupts if transmit data is present
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

		parse_gps_data();
		if(xbee_rx_count)
			parse_modem_data();

		if(xbee_fresh_data)	//if received fresh data
		{
			calculate_xbee();		//call appropriate function based on received data
			xbee_fresh_data = 0;
		}

		if(log_init_flag)
		{
			if(write_buff_to_card == 1)		//if one buffer is full
			{
				res = f_open(&fil, file_name, FA_OPEN_ALWAYS | FA_WRITE |FA_READ);
				f_lseek(&fil, cursor_pos);
				if(log_buff_status == 1)
				{
					f_write(&fil, SD_buffer2.BUFFER, SD_buffer2.write_count, &bw);
					cursor_pos += SD_buffer2.write_count;
				}
				else if(log_buff_status == 2)
				{
					f_write(&fil, SD_buffer1.BUFFER, SD_buffer1.write_count, &bw);
					cursor_pos += SD_buffer1.write_count;
				}
				write_buff_to_card = 0;
				f_close(&fil);
				GPIOF->ODR ^= GPIO_Pin_12;
			}
		}

    	ticks_now = xTaskGetTickCount()*2;
    	if((ticks_now - ticks_old) >=100)		//do the following every 100ms
    	{
    		time_elapsed = ticks_now - ticks_old;
    	   	rpm = pulse_cnt*2*60/MOTOR_POLES*(1000/time_elapsed);

    	   	pulse_cnt = 0;

    		ticks_old = ticks_now;
    		air_speed_indicator();
    		send_status();
    		if(link_count >= 5 )		//no data from more than 500ms
    		{
    			modem_link_active = 0;	//indicate absence of GIB
    			link_count = 0;
    		}
    		else
    			link_count++;

    		if(time_200ms == 2)			//do the following every 200ms
    		{
    			send_gps_data();
    			time_200ms = 0;
    		}
    		else
    			time_200ms++;

		    if(time_count == 5)			//do the following every 500ms
		    {
		    	queue_data();			//append debug data for transmission
		    	update_battery();
		    	update_RPM();
		    	send_press_temp();
		    	send_defections();
		    	send_power_propulsion();
		    	time_count = 0;
		    }
		    else
		    	time_count++;
    	}
	}
}

void vIMU_tasks()
{
	uint8_t time_count;
	portTickType ticks_now,ticks_old = 0;
	uint8_t IMU_update_count = 0;

   	IMU_setup();
   	Baro_init();

    if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) == RESET)		//if it was a hardware reset
	{
		NVIC_SystemReset();								//force software reset so that MPU sensitivity returns to normal range
	}
    NVIC_Configuration();
	SD_Init();

//////////////////////////****************************//////////////////////////////
//	create_log_file();		//Uncomment to enable log on start, else log will start only when the button on GCS is pressed
//////////////////////////****************************//////////////////////////////

	for(int y=0; y<=5; y++)   // Read first initial ADC values for offset.
		AN_OFFSET[y] = 0;

    for(;;)
	{
    	WDT_status |= 1;		//update status for IMU tasks
    	if(log_init_flag)
		{
			log_data();			//append new data to buffer
		}
    	ticks_now = xTaskGetTickCount()*2;

    	if((ticks_now - ticks_old) >= 17)		//do the following every 17ms
    	{
    		G_Dt = (ticks_now - ticks_old)/1000.0;
    		ticks_old = ticks_now;

    		if((!calibrating_IMU) && (!calibrating_mag))	//update IMU only if not calibrating IMU or MAG
    		{
    			IMU_Update_ARDU();
    			HMC5883_calculate(roll,pitch);
    			Matrix_update();
    		    Normalize();
    		    Drift_correction();
    		    Euler_angles();
    		    update_variables();
    		}
    		else if(calibrating_mag)						//if calibrating MAG then do not process IMU, only get raw values and obtain MAG offsets
    		{
    		    if(IMU_update_count >= 5)
    			{
    		    	IMU_Update_ARDU();
    				IMU_update_count = 0;
    			}
    			else
    				IMU_update_count++;
    		}
			altitude = ((Baro_update(G_Dt*1000)) - ground_alt_offset);		//time in ms

			innerloop();									//set servo output
		    if(time_count == 5)								//do this every 5*17ms ~= 100ms
		    {
		    	send_attitude();
		    	send_attitude_RAW();
		    	time_count = 0;
		    }
		    else
		    	time_count++;
    	}
		vTaskDelay( 17/ portTICK_RATE_MS );					//3ms less to compensate for IMU update time
	}
}
