#include "log.h"
#include"externalPackets.h"

volatile uint8_t log_init_flag = 0;
char file_name[] = "0:/LOG000.txt";
volatile DWORD cursor_pos =0;

char *LOG_BUFF;
volatile uint8_t log_buff_status = 1;		//keep track of which buffer you are writing too, 1 = buffer 1, 2 = buffer 2
log_buffer SD_buffer1,SD_buffer2;
volatile uint8_t write_buff_to_card = 0;

uint32_t log_loop_count = 0;

void create_log_file()
{
	uint8_t byte_append_count;
	char str[256];			//single write is 312bytes

	memset(&fs32, 0, sizeof(FATFS));
	res = f_mount(0, &fs32);
	memset(&fil, 0, sizeof(FIL));

	//******************clear buffers to get rid of previous data******************//
	memset(SD_buffer1.BUFFER, 0, sizeof(SD_buffer1.BUFFER));
	memset(SD_buffer2.BUFFER, 0, sizeof(SD_buffer2.BUFFER));
	write_buff_to_card = 0;					//reset variable since buffers are empty
	log_buff_status = 1;					//start appending data to buffer 1
	//****************************************************************************//

	while(log_init_flag==0)
	{
		res = f_open(&fil, file_name, FA_OPEN_EXISTING);

		if(res==FR_NO_FILE)
		{	// success in creating file
			res = f_open(&fil, file_name, FA_OPEN_ALWAYS | FA_WRITE |FA_READ);
			if(res==FR_OK)
			{
				cursor_pos = 0;
				byte_append_count = sprintf(str,"Data Logger on board LOG at 50 Hz \r\n");
				append_byte_to_array(str, byte_append_count);

				byte_append_count = sprintf(str,"UTC Start: %02d-%02d-%d %02d:%02d:%02d \r\n",(int)GPS.day, (int)GPS.month, (int)GPS.year, (int)GPS.hour, (int)GPS.min, (int)GPS.sec);
				append_byte_to_array(str, byte_append_count);

				byte_append_count = sprintf(str,"Time,Roll,Pitch,Yaw,Roll Rate,Pitch Rate,Yaw Rate,ACC_X,ACC_Y,ACC_Z,GYR_X,GYR_Y,GYR_Z,MAG_X,MAG_Y,MAG_Z,");
				append_byte_to_array(str, byte_append_count);

				byte_append_count = sprintf(str,"Elevator,Aileron_1,Aileron_2,Rudder,Latitude,Longitude,Ground Speed,Ground Course,Altitude,HDOP,VDOP,PDOP,ToW,Satellites,Fix,");
				append_byte_to_array(str, byte_append_count);

				byte_append_count = sprintf(str,"Pressure,Altitude,Temperature,Air Speed,RC1,RC2,RC3,RC4,RC5,RC6,RC7,RC8,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,RPM,Battery_V,Motor_I, \r\n");
				append_byte_to_array(str, byte_append_count);
				log_init_flag=1;
				send_message(LOG_INIT_MSG_ID);
				GPIOF->ODR ^= GPIO_Pin_12;
				break;
			}
		}
		else
		{
			file_name[8] += 1;
			if((file_name[6] == '5') && (file_name[7] == '1') && (file_name[8] == '2'))		//break if limit of 512 files has been reached
			{
				break;   // put a flag for log filename overflow
			}
			else
			{
				if(file_name[8]>'9')
				{
					file_name[8] = '0';
					file_name[7] += 1;
					if(file_name[7]>'9')
					{
						file_name[7] = '0';
						file_name[6] += 1;
					}
				}
			}
		}
	}
	f_close(&fil);
}

//Check which data is to be written and call the function to append it to the active buffer
void log_data()
{
	uint8_t byte_append_count;
	uint32_t ticks_now;
	char str[256];			//single write is 312bytes


	ticks_now = xTaskGetTickCount()*2;

	if (dataFromInternalSensors)
	{
		byte_append_count = sprintf(str,"%08.03f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,",(double)ticks_now/1000.0,DL_attitude.roll,DL_attitude.pitch,DL_attitude.yaw,DL_attitude.roll_rate,DL_attitude.pitch_rate,DL_attitude.yaw_rate,DL_attitude_raw.acc_X,DL_attitude_raw.acc_Y,DL_attitude_raw.acc_Z,DL_attitude_raw.gyr_X,DL_attitude_raw.gyr_Y,DL_attitude_raw.gyr_Z,DL_attitude_raw.mag_X,DL_attitude_raw.mag_Y,DL_attitude_raw.mag_Z,DL_surface_deflection.elevator,DL_surface_deflection.aileron_1,DL_surface_deflection.aileron_2,DL_surface_deflection.rudder);
		append_byte_to_array(str, byte_append_count);

		//GPS Data
		if(((log_loop_count+1)%GPS_LOG_RATE) == 0)		//if multiple of log rate
		{
			byte_append_count = sprintf(str,"%08ld,%08ld,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08ld,%02d,%01d,",DL_GPS.latitude,DL_GPS.longitude,DL_GPS.ground_speed,DL_GPS.ground_course,DL_GPS.altitude,DL_GPS.HDOP,DL_GPS.VDOP,DL_GPS.PDOP,DL_GPS.time,DL_GPS.satellites,DL_GPS.fix);
			append_byte_to_array(str, byte_append_count);
		}
		else
		{
			byte_append_count = sprintf(str,",,,,,,,,,,,");
			append_byte_to_array(str, byte_append_count);
		}
	}
	else
	{
		/*	to parse external sensor data come in				*/
		ParseExtDataByte();
		/*	external IMU Data				*/
		byte_append_count = sprintf(str,"%08.03f,%08.03f,%08.03f,%08.03f,%08.04f,%08.04f,%08.04f,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,%06d,",(double)ticks_now/1000.0,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate,DL_attitude_raw.acc_X,DL_attitude_raw.acc_Y,DL_attitude_raw.acc_Z,DL_attitude_raw.gyr_X,DL_attitude_raw.gyr_Y,DL_attitude_raw.gyr_Z,DL_attitude_raw.mag_X,DL_attitude_raw.mag_Y,DL_attitude_raw.mag_Z,DL_surface_deflection.elevator,DL_surface_deflection.aileron_1,DL_surface_deflection.aileron_2,DL_surface_deflection.rudder);
		append_byte_to_array(str, byte_append_count);

		if (extBuffer[1]==2)
		{
			byte_append_count = sprintf(str,"%08.05ld,%08.05ld,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08ld,%02d,%01d,",GPSLAT,GPSLONG,Speed,DL_GPS.ground_course,GPSAlt,DL_GPS.HDOP,DL_GPS.VDOP,DL_GPS.PDOP,DL_GPS.time,GPSSAT,GPSFix);
			append_byte_to_array(str, byte_append_count);
		}
		else

		{
			byte_append_count = sprintf(str,",,,,,,,,,,,");
			append_byte_to_array(str, byte_append_count);
		}

	}


	//Press Data
	if(((log_loop_count+1)%PRESS_TEMP_RATE) == 0)		//if multiple of log rate
	{
		byte_append_count = sprintf(str,"%08.2f,%08.2f,%08.2f,%08.2f,",DL_pressure_temperature.pressure ,DL_pressure_temperature.altitude,DL_pressure_temperature.temperature ,DL_pressure_temperature.air_speed);
		append_byte_to_array(str, byte_append_count);
	}
	else
	{
		byte_append_count = sprintf(str,",,,,");
		append_byte_to_array(str, byte_append_count);
	}

	byte_append_count = sprintf(str,"%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%06ld,%08.2f,%08.2f,\r\n",RC_in_sticks[0],RC_in_sticks[1],RC_in_sticks[2],RC_in_sticks[3],RC_in_sticks[4],RC_in_sticks[5],RC_in_sticks[6],RC_in_sticks[7],PWM_out_channel[0],PWM_out_channel[1],PWM_out_channel[2],PWM_out_channel[3],PWM_out_channel[4],PWM_out_channel[5],PWM_out_channel[6],PWM_out_channel[7],DL_power_propulsion.motor_RPM ,DL_power_propulsion.battery_V ,DL_power_propulsion.motor_I);
	append_byte_to_array(str, byte_append_count);
	log_loop_count++;	//log has been written either to master buffer or secondary
	GPIOF->ODR ^= GPIO_Pin_13;

	//	else
	//	{
	//		float simAppendTime = 0000.201;
	//		float simPitch = -268.12;
	//		float simRoll = 1.26;
	//		float simYaw = 84.84;
	//		float simPitchRate = 00007.21;
	//		float simRollRate = -0106.21;
	//		float simYawRate = 00000.21;
	//		byte_append_count = sprintf(str,"%08.03f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f,%08.2f\r\n",simAppendTime,simPitch, simRoll,
	//				simYaw, simPitchRate, simRollRate, simYawRate);
	//		append_byte_to_array(str, byte_append_count);
	//
	//	}
}

//Append incoming data to the active buffer, check buffer boundary and switch accordingly
void append_byte_to_array(char array[],uint8_t bytes_to_append)
{
	if(log_buff_status == 1)		//if appending to first buffer
	{
		if((SD_buffer1.write_count + bytes_to_append) > LOG_BUFF_SIZE)		//overflow detected
		{
			write_buff_to_card = 1;
			log_buff_status = 2;		//switch to second buffer
			memset(SD_buffer2.BUFFER, 0, sizeof(SD_buffer2.BUFFER));
			strcat(SD_buffer2.BUFFER,array);
			SD_buffer2.write_count = bytes_to_append;
		}
		else
		{
			strcat(SD_buffer1.BUFFER,array);
			SD_buffer1.write_count += bytes_to_append;
		}
	}
	else if(log_buff_status == 2)
	{
		if((SD_buffer2.write_count + bytes_to_append) > LOG_BUFF_SIZE)		//overflow detected
		{
			write_buff_to_card = 1;
			log_buff_status = 1;		//switch to second buffer
			memset(SD_buffer1.BUFFER, 0, sizeof(SD_buffer1.BUFFER));
			strcat(SD_buffer1.BUFFER,array);
			SD_buffer1.write_count = bytes_to_append;
		}
		else
		{
			strcat(SD_buffer2.BUFFER,array);
			SD_buffer2.write_count += bytes_to_append;
		}
	}
}
