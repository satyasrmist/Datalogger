#ifndef PACKETS_H
#define PACKETS_H

#include "stm32f4xx.h"
#include "ADC.h"
#include "mavlink_types.h"
#include "arduimu.h"
#include "ublox.h"
#include "air_speed.h"
#include "MS5611.h"
#include "power_propulsion.h"
#include "log.h"
#include "rpm.h"

extern const uint8_t firmware_version[3];

typedef struct __attitude_processed_t	//24bytes
{
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
} attitude_processed_t;

typedef struct __attitude_raw_t			//18bytes
{
	int16_t acc_X;
	int16_t acc_Y;
	int16_t acc_Z;
	int16_t gyr_X;
	int16_t gyr_Y;
	int16_t gyr_Z;
	int16_t mag_X;
	int16_t mag_Y;
	int16_t mag_Z;
} attitude_raw_t;

typedef struct __surface_deflections_t	//8bytes
{
	uint16_t elevator;
	uint16_t aileron_1;
	uint16_t aileron_2;
	uint16_t rudder;
} surface_deflections_t;

typedef struct __gps_t					//38bytes
{
	int32_t latitude;
	int32_t longitude;
	float ground_speed;
	float ground_course;
	float altitude;
	float HDOP;
	float VDOP;
	float PDOP;
	uint32_t time;
	uint8_t satellites;
	uint8_t fix;
} gps_t;

typedef struct __pressure_temperature_t	//16bytes
{
	float pressure;
	float altitude;
	float temperature;
	float air_speed;
} pressure_temperature_t;

struct __power_propulsion_t				//12bytes
{
	uint32_t motor_RPM;
	float battery_V;
	float motor_I;
} __attribute__((packed));
typedef struct __power_propulsion_t power_propulsion_t;

struct __status_t 						//22bytes
{
	uint8_t flight_mode;
	uint8_t log_status;
	float target_heading;
	uint16_t RC_channels[8];
} __attribute__((packed));
typedef struct __status_t status_t;

extern attitude_processed_t DL_attitude;
extern attitude_raw_t DL_attitude_raw;
extern surface_deflections_t DL_surface_deflection;
extern gps_t DL_GPS;
extern pressure_temperature_t DL_pressure_temperature;
extern power_propulsion_t DL_power_propulsion;
extern status_t DL_status;

void update_variables();
void send_attitude();
void send_attitude_RAW();
void send_gps_data();
void send_press_temp();
void send_defections();
void send_power_propulsion();
void send_status();
void send_message(uint8_t ID);
void send_firmware_version();
#endif

