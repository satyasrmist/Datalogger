#include "packets.h"

attitude_processed_t DL_attitude;
attitude_raw_t DL_attitude_raw;
surface_deflections_t DL_surface_deflection;
gps_t DL_GPS;
pressure_temperature_t DL_pressure_temperature;
power_propulsion_t DL_power_propulsion;
status_t DL_status;

void update_variables()
{
	DL_attitude.roll = ToDeg(roll);
	DL_attitude.pitch = ToDeg(pitch);
	DL_attitude.yaw = ToDeg(yaw);
	DL_attitude.roll_rate = AN[0]/GYRO_SENSITIVITY;
	DL_attitude.pitch_rate = AN[1]/GYRO_SENSITIVITY;
	DL_attitude.yaw_rate = AN[2]/GYRO_SENSITIVITY;

	DL_attitude_raw.acc_X = AN[3];
	DL_attitude_raw.acc_Y = AN[4];
	DL_attitude_raw.acc_Z = AN[5];
	DL_attitude_raw.gyr_X = AN[0];
	DL_attitude_raw.gyr_Y = AN[1];
	DL_attitude_raw.gyr_Z = AN[2];
	DL_attitude_raw.mag_X = mag_imu[1];
	DL_attitude_raw.mag_Y = mag_imu[0];
	DL_attitude_raw.mag_Z = mag_imu[2];

	DL_pressure_temperature.pressure = pressure*0.01;
	DL_pressure_temperature.altitude = altitude;
	DL_pressure_temperature.temperature = 0.5*(temperature + MPU_TEMP);
	DL_pressure_temperature.air_speed = air_speed;

	DL_power_propulsion.motor_RPM = motor_RPM;
	DL_power_propulsion.battery_V = battery_V;
	DL_power_propulsion.motor_I = motor_current;
}

void send_attitude()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 150, (const char *)&DL_attitude,24,150);
}

void send_attitude_RAW()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 151, (const char *)&DL_attitude_raw,18,151);
}

void send_press_temp()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 152, (const char *)&DL_pressure_temperature,16,152);
}

void send_defections()
{

}

void send_gps_data()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 154, (const char *)&DL_GPS,38,154);
}

void send_power_propulsion()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 155, (const char *)&DL_power_propulsion,12,155);
}

void send_status()
{
	uint8_t i;

	DL_status.flight_mode = flight_mode;
	DL_status.log_status = log_init_flag;
	DL_status.target_heading = heading_target;
	for(i=0;i<8;i++)
		DL_status.RC_channels[i] = PWM_out_channel[i];

	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 156, (const char *)&DL_status,22,156);
}

void send_firmware_version()
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 157, (const char *)&firmware_version,3,157);
}

void send_message(uint8_t ID)
{
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, 200, (const char *)&ID,1,200);
}
