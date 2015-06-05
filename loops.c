#include "loops.h"

float roll_target = 0, heading_target = 0;

uint8_t flight_mode = 0;		//0 - Manual, 1 - Stabilize, 2 - Auto Mode (Heading Hold)

int16_t excitation_offset = 0;
uint16_t excitation_time = 0;	//ms
uint8_t excitation_state = 0;	//0-inactive, 1 - state1, 2 - state2
uint16_t excitation_time_elapsed = 0;
uint32_t excitation_time_old = 0;
uint8_t excitation_channel = 1;	//valid range 1-8 corresponding to PWM channels
uint8_t excitation_type = 0;	//0-undefined,1- undefined, 2- doublet

//Computes and sets the PWM servo outputs
void innerloop()
{
	switch(flight_mode)
	{
	case 0:			//Manual
		PWM_Set_Duty_Cycle(AILERON,RC_in_sticks[0]);		//internally limited between MAX/MIN
		PWM_Set_Duty_Cycle(ELEVATOR,RC_in_sticks[1]);
		PWM_Set_Duty_Cycle(THROTTLE,RC_in_sticks[2]);
		PWM_Set_Duty_Cycle(RUDDER,RC_in_sticks[3]);
		break;

	case 1:			//Stabilize
		roll_hold(0);										//sets AILERON outputs to hold zero roll angle
		check_servo_limits();								//check limits of all channels
		PWM_Set_Duty_Cycle(ELEVATOR,RC_in_sticks[1]);		//internally limited between MAX/MIN
		PWM_Set_Duty_Cycle(THROTTLE,RC_in_sticks[2]);
		PWM_Set_Duty_Cycle(RUDDER,RC_in_sticks[3]);
		if(excitation_state > 0)
		{
			carry_out_excitation();
		}
		break;

	case 2:  //Auto mode (Heading Hold)
		heading_hold(heading_target); 						//sets roll target to follow the heading target
		roll_hold(roll_target);								//sets AILERON outputs to follow roll target
		PWM_Set_Duty_Cycle(ELEVATOR,RC_in_sticks[1]);		//internally limited between MAX/MIN
		PWM_Set_Duty_Cycle(THROTTLE,RC_in_sticks[2]);
		PWM_Set_Duty_Cycle(RUDDER,RC_in_sticks[3]);
		break;
	}
}

void set_target_heading()
{
	uint8_t count;
	union
	{
		float val;
		struct
		{
			uint8_t bytes[4];
		};
	}extract_float;

	for(count = 0; count <4; count++)
	{
		extract_float.bytes[count] = xbee_buff[count + 5];
	}
	heading_target = extract_float.val;
}

//Estimates the roll correction/target to hold the set heading target
void heading_hold(int val)
{
	float heading_error,heading_output;

	heading_error = (heading_target - ToDeg(yaw));

	heading_output = heading_error - (roll*K_ROLL);
	if((heading_output>180))
		heading_output -= 360;
	else if(heading_output<-180)
		heading_output += 360;
	roll_target = Kp_HEAD*heading_output; //compute roll target from heading error
}

//Computes the Aileron output based on current roll and send corrective output to the Aileron
void roll_hold(float roll_target)
{
	float aileron_output;

	if(roll_target > ROLL_LIMIT)
		roll_target = ROLL_LIMIT;
	else if(roll_target < -ROLL_LIMIT)
		roll_target = -ROLL_LIMIT;
	aileron_output = ((Kp_ROLL*(ToDeg(roll) - roll_target)) + Kd_ROLL*(AN[0]))*Kscale_ROLL; //PID which computes aileron deflection required to follow roll target
	PWM_Set_Duty_Cycle(AILERON,RC_in_sticks[AILERON-1] + aileron_output);  //add trim value to aileron deflection to compute final aileron output
}

void set_excitation()
{
	uint8_t count;
	union
	{
		int16_t val;
		struct
		{
			uint8_t bytes[2];
		};
	}extract_int16_t;
	excitation_type = xbee_buff[5];

	for(count = 0; count <2; count++)
	{
		extract_int16_t.bytes[count] = xbee_buff[count + 6];
	}
	excitation_offset = extract_int16_t.val;
	for(count = 0; count <2; count++)
	{
		extract_int16_t.bytes[count] = xbee_buff[count + 8];
	}
	excitation_time = (uint16_t) extract_int16_t.val;
	excitation_channel = xbee_buff[10];
	send_message(EXCITATION_SET_MSG_ID);
}

//Initiate excitation
void trigger_excitation()
{
	excitation_state = 1;
	excitation_time_elapsed = 0;
	excitation_time_old = xTaskGetTickCount()*2;	//	store current time
}

void carry_out_excitation()
{
	uint16_t temp_time;
	if(excitation_type == 2)		//if it is a doublet
	{
		temp_time = (xTaskGetTickCount()*2);
		excitation_time_elapsed += temp_time - excitation_time_old;
		excitation_time_old = temp_time;
		if(excitation_time_elapsed >= excitation_time)
		{
			if(excitation_state == 1)
			{
				excitation_state = 2;
				excitation_time_elapsed = 0;
			}
			else if(excitation_state == 2)
			{
				excitation_state = 0;
				excitation_time_elapsed = 0;
			}
		}
		if((excitation_channel >0) && (excitation_channel < 9))		//if valid channel is selected
		{
			if(excitation_state == 1)
				PWM_Set_Duty_Cycle(excitation_channel,RC_trim[excitation_channel - 1] + excitation_offset);	//internally limited between MAX/MIN
			else if(excitation_state == 2)
				PWM_Set_Duty_Cycle(excitation_channel,RC_trim[excitation_channel - 1] - excitation_offset);	//internally limited between MAX/MIN
		}
	}
}
