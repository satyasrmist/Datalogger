#include "arduimu.h"

float G_Dt=0.02;    // Integration time (DCM algorithm)

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=180;

int mag_x;
int mag_y;
int mag_z;
float Heading;
float Heading_X;
float Heading_Y;

uint16_t ground_alt_offset = 0;
int16_t MAG_OFFSET[3];		//store offset values for X,Y,Z, divide the raw value by each to obtain the scaled value
float MAG_SCALE[3];
float MPU_TEMP = 0;

volatile uint8_t calibrating_IMU=0,calibrating_mag =0;
int16_t MAG_X[MAG_CALIB_SIZE],MAG_Y[MAG_CALIB_SIZE],MAG_Z[MAG_CALIB_SIZE];
int16_t MAG_MAX[3],MAG_MIN[3];
uint8_t MAG_sample_count = 0;

int16_t mag_imu[3];

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

const int counts_per_milligauss[8]={
  1370,
  1090,
  820,
  660,
  440,
  390,
  330,
  230
};

int SENSOR_SIGN[] = {1,-1,-1,-1,1,1,-1,1,-1};

float read_adc(int select)
{
  if (SENSOR_SIGN[select]<0) {
    return (AN_OFFSET[select]-AN[select]);
  }
  else {
    return (AN[select]-AN_OFFSET[select]);
  }
}

void IMU_Update_ARDU()
{
	uint8_t success; //,i;
	int16_t accel_imu[3];
	int16_t gyro_imu[3];

    success = MPU6050_GetRawAccelGyroMag(accel_imu,gyro_imu,mag_imu);
    if(success==0)
	{
		MPU6050_I2C_Reboot();
		return;
	}
    GPIOF->ODR ^= GPIO_Pin_11;
	AN[0] = gyro_imu[1];		//X
	AN[1] = -gyro_imu[0];
	AN[2] = gyro_imu[2];
	AN[3] = accel_imu[1];		//X
	AN[4] = -accel_imu[0];
	AN[5] = accel_imu[2];

	if(!calibrating_mag)
	{
//		for(i=0;i<3;i++)
//			mag_imu[i] = (mag_imu[i] - MAG_OFFSET[i]) * MAG_SCALE[i];
		mag_x = (mag_imu[1] - MAG_OFFSET[1]) * MAG_SCALE[1];
		mag_y = (mag_imu[0] - MAG_OFFSET[0]) * MAG_SCALE[0];
		mag_z = (mag_imu[2] - MAG_OFFSET[2]) * MAG_SCALE[2];
	}

//	mag_x = mag_imu[0];
//	mag_y = -mag_imu[1];
//	mag_z = mag_imu[2];
//
//	mag_x = mag_imu[1];
//	mag_y = mag_imu[0];
//	mag_z = mag_imu[2];

	if(calibrating_mag)		//Razor AHRS
	{
		MAG_X[MAG_sample_count] = mag_imu[0];
		MAG_Y[MAG_sample_count] = mag_imu[1];
		MAG_Z[MAG_sample_count++] = mag_imu[2];

		if(MAG_sample_count >= MAG_CALIB_SIZE)
		{
			find_max_min_values();
			for(uint8_t i=0;i<3;i++)
			{
				MAG_OFFSET[i] = (MAG_MAX[i] + MAG_MIN[i])/2.0f;
				MAG_SCALE[i] = 100.0f/(MAG_MAX[i] - MAG_OFFSET[i]);
			}
			calibrating_mag = 0;
			MAG_sample_count = 0;
			store_offsets();
			send_message(CALIBRATE_MAG_MSG_ID);
		}
	}
}

void find_max_min_values()
{
	int i;
	MAG_MAX[0] =-32000;
	MAG_MIN[0] = 32000;
	for (i=0; i<MAG_CALIB_SIZE; i++)
	{
		if (MAG_X[i]>MAG_MAX[0])
		{
			MAG_MAX[0] = MAG_X[i];
		}
		if (MAG_X[i]<MAG_MIN[0])
		{
			MAG_MIN[0] = MAG_X[i];
		}
	}
	MAG_MAX[1] =-32000;
	MAG_MIN[1] = 32000;
	for (i=0; i<MAG_CALIB_SIZE; i++)
	{
		if (MAG_Y[i]>MAG_MAX[1])
		{
			MAG_MAX[1] = MAG_Y[i];
		}
		if (MAG_Y[i]<MAG_MIN[1])
		{
			MAG_MIN[1] = MAG_Y[i];
		}
	}
	MAG_MAX[2] =-32000;
	MAG_MIN[2] = 32000;
	for (i=0; i<MAG_CALIB_SIZE; i++)
	{
		if (MAG_Z[i]>MAG_MAX[2])
		{
			MAG_MAX[2] = MAG_Z[i];
		}
		if (MAG_Z[i]<MAG_MIN[2])
		{
			MAG_MIN[2] = MAG_Z[i];
		}
	}
}

void startup_ground(void)
{
	int flashcount = 0;
	calibrating_IMU = 1;

	IMU_Update_ARDU();
	delay(100);
	IWDG_ReloadCounter();
	delay(1000);
	IWDG_ReloadCounter();
	for(int y=0; y<=5; y++)   // Read first initial ADC values for offset.
		AN_OFFSET[y]=AN[y];

	for(int i=0;i<100;i++)    // We take some readings...
	{
		IWDG_ReloadCounter();
		IMU_Update_ARDU();
		for(int y=0; y<=5; y++)   // Read initial ADC values for offset (averaging).
			AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
		delay(250);
		if(flashcount == 5) {
			GPIOF->ODR ^= GPIO_Pin_11;
			flashcount = 0;
		}
		flashcount++;
	}
    AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
	store_offsets();
	send_message(CALIBRATE_IMU_MSG_ID);
	calibrating_IMU = 0;
}


void HMC5883_calculate(float roll, float pitch)
{
  float Head_X;
  float Head_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  // Tilt compensated Magnetic field X component:
  Head_X = mag_x*cos_pitch+mag_y*sin_roll*sin_pitch+mag_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = mag_y*cos_roll-mag_z*sin_roll;
  // Magnetic Heading
  Heading = atan2(-Head_Y,Head_X);

  // Optimization for external DCM use. Calculate normalized components
  Heading_X = cos(Heading);
  Heading_Y = sin(Heading);
}

void IMU_setup()
{
	delay(2000);
	MPU6050_I2C_Init();				//hardware I2C initialization
	delay(2000);

	MPU6050_Initialize();
	delay(50);
	MPU6050_Aux_Bypass_Enable();	//enable direct access to HMC5883L
	delay(50);
	HMC5883L_Initialize();		//initialize chip with proper data and set gain
	MPU6050_ConfigMag();		//disable direct access to HMC5883L and initialize MPU6050
}

void zero_altitude()
{
	float temp;
	char i=0;

	while((temp = Baro_update(0))==-100)
	{  // return latest or last known pressure_altitude
		delay(1000);
		if(i++>10)
			return;
	}
	ground_alt_offset = temp;
	store_offsets();
	send_message(CALIBRATE_ALTITUDE_MSG_ID);
}

void mag_calib()
{
	calibrating_mag = 1;
	MAG_sample_count = 0;
}
