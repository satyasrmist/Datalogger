#ifndef ARDUIMU_H
#define ARDUIMU_H

#include "MPU6050.h"
#include "stm32f4xx_gpio.h"
#include "math.h"
#include "mavlink_types.h"
#include "common.h"
#include "MS5611.h"
#include "flash.h"
#include "stm32f4xx_iwdg.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define BOARD_VERSION 3

//OUTPUTMODE=1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 will print accelerometer only data
#define OUTPUTMODE 1

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation (m/s), 0=do not use speed filter

#define USE_MAGNETOMETER 1 // use 1 if you want to make yaw gyro drift corrections using the optional magnetometer

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

////////////////CORRECTIONhttp://diydrones.com/forum/topics/arduimu-v3-all-axes-output-wrong-values?id=705844%3ATopic%3A905476&page=1#comments////////////////////
#define GRAVITY 8192  // This equivalent to 1G in the raw data coming from the accelerometer
////////////////CORRECTIONhttp://diydrones.com/forum/topics/arduimu-v3-all-axes-output-wrong-values?id=705844%3ATopic%3A905476&page=1#comments////////////////////

#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

// MPU6000 sensibility  (theorical 0.0152 => 1/65.6LSB/deg/s at 500deg/s) (theorical 0.0305 => 1/32.8LSB/deg/s at 1000deg/s) ( 0.0609 => 1/16.4LSB/deg/s at 2000deg/s)
#define Gyro_Gain_X 0.0609
#define Gyro_Gain_Y 0.0609
#define Gyro_Gain_Z 0.0609
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

//#define Kp_ROLLPITCH 0.015
//#define Ki_ROLLPITCH 0.000010

////////////////CORRECTIONhttp://diydrones.com/forum/topics/arduimu-v3-all-axes-output-wrong-values?id=705844%3ATopic%3A905476&page=1#comments////////////////////
#define Kp_ROLLPITCH 1.515/GRAVITY
#define Ki_ROLLPITCH 0.00101/GRAVITY
////////////////CORRECTIONhttp://diydrones.com/forum/topics/arduimu-v3-all-axes-output-wrong-values?id=705844%3ATopic%3A905476&page=1#comments////////////////////

#define Kp_YAW 1.2
//#define Kp_YAW 2.5      //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

#define ADC_WARM_CYCLES 75

#define FALSE 0
#define TRUE 1

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

#define MAG_CALIB_SIZE 	50

#define GYRO_SENSITIVITY  16.4		// 16.4 LSB/deg/s  1 LSB = 1/16.4 deg/sec for full scale +- 2000 deg/s

extern float G_Dt;    // Integration time (DCM algorithm)

extern float AN[8]; //array that store the 6 ADC filtered data
extern float AN_OFFSET[8]; //Array that stores the Offset of the gyros

extern float Accel_Vector[3]; //Store the acceleration in a vector
extern float Gyro_Vector[3];//Store the gyros rutn rate in a vector
extern float Omega_Vector[3]; //Corrected Gyro_Vector data
extern float Omega_P[3];//Omega Proportional correction
extern float Omega_I[3];//Omega Integrator
extern float Omega[3];

// Euler angles
extern float roll;
extern float pitch;
extern float yaw;

extern float errorRollPitch[3];
extern float errorYaw[3];
extern float errorCourse;

extern float DCM_Matrix[3][3];
extern float Update_Matrix[3][3]; //Gyros here

extern float Temporary_Matrix[3][3];

extern int SENSOR_SIGN[];

extern int mag_x;
extern int mag_y;
extern int mag_z;
extern float Heading;
extern float Heading_X;
extern float Heading_Y;
extern uint16_t ground_alt_offset;
extern int16_t MAG_OFFSET[3];
extern float MAG_SCALE[3];

extern float MPU_TEMP;
extern volatile uint8_t calibrating_IMU,calibrating_mag;
extern int16_t MAG_X[MAG_CALIB_SIZE],MAG_Y[MAG_CALIB_SIZE],MAG_Z[MAG_CALIB_SIZE];

extern int16_t mag_imu[3];

float read_adc(int select);

void IMU_Update_ARDU();
void startup_ground(void);
void HMC5883_calculate(float roll, float pitch);
void IMU_setup();
void zero_altitude();
void mag_calib();
void find_max_min_values();


#endif
