
#ifndef MS5611_H
#define MS5611_H

#include "stm32f4xx.h"

#define MS561101BA_I2C                  I2C1
#define MS561101BA_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define MS561101BA_I2C_Port             GPIOB
#define MS561101BA_I2C_SCL_Pin          GPIO_Pin_8
#define MS561101BA_I2C_SDA_Pin          GPIO_Pin_9
#define MS561101BA_I2C_RCC_Port         RCC_AHB1Periph_GPIOB
#define MS561101BA_I2C_Speed            100000

#define MS561101BA_ADDRESS            0x77
#define MS561101BA_DEFAULT_ADDRESS    (MS561101BA_ADDRESS<<1)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

extern float temperature;
extern uint32_t pressure;
extern float altitude;

void i2c_MS561101BA_reset();
void i2c_MS561101BA_readCalibration();
void  Baro_init();
int i2c_MS561101BA_UT_Start();
int i2c_MS561101BA_UP_Start ();
int i2c_MS561101BA_UP_Read ();
int i2c_MS561101BA_UT_Read();
void i2c_MS561101BA_Calculate();
float Baro_update(uint32_t currentTime);
int MS561101BA_I2C_ADDRWrite(u8 slaveAddr, u8 WriteAddr);
void MS561101BA_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 WriteAddr);
int MS561101BA_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);


#endif


