
#include "MS5611.h"
#include "delay.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "math.h"
#include "MPU6050.h"


uint32_t pressure=101325;
float BaroAlt = -100;
float temperature = 0;
float altitude = 0;

static struct {
	// sensor registers from the MS561101BA datasheet
	uint16_t c[7];
	union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
	union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
	uint8_t  state;
	uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
	//  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
	MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_RESET);
}

void i2c_MS561101BA_readCalibration(){
//	union {uint16_t val; uint8_t raw[2]; } data;
	uint8_t pbuffer[2];
	uint8_t i=0;
	uint8_t success;

	delay(10);

	for(i=0;i<6;i++) {

		success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , (0xA2+2*i), 2);
		if(success==0){
			MPU6050_I2C_Reboot();
			i2c_MS561101BA_readCalibration();
			return;
		}
		ms561101ba_ctx.c[i+1] = ((uint16_t)pbuffer[0] <<8) + pbuffer[1];
	}
}

void  Baro_init()
{
	delay(10);
	i2c_MS561101BA_reset();
	delay(100);
	i2c_MS561101BA_readCalibration();
}

// read uncompensated temperature value: send command first
int i2c_MS561101BA_UT_Start() {
	//  i2c_rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
	//  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection

	uint8_t success;

	success = MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_TEMPERATURE + OSR);

	if(success == 0){
//		target_wp_index = 151;
		MPU6050_I2C_Reboot();
//		i2c_MS561101BA_UT_Start();
		return 0;
	}
	return 1;
}

// read uncompensated pressure value: send command first
int i2c_MS561101BA_UP_Start() {
//	i2c_rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
//	i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
	uint8_t success;

	success = MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_PRESSURE + OSR);

	if(success == 0){
		MPU6050_I2C_Reboot();
		return 0;
	}
	return 1;
}

// read uncompensated pressure value: read result bytes
int i2c_MS561101BA_UP_Read () {

	uint8_t pbuffer[3];
	uint8_t success;
//	i2c_rep_start(MS561101BA_ADDRESS + 0);
//	i2c_write(0);
//	i2c_rep_start(MS561101BA_ADDRESS + 1);
//	ms561101ba_ctx.up.raw[2] = i2c_readAck();
//	ms561101ba_ctx.up.raw[1] = i2c_readAck();
//	ms561101ba_ctx.up.raw[0] = i2c_readNak();

	success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , 0x00, 3);

	if(success == 0){
		MPU6050_I2C_Reboot();

//		i2c_MS561101BA_UP_Read();
		return 0;
	}

	ms561101ba_ctx.up.raw[2] = pbuffer[0];
	ms561101ba_ctx.up.raw[1] = pbuffer[1];
	ms561101ba_ctx.up.raw[0] = pbuffer[2];

	return 1;
}

// read uncompensated temperature value: read result bytes
int i2c_MS561101BA_UT_Read() {
	uint8_t pbuffer[3];
	uint8_t success;
//	i2c_rep_start(MS561101BA_ADDRESS + 0);
//	i2c_write(0);
//	i2c_rep_start(MS561101BA_ADDRESS + 1);
//	ms561101ba_ctx.ut.raw[2] = i2c_readAck();
//	ms561101ba_ctx.ut.raw[1] = i2c_readAck();
//	ms561101ba_ctx.ut.raw[0] = i2c_readNak();
	success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , 0x00, 3);

	if(success == 0){
		MPU6050_I2C_Reboot();
//		i2c_MS561101BA_UT_Read();
		return 0;
	}
	ms561101ba_ctx.ut.raw[2] = pbuffer[0];
	ms561101ba_ctx.ut.raw[1] = pbuffer[1];
	ms561101ba_ctx.ut.raw[0] = pbuffer[2];
	return 1;
}

void i2c_MS561101BA_Calculate() {
	int64_t dT   = ms561101ba_ctx.ut.val - ((uint32_t)ms561101ba_ctx.c[5] << 8);  //int32_t according to the spec, but int64_t here to avoid cast after
	int64_t off  = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
	int64_t sens = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);
	pressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
	temperature = (float) (2000+ ((dT*ms561101ba_ctx.c[6])>>23))/100.0;
}

float Baro_update(uint32_t currentTime)
{
	if (currentTime < ms561101ba_ctx.deadline)
		return BaroAlt;
	ms561101ba_ctx.deadline = 12;
	switch (ms561101ba_ctx.state)
	{
	case 0:
		if(i2c_MS561101BA_UT_Start())
		{
			ms561101ba_ctx.state++; ms561101ba_ctx.deadline = 12; //according to the specs, the pause should be at least 8.22ms
		}
		else
		{
			ms561101ba_ctx.state =0; ms561101ba_ctx.deadline = 12;
		}
		break;
	case 1:
		if(i2c_MS561101BA_UT_Read())
		{
			ms561101ba_ctx.state++; ms561101ba_ctx.deadline = 12; //according to the specs, the pause should be at least 8.22ms
		}
		else
		{
			ms561101ba_ctx.state =0; ms561101ba_ctx.deadline = 12;
		}
		break;
	case 2:
		if(i2c_MS561101BA_UP_Start())
		{
			ms561101ba_ctx.state++; ms561101ba_ctx.deadline = 12; //according to the specs, the pause should be at least 8.22ms
		}
		else
		{
			ms561101ba_ctx.state =2; ms561101ba_ctx.deadline = 12;
		}
		break;
	case 3:
		if(i2c_MS561101BA_UP_Read())
		{
			i2c_MS561101BA_Calculate();
			if((pressure>=1000) && (pressure<= 120000))
				BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 44330.0f;
			ms561101ba_ctx.state = 0; ms561101ba_ctx.deadline = 15;
		}
		else
		{
			ms561101ba_ctx.state = 2; ms561101ba_ctx.deadline = 12;
		}
		break;
	}
	return BaroAlt;
}


int MS561101BA_I2C_ADDRWrite(u8 slaveAddr, u8 WriteAddr)
{
//  ENTR_CRT_SECTION();
	int TimeOut = 10000;
	  while(I2C_GetFlagStatus(MS561101BA_I2C, I2C_FLAG_BUSY)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
  /* Send START condition */
  I2C_GenerateSTART(MS561101BA_I2C, ENABLE);
  /* Test on EV5 and clear it */
//  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_MODE_SELECT)){
  while (!I2C_GetFlagStatus(MS561101BA_I2C,I2C_FLAG_SB)){
		TimeOut--;
		if (TimeOut == 0)
		{
//			if(WriteAddr == (MS561101BA_TEMPERATURE + OSR))
////				target_wp_index = 1;
//			else
////				target_wp_index = 5;
			I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
			return 0;
		}
	}
  /* Send HMC5883 address for write */
  I2C_Send7bitAddress(MS561101BA_I2C, slaveAddr, I2C_Direction_Transmitter);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		TimeOut--;
		if (TimeOut == 0)
		{
//			if(WriteAddr == (MS561101BA_TEMPERATURE + OSR))
//				target_wp_index = 2;
//			else
//				target_wp_index = 6;
			I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
			return 0;
		}
	}
  /* Send the MS561101BA internal address to write to */
  I2C_SendData(MS561101BA_I2C, WriteAddr);
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
	  		TimeOut--;
	  		if (TimeOut == 0)
	  		{
//				if(WriteAddr == (MS561101BA_TEMPERATURE + OSR))
//					target_wp_index = 3;
//				else
//					target_wp_index = 7;
	  			I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
	  			return 0;
	  		}
  }
//  /* Send the byte to be written */
//  I2C_SendData(MS561101BA_I2C, *pBuffer);
//  /* Test on EV8 and clear it */
//  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  /* Send STOP condition */
  I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
  while(I2C_GetFlagStatus(MS561101BA_I2C, I2C_FLAG_STOPF));
 // EXT_CRT_SECTION();
  return 1;
}

void MS561101BA_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 WriteAddr)
{
//  ENTR_CRT_SECTION();

  /* Send START condition */
  I2C_GenerateSTART(MS561101BA_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send HMC5883 address for write */
  I2C_Send7bitAddress(MS561101BA_I2C, slaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MS561101BA internal address to write to */
  I2C_SendData(MS561101BA_I2C, WriteAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(MS561101BA_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
 // EXT_CRT_SECTION();

}

int MS561101BA_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
	int TimeOut = 10000;
	I2C_AcknowledgeConfig(MS561101BA_I2C, ENABLE);
	  /* While the bus is busy */
	  while(I2C_GetFlagStatus(MS561101BA_I2C, I2C_FLAG_BUSY)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
	  /* Send START condition */
	  I2C_GenerateSTART(MS561101BA_I2C, ENABLE);
	  /* Test on EV5 and clear it */
//	  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_MODE_SELECT)){
	  while (!I2C_GetFlagStatus(MS561101BA_I2C,I2C_FLAG_SB)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
	  /* Send MS561101BA_Magn address for write */ // Send MS561101BA address for write
	  I2C_Send7bitAddress(MS561101BA_I2C, slaveAddr, I2C_Direction_Transmitter);
	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
//	  /* Clear EV6 by setting again the PE bit */
//	  I2C_Cmd(MS561101BA_I2C, ENABLE);
	  /* Send the MS561101BA's internal address to write to */
	  I2C_SendData(MS561101BA_I2C, ReadAddr);
	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
	  /* Send STRAT condition a second time */
	  I2C_GenerateSTART(MS561101BA_I2C, ENABLE);
	  /* Test on EV5 and clear it */
//	  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_MODE_SELECT)){
	  while (!I2C_GetFlagStatus(MS561101BA_I2C,I2C_FLAG_SB)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
	  /* Send MS561101BA address for read */
	  I2C_Send7bitAddress(MS561101BA_I2C, slaveAddr, I2C_Direction_Receiver);
	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
	}
	  /* While there is data to be read */
	  while(NumByteToRead)
	  {
  	  		TimeOut--;
  	  		if (TimeOut == 0)
  	  		{
  	  			I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
  	  			return 0;
  	  		}
	    if(NumByteToRead == 1)
	    {
	      /* Disable Acknowledgement */
	    	I2C_NACKPositionConfig(MS561101BA_I2C, I2C_NACKPosition_Current);
	      I2C_AcknowledgeConfig(MS561101BA_I2C, DISABLE);
//	      /* Send STOP Condition */
	      I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
	      while(I2C_GetFlagStatus(MS561101BA_I2C, I2C_FLAG_STOPF)); // stop bit flag
	    }
		while (!I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			TimeOut--;
			if (TimeOut == 0)
			{
				I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
				return 0;
			}
		}
	    /* Test on EV7 and clear it */
//	    if(I2C_CheckEvent(MS561101BA_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
//	    {
	      /* Read a byte from the MS561101BA */
	      *pBuffer = I2C_ReceiveData(MS561101BA_I2C);
	      /* Point to the next location where the byte read will be saved */
	      pBuffer++;
	      /* Decrement the read bytes counter */
	      NumByteToRead--;
//	    }
//	    if(NumByteToRead == 0)
//	    {
//	      /* Send STOP Condition */
//	      I2C_GenerateSTOP(MS561101BA_I2C, ENABLE);
//	    }
	  }
	  /* Enable Acknowledgement to be ready for another reception */
	  I2C_AcknowledgeConfig(MS561101BA_I2C, ENABLE);
	//  EXT_CRT_SECTION();
	  return 1;
}
