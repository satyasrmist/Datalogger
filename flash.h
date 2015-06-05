#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "delay.h"
#include "arduimu.h"
#include "air_speed.h"

#define BACKUP_SECTOR	FLASH_Sector_11
#define BACKUP_ADDRESS	0x080E0000

//Store the computed offsets to internal FLASH
void store_offsets();

//Restore the stored offsets for use
void restore_offsets();
#endif
