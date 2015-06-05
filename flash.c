#include "flash.h"

//Store the computed offsets to internal FLASH
void store_offsets()
{
	union
	{
		float val;
		struct
		{
			uint8_t bytes[4];
		};
	}extract_float;

	uint32_t ADDRESS = BACKUP_ADDRESS;
	uint8_t i,j;

	IWDG_ReloadCounter();
	FLASH_Unlock();				//unlock FLASH to enable writing
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);	//clear any pending flags
	FLASH_EraseSector(BACKUP_SECTOR,VoltageRange_3);		//Erase the sector where data is to be written too

	FLASH_ProgramHalfWord(ADDRESS, ASI_ZERO);
	ADDRESS+=2;
	for(j=0;j<6;j++)
	{
		extract_float.val = AN_OFFSET[j];
		for(i=0;i<4;i++)
		{
			FLASH_ProgramByte(ADDRESS, extract_float.bytes[i]);
			ADDRESS+=1;
		}
	}

	for(i=0;i<3;i++)
	{
		FLASH_ProgramHalfWord(ADDRESS, MAG_OFFSET[i]);
		ADDRESS+=2;
	}

	for(j=0;j<3;j++)
	{
		extract_float.val = MAG_SCALE[j];
		for(i=0;i<4;i++)
		{
			FLASH_ProgramByte(ADDRESS, extract_float.bytes[i]);
			ADDRESS+=1;
		}
	}
	FLASH_ProgramHalfWord(ADDRESS, ground_alt_offset);
	ADDRESS+=2;

	for(i=0;i<8;i++)
	{
		FLASH_ProgramHalfWord(ADDRESS, RC_trim[i]);
		ADDRESS+=2;
	}
	FLASH_Lock();
}

//Restore the stored offsets for use
void restore_offsets()
{
	union
	{
		float val;
		struct
		{
			uint8_t bytes[4];
		};
	}extract_float;

	uint32_t ADDRESS = BACKUP_ADDRESS;
	uint8_t i,j;

	ASI_ZERO = *(uint16_t*)ADDRESS;
	ADDRESS+=2;

	for(j=0;j<6;j++)
	{
		  for(i=0;i<4;i++)
		  {
			  extract_float.bytes[i] = *(uint8_t*)ADDRESS;
			ADDRESS+=1;
		  }
		  AN_OFFSET[j] = extract_float.val;
	  }

	for(i=0;i<3;i++)
	{
		MAG_OFFSET[i] = *(uint16_t*)ADDRESS;
		ADDRESS+=2;
	}
	for(j=0;j<3;j++)
	{
		  for(i=0;i<4;i++)
		  {
			  extract_float.bytes[i] = *(uint8_t*)ADDRESS;
			ADDRESS+=1;
		  }
		  MAG_SCALE[j] = extract_float.val;
	}

	ground_alt_offset = *(uint16_t*)ADDRESS;
	ADDRESS+=2;

	for(i=0;i<8;i++)
	{
		RC_trim[i] = *(uint16_t*)ADDRESS;
		ADDRESS+=2;
	}
}
