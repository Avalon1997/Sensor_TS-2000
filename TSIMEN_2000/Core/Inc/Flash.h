#ifndef __STM32FLASH_H__
#define __STM32FLASH_H__

#include "main.h"

// ----- Define Config

#define STM32_FLASH_SIZE        512
#define STM32_SECTOR_SIZE       2048
#define STM32_FLASH_BASE        0x08000000
#define STM32_FLASH_OPTICALPATH  STM32_FLASH_BASE+STM32_SECTOR_SIZE*249
#define STM32_FLASH_ENSAVEADDR1  STM32_FLASH_BASE+STM32_SECTOR_SIZE*250
#define STM32_FLASH_ENSAVEADDR2  STM32_FLASH_BASE+STM32_SECTOR_SIZE*251
#define STM32_FLASH_ENSAVEADDR3  STM32_FLASH_BASE+STM32_SECTOR_SIZE*252
#define STM32_FLASH_ENSAVEADDR4  STM32_FLASH_BASE+STM32_SECTOR_SIZE*253
#define STM32_FLASH_WASAVEADDR  STM32_FLASH_BASE+STM32_SECTOR_SIZE*255
#define STM32_FLASH_ENABLE      1
#define STM32_FLASH_TIMEOUT     50000

// ----- Define Function

uint8_t STM32FLASH_GetStatus(void);
uint8_t STM32FLASH_Wait(uint16_t time);

uint8_t STM32FLASH_WriteHalfWord(uint32_t halfaddr,uint16_t dat);
uint16_t STM32FLASH_ReadHalfWord(uint32_t halfaddr);
void STM32FLASH_Write(uint32_t writeaddr,uint16_t *pbuffer,uint16_t length);
void STM32FLASH_Read(uint32_t readaddr,uint16_t *pbuffer,uint16_t length);
void STM32FLASH_ErasePage(uint32_t pageaddr);

#endif
