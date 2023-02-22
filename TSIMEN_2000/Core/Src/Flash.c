#include "FLASH.h"
#include <stdio.h>

FLASH_ProcessTypeDef p_Flash;
uint16_t STM32FLASH_BUFFER[STM32_SECTOR_SIZE/2];

/**
 * @brief read half word
 * 
 * @param halfaddr 
 * @return uint16_t 
 */
uint16_t STM32FLASH_ReadHalfWord(uint32_t halfaddr)
{
    return *(__IO uint16_t *)halfaddr;
}

/**
 * @brief write flash in force
 * 
 * @param writeaddr 
 * @param pbuffer 
 * @param length 
 */
void STM32FLASH_WriteForce(uint32_t writeaddr,uint16_t *pbuffer,uint16_t length)
{
    uint16_t i;
    for (i=0;i<length;i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,writeaddr,pbuffer[i]);
        writeaddr = writeaddr + 2;
    }
}

/**
 * @brief flash write
 * 
 * @param writeaddr 
 * @param pbuffer 
 * @param length 
 */
void STM32FLASH_Write(uint32_t writeaddr,uint16_t *pbuffer,uint16_t length)
{
    uint32_t secpos;        //Number of pages
    uint32_t offaddr;       //Offset address
    uint16_t secoff;        //Offset address in sector
    uint16_t secremain;     //Sector reamining space
    uint16_t i;

    if (writeaddr<STM32_FLASH_BASE||(writeaddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
    {
        printf("Wrong Address\r\n");
        return;
    }

    HAL_FLASH_Unlock();                             // Unlock flash write
    offaddr = writeaddr - STM32_FLASH_BASE;         // Offset address
    secpos = offaddr/STM32_SECTOR_SIZE;             // Number of pages
    secoff = (offaddr%STM32_SECTOR_SIZE);           // Offset address in sector
    secremain = STM32_SECTOR_SIZE/2-secoff;         // Sector remaining space
    
    if (length <= secremain){secremain = length;}

    while (1)
    {  
        STM32FLASH_Read(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32FLASH_BUFFER,STM32_SECTOR_SIZE/2);
        for (i=0;i<secremain;i++)
        {
            if(STM32FLASH_BUFFER[secoff+i] != 0xFFFF)
            break;
        }
        if (i<secremain)
        {
            STM32FLASH_ErasePage(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE);
            FLASH_WaitForLastOperation(STM32_FLASH_TIMEOUT);
            CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
            for (i=0;i<secremain;i++)
            {
                STM32FLASH_BUFFER[i+secoff] = pbuffer[i];
            }
            STM32FLASH_WriteForce(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32FLASH_BUFFER,STM32_SECTOR_SIZE/2);
        }
        else
        {
            FLASH_WaitForLastOperation(STM32_FLASH_TIMEOUT);
            STM32FLASH_WriteForce(writeaddr,pbuffer,secremain);
        }
        if (length == secremain)
        {break;}
        else
        {
            secpos++;
            secoff=0;
            pbuffer+=secremain;
            writeaddr=secremain*2;
            length-=secremain;
            if(length>(STM32_SECTOR_SIZE/2))
            secremain = STM32_SECTOR_SIZE/2;
            else 
            secremain = length;
        }
    }
    HAL_FLASH_Lock();
}

/**
 * @brief flash read
 * 
 * @param readaddr 
 * @param pbuffer 
 * @param length 
 */
void STM32FLASH_Read(uint32_t readaddr,uint16_t *pbuffer,uint16_t length)
{
    uint16_t i;
    for (i=0;i<length;i++)
    {
        pbuffer[i]=STM32FLASH_ReadHalfWord(readaddr);
        readaddr = readaddr + 2;
    }

}

/**
 * @brief flash erase
 * 
 * @param pageaddr 
 */
void STM32FLASH_ErasePage(uint32_t pageaddr)
{
    p_Flash.ErrorCode = HAL_FLASH_ERROR_NONE;

    #if defined(FLASH_BANK2_END)
    if(pageaddr > FLASH_BANK1_END)
    { 
        /* Proceed to erase the page */
        SET_BIT(FLASH->CR2, FLASH_CR2_PER);
        WRITE_REG(FLASH->AR2, pageaddr);
        SET_BIT(FLASH->CR2, FLASH_CR2_STRT);
    }
    else
    {
    #endif /* FLASH_BANK2_END */
        /* Proceed to erase the page */
        SET_BIT(FLASH->CR, FLASH_CR_PER);
        WRITE_REG(FLASH->AR, pageaddr);
        SET_BIT(FLASH->CR, FLASH_CR_STRT);
    #if defined(FLASH_BANK2_END)
    
    }
    #endif /* FLASH_BANK2_END */
}








