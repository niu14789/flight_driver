#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>
#include "flash_driver.h"



//页擦除，只支持单页擦除
void fmc_erase_page(uint32_t page_start_addr)
{
    uint8_t erase_finish_flag = 0;
    while (!erase_finish_flag)
    {
        /* unlock the flash program/erase controller */
        fmc_unlock();
        /* clear all pending flags */
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
        //erase page
        fmc_page_erase(page_start_addr);
        //clear flag
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
        /* lock the main FMC after the erase operation */
        fmc_lock();

        erase_finish_flag = fmc_erase_page_check(page_start_addr);
    }
}
//单页擦除校验
uint8_t fmc_erase_page_check(uint32_t page_start_addr)
{
    uint16_t i;
    uint32_t *ptrd;

    ptrd = (uint32_t *)page_start_addr;

    /* check flash whether has been erased */
    for (i = 0U; i < 256; i++)
    {
        if (0xFFFFFFFFU != (*ptrd))
        {
            return 0;
        }
        else
        {
            ptrd++;
        }
    }
    return 1;
}


//多字节写
void fmc_write_words(uint32_t flash_addr, uint32_t *pbuf, uint16_t len)
{
    uint16_t i;
    uint32_t address;
    
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = flash_addr;

    /* program flash */
    for (i = 0; i < len; i++)
    {
        fmc_word_program(address, pbuf[i]);
        address += 4U;
    }
    /* lock the main FMC after the program operation */
    fmc_lock();
}

//多字节写校验
// 0:check failed, 1:check success
uint8_t fmc_write_words_check(uint32_t flash_addr, uint32_t *pbuf, uint16_t len)
{
    uint16_t i;
    uint32_t *ptrd = (uint32_t *)flash_addr;

    /* check flash whether has been erased */
    for (i = 0U; i < len; i++)
    {
        if ((*pbuf) != (*ptrd))
        {
            return 0;
        }
        else
        {
            pbuf++; // add this line by Ryan Huang 2019.05.13 16:42
            ptrd++;
        }
    }
    return 1;
}

//多字节读
void fmc_read_words(uint32_t flash_addr, uint32_t *pbuf, uint16_t len)
{
    uint16_t i;

    uint32_t *ptrd = (uint32_t *)flash_addr;

    for (i = 0U; i < len; i++)
    {
        *pbuf = *ptrd;
        ptrd++;
        pbuf++;
    }
}



