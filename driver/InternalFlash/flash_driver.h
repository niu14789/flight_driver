#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include "stdint.h"
#include "stdbool.h"

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif

#ifndef I8
#define I8 char
#endif

#ifndef I16
#define I16 short
#endif


#ifndef I32
#define I32 int
#endif

#define FMC_PAGE_SIZE                   ((U16)0x800U)

#define FLASH_DATA_BUFFER_LEN           200      //25  words
#define FMC_STICK_STORE_ADDR_INDEX      0       //7800~7809 保存摇杆校准数据 10 words
#define FMC_RF_ID_STORE_ADDR_INDEX      10       //780A~780B 保存RF地址数据
#define FMC_RF_ID_LEN                   2

//extern FLASH flash;

void fmc_erase_page(uint32_t page_start_addr);
uint8_t fmc_erase_page_check(uint32_t page_start_addr);
void fmc_write_words(uint32_t flash_addr,uint32_t *pbuf,uint16_t len);
uint8_t fmc_write_words_check(uint32_t flash_addr,uint32_t *pbuf,uint16_t len);

uint32_t get_32bits_data_checksum(uint32_t *pbuf,uint16_t len);
void set_flash_updata_flag(void);
void clear_flash_updata_flag(void);
void system_data_store(void);
uint8_t system_data_read(void);

#endif
