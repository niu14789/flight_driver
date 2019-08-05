#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "gd32f30x.h"


#ifndef U8
#define U8 unsigned char
#endif
 
#ifndef U16
#define U16 unsigned short
#endif
 
 
#ifndef U32
#define U32 unsigned int
#endif
 

#ifndef NULL
#define NULL 0
#endif


typedef struct _SPIBaseDefine_ {
    U32 m_dwRegBaseAddr; // i2C register base address
    U32 m_dwCsGpioPeriph;
    U32 m_dwCsGpioPin;
}SPIBaseDefine;



typedef enum _SPI_DEVICE_{
    SDC_SPI_IDX = 0,
    IMU_SPI_IDX,
    G24_SPI_IDX
}SPI_DEVICE;


SPIBaseDefine *SpiOpen(SPI_DEVICE eDevice);

U8 SpiWriteReadByte(SPIBaseDefine *poHandle, U8 WriteByte);

int SpiReadDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataOut, int iWantReadLen);

int SpiWriteDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataIn, int iWantWriteLen);

#endif




