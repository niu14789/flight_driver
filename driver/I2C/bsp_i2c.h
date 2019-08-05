#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_


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
 
#ifndef I8
#define I8 char
#endif
 
#ifndef I16
#define I16 short
#endif
 
 
#ifndef I32
#define I32 int
#endif

#ifndef NULL
#define NULL 0
#endif



typedef enum _I2C_DEVICE_{
    IMU_I2C_IDX = 3,
    MAG_I2C_IDX = 4,
    BARO_I2C_IDX = 5
}I2C_DEVICE;


//#define MASTER_ADDR  (0X0D)


typedef enum _I2C_PORT_{
    I2C_PORT0 = 0,   // PB6(SCL), Pb7(SDA) for ST480
    I2C_PORT1,       // PB10(SCL), Pb11(SDA) for DPS310
    I2C_PORT_LAST
}I2C_PORT;

#define I2C_BUFFER_SIZE   128


typedef struct _I2cBaseDefine_ {
    U32 m_dwRegBaseAddr; // i2C register base address
    U32 m_dwDeviceSpeed; // device operate speed
    U8 m_byDeviceAddr;   // I2C slave device address. Write is DeviceAddr*2, read is DeviceAddr*2+1
    U8 m_byDeviceUseFlag; // 0:not use, 1:has been used.
}I2cBaseDefine;




#define IIC0_SCL_PERIPH_CLOCK  RCU_GPIOB
#define IIC0_SDA_PERIPH_CLOCK  RCU_GPIOB

#define IIC0_SCL_GPIO_PORT  GPIOB
#define IIC0_SCL_GPIO_PIN   GPIO_PIN_6
#define IIC0_SDA_GPIO_PORT  GPIOB
#define IIC0_SDA_GPIO_PIN   GPIO_PIN_7

#define IIC0_PERIPH_CLOCK   RCU_I2C0


#define IIC1_SCL_PERIPH_CLOCK  RCU_GPIOB
#define IIC1_SDA_PERIPH_CLOCK  RCU_GPIOB

#define IIC1_SCL_GPIO_PORT  GPIOB
#define IIC1_SCL_GPIO_PIN   GPIO_PIN_10
#define IIC1_SDA_GPIO_PORT  GPIOB
#define IIC1_SDA_GPIO_PIN   GPIO_PIN_11


I2cBaseDefine *I2cOpen(I2C_DEVICE eDevice);

void I2cClose(I2cBaseDefine *poHandle);

int I2cWriteBytes(I2cBaseDefine *poHandle, U8 *pbyBuffer, U8 byBufferLen, U8 byRegOffset);

int I2cWriteByte(I2cBaseDefine *poHandle, U8 byOffsetAddr, U8 byData);

int I2cReadBytes(I2cBaseDefine *poHandle, U8* pbyBuffer, U8 byRegOffset, U16 wWantReadBytes);

int I2cWriteBytesAndRead(I2cBaseDefine *poHandle, U8 *pbyWriteBuff, U8 byWBytes, U8 *pbyReadBuff, U8 byRBytes);


#endif

