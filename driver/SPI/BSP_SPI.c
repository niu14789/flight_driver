/*********************************************************************************************************
*
*    模 块 名: BSP_SPI
*    文 件 名: BSP_SPI.c
*    版本信息: V1.0
*    注    意：No
*    修改记录：No
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-28               V1.0
*********************************************************************************************************/ 

#include "BSP_SPI.h"
#include "systick.h"

#define SPI_WRITE_TIME_OUT_MS 50
#define SPI_READ_TIME_OUT_MS 50

#define  SPI_TIME_OUT_MS          100

SPIBaseDefine g_oSdc = {SPI0, GPIOA, GPIO_PIN_4};
SPIBaseDefine g_oImu = {SPI1, GPIOB, GPIO_PIN_12};
SPIBaseDefine g_o24G = {SPI2, GPIOA, GPIO_PIN_15};


//           CS        SCK          MISO            MOSI
// SPI0      PA4       PA5          PA6             PA7------SD card(un-used)
// SPI1      PB12      PB13         PB14            PB15------ICM20600(Max frequence 10MHz, we set as 7.5MHz that is PCLK1/8)
// SPI2      PA15      PB3          PB4             PB5------2.4G
SPIBaseDefine *SpiOpen(SPI_DEVICE eDevice)
{
    spi_parameter_struct oSpiInit;
    
 //   rcu_periph_clock_enable(RCU_AF);
    switch (eDevice)
    {
    case SDC_SPI_IDX:
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_SPI0);
        gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);      // CS
        gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);       // SCK
        gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6); // MISO
        gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);       // MOSI

        //SPI0 parameter config
        oSpiInit.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
        oSpiInit.device_mode = SPI_MASTER;
        oSpiInit.frame_size = SPI_FRAMESIZE_8BIT;
        oSpiInit.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        oSpiInit.nss = SPI_NSS_SOFT;
        oSpiInit.prescale = SPI_PSC_256; // 120MHz /256 = 468750Hz
        oSpiInit.endian = SPI_ENDIAN_MSB;;
        spi_init(SPI0, &oSpiInit);
        spi_enable(SPI0);
        spi_nssp_mode_enable(SPI0);
        spi_nss_output_enable(SPI0);
        return &g_oSdc;
    case IMU_SPI_IDX:
        rcu_periph_clock_enable(RCU_GPIOB);
        rcu_periph_clock_enable(RCU_SPI1);
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);      // CS
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);       // SCK
        gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14); // MISO
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);       // MOSI

        //SPI1 parameter config
        oSpiInit.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
        oSpiInit.device_mode = SPI_MASTER;
        oSpiInit.frame_size = SPI_FRAMESIZE_8BIT;
        oSpiInit.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
        oSpiInit.nss = SPI_NSS_SOFT;
        oSpiInit.prescale = SPI_PSC_8; // 60MHz /8 = 7.5MHz
        oSpiInit.endian = SPI_ENDIAN_MSB;;
        spi_init(SPI1, &oSpiInit);
        spi_nssp_mode_enable(SPI1);
        spi_nss_output_enable(SPI1);
        spi_enable(SPI1);
        return &g_oImu;
    case G24_SPI_IDX:
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_GPIOB);
        rcu_periph_clock_enable(RCU_SPI2);
        gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);     // CS
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);       // SCK
        gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // MISO
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);       // MOSI

        //SPI2 parameter config
        oSpiInit.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
        oSpiInit.device_mode = SPI_MASTER;
        oSpiInit.frame_size = SPI_FRAMESIZE_8BIT;
        oSpiInit.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        oSpiInit.nss = SPI_NSS_SOFT;
        oSpiInit.prescale = SPI_PSC_32; // 60MHz /32 = 1.875MHz
        oSpiInit.endian = SPI_ENDIAN_MSB;;
        spi_init(SPI2, &oSpiInit);
        spi_enable(SPI2);
        spi_nssp_mode_enable(SPI2);
        spi_nss_output_enable(SPI2);
        return &g_o24G;
    default:
        return NULL;
    }
}



/******************************************************************************
Function Name : U8 SpiWriteReadByte(SPIBaseDefine *poHandle, U8 WriteByte)
Description   : write/read one byte data on SPI bus
Parameter     : 1.poHandle[in]:spi device handle
                    2.WriteByte[in]:write data
Return        : USART_ERR_PORT_NUM : usart handle is null
                 >=0 : the bytes of the data has been read
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
U8 SpiWriteReadByte(SPIBaseDefine *poHandle, U8 WriteByte)
{
    U32  dwTimeStart = 0;
    U32  dwTimeEnd = 0;

    dwTimeEnd = dwTimeStart = GetTickCountMs();
    while (RESET == spi_i2s_flag_get(poHandle->m_dwRegBaseAddr, SPI_FLAG_TBE))
    {
        if ((dwTimeEnd - dwTimeStart) > SPI_TIME_OUT_MS)
        {
            return 1;
        }
        dwTimeEnd = GetTickCountMs();
    }
    spi_i2s_data_transmit(poHandle->m_dwRegBaseAddr, WriteByte);

    dwTimeEnd = dwTimeStart = GetTickCountMs();
    while (RESET == spi_i2s_flag_get(poHandle->m_dwRegBaseAddr, SPI_FLAG_RBNE))
    {
        if ((dwTimeEnd - dwTimeStart) > SPI_TIME_OUT_MS)
        {
            return 2;
        }
        dwTimeEnd = GetTickCountMs();
    }
    return spi_i2s_data_receive(poHandle->m_dwRegBaseAddr);

}

/******************************************************************************
Function Name : int SpiReadDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataOut, int iWantReadLen)
Description   : read data from SPI bus
Parameter     : 1.poHandle[in]:spi device handle
                    2.byRegAddr[in]: register start address
                    3.pbyDataOut[out]:read data buffer
                    4.iWantReadLen[in]:want read length
Return        : the bytes of the data has been read
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int SpiReadDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataOut, int iWantReadLen)
{
    int i;

    gpio_bit_reset(poHandle->m_dwCsGpioPeriph, poHandle->m_dwCsGpioPin);  // active CS
    SpiWriteReadByte(poHandle, byRegAddr|0x80);
    for (i = 0; i < iWantReadLen; i++)
    {
        pbyDataOut[i] = SpiWriteReadByte(poHandle, 0xff);
    }
    gpio_bit_set(poHandle->m_dwCsGpioPeriph, poHandle->m_dwCsGpioPin);  // inactive CS
    return iWantReadLen;
}

/******************************************************************************
Function Name : int SpiWriteDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataIn, int iWantWriteLen)
Description   : write data to SPI bus
Parameter     : 1.poHandle[in]:spi device handle
                    2.byRegAddr[in]: register start address
                    3.pbyDataIn[in]:write data buffer
                    4.iWantReadLen[in]:want write length
Return        : the bytes of the data has been wrote
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int SpiWriteDataBlock(SPIBaseDefine *poHandle, U8 byRegAddr, U8 *pbyDataIn, int iWantWriteLen)
{
    int i;

    gpio_bit_reset(poHandle->m_dwCsGpioPeriph, poHandle->m_dwCsGpioPin);  // active CS
    SpiWriteReadByte(poHandle, byRegAddr&0x7f);
    for (i = 0; i < iWantWriteLen; i++)
    {
        SpiWriteReadByte(poHandle, pbyDataIn[i]);
    }
    gpio_bit_set(poHandle->m_dwCsGpioPeriph, poHandle->m_dwCsGpioPin);  // inactive CS
    return iWantWriteLen;
}

