/*********************************************************************************************************
*
*    模 块 名: bsp_i2c
*    文 件 名: bsp_i2c.c
*    版本信息: V1.0
*    注    意：No
*    修改记录：No
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-23               V1.0
*********************************************************************************************************/ 

#include "bsp_i2c.h"
#define MAX_I2C_DEVICE_NUMBER 6


#define I2C_PORT0_SPEED  400000
#define I2C_PORT1_SPEED  400000

#define BARO_I2C_SPEED   I2C_PORT1_SPEED
#define MAG_I2C_SPEED    I2C_PORT1_SPEED
#define IMU_I2C_SPEED    I2C_PORT1_SPEED

#define BARO_I2C_ADDRESS   0xee 
#define MAG_I2C_ADDRESS    0x18
//#define IMU_I2C_ADDRESS    0xd2
#define IMU_I2C_ADDRESS    0xd0

static I2cBaseDefine gs_aoI2cHandle[MAX_I2C_DEVICE_NUMBER] = {
    {I2C0,0,0,0},
    {I2C0,0,0,0},
    {I2C0,0,0,0},
    {I2C1,  IMU_I2C_SPEED,  IMU_I2C_ADDRESS, 0},
    {I2C0,  MAG_I2C_SPEED,  MAG_I2C_ADDRESS, 0},
    {I2C1, BARO_I2C_SPEED, BARO_I2C_ADDRESS, 0}
};


#define I2C_WRITE_TIME_OUT_MS 500
#define I2C_READ_TIME_OUT_MS 500

#define SELF_MASTER_ADDR  0x0d


extern U32 GetTickCountMs(void);

static void s_I2CGpioInit(I2C_PORT ePort)
{
    static int s_iGpioInitFlag = 0;
    switch (ePort)
    {
    case I2C_PORT0:
        if (s_iGpioInitFlag&0x01)
        {
            return ;
        }
        rcu_periph_clock_enable(RCU_GPIOB);      // Enable SDA0/SCL0 peripheral clock device
        rcu_periph_clock_enable(RCU_I2C0);       // enable I2C0 clock
        gpio_init(IIC0_SCL_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, IIC0_SCL_GPIO_PIN); // connect PB6 to I2C0_SCL
        gpio_init(IIC0_SDA_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, IIC0_SDA_GPIO_PIN);// connect PB7 to I2C0_SDA 
        gpio_bit_set(IIC0_SCL_GPIO_PORT, IIC0_SCL_GPIO_PIN);
        gpio_bit_set(IIC0_SDA_GPIO_PORT, IIC0_SDA_GPIO_PIN);
        s_iGpioInitFlag |= 0x01;
        return;

    case I2C_PORT1:
        if (s_iGpioInitFlag&0x02)
        {
            return ;
        }
        rcu_periph_clock_enable(RCU_GPIOB);     // Enable SDA1/SCL1 peripheral clock device
        rcu_periph_clock_enable(RCU_I2C1);      // Enable I2C1 clock
        gpio_init(IIC1_SCL_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, IIC1_SCL_GPIO_PIN); // connect PB10 to I2C1_SCL
        gpio_init(IIC1_SDA_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, IIC1_SDA_GPIO_PIN); // connect PB11 to I2C1_SDA
        gpio_bit_set(IIC1_SCL_GPIO_PORT, IIC1_SCL_GPIO_PIN);
        gpio_bit_set(IIC1_SDA_GPIO_PORT, IIC1_SDA_GPIO_PIN);
        s_iGpioInitFlag |= 0x02;
        return;
    default:
        return;
    }
}


// ePort:port number,  byTargetDeviceAddr:7bits I2C addr, dwDeviceSpeed:I2C speed
I2cBaseDefine *I2cOpen(I2C_DEVICE eDevice)
{
    static int s_iPort0OpenFlag = 0, s_iPort1OpenFlag = 0;

    switch (eDevice)
    {
    case MAG_I2C_IDX:
        if (0 == s_iPort0OpenFlag)
        {
            s_I2CGpioInit(I2C_PORT0);
            i2c_clock_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, gs_aoI2cHandle[eDevice].m_dwDeviceSpeed, I2C_DTCY_2); // configure I2C clock
            i2c_mode_addr_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SELF_MASTER_ADDR); // configure I2C address
            i2c_enable(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr);  // enable I2C
            i2c_ack_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, I2C_ACK_ENABLE); // enable acknowledge
            s_iPort1OpenFlag = 1;
        }
        gs_aoI2cHandle[eDevice].m_byDeviceUseFlag = 1;
        return &gs_aoI2cHandle[eDevice];
    case IMU_I2C_IDX:
    case BARO_I2C_IDX:
        if (0 == s_iPort1OpenFlag)
        {
            s_I2CGpioInit(I2C_PORT1);
            i2c_clock_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, gs_aoI2cHandle[eDevice].m_dwDeviceSpeed, I2C_DTCY_2); // configure I2C clock
            i2c_mode_addr_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SELF_MASTER_ADDR); // configure I2C address
            i2c_enable(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr);  // enable I2C
            i2c_ack_config(gs_aoI2cHandle[eDevice].m_dwRegBaseAddr, I2C_ACK_ENABLE); // enable acknowledge
            s_iPort1OpenFlag = 1;
        }
        gs_aoI2cHandle[eDevice].m_byDeviceUseFlag = 1;
        return &gs_aoI2cHandle[eDevice];
    default:
        return NULL;
    }
}


void s_Delay_I2C(int iTime)
{
    while (0!=iTime) {iTime--;}
}

void s_Resume_IIC(U32 Timeout,U32 I2Cx )
{
    U32 GPIO_SDA;
    U32 GPIO_SCL;
    U32 GPIO_Pin_SDA, GPIO_Pin_SCL;

    if (I2Cx == I2C0)
    {
        /* enable GPIOB clock */
        rcu_periph_clock_enable(RCU_GPIOB);

        /* enable BOARD_I2C APB1 clock */
        rcu_periph_clock_disable(RCU_I2C0);

        GPIO_SCL = GPIOB;
        GPIO_Pin_SCL = GPIO_PIN_6;
        GPIO_SDA = GPIOB;
        GPIO_Pin_SDA = GPIO_PIN_7;
    }
    else
    {
        /* enable GPIOB clock */
        rcu_periph_clock_enable(RCU_GPIOB);

        /* enable BOARD_I2C APB1 clock */
        rcu_periph_clock_disable(RCU_I2C1);

        GPIO_SCL = GPIOB;
        GPIO_Pin_SCL = GPIO_PIN_10;
        GPIO_SDA = GPIOB;
        GPIO_Pin_SDA = GPIO_PIN_11;
    }

    do {
        gpio_init(GPIO_SCL, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_Pin_SCL);
        gpio_init(GPIO_SDA, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_Pin_SDA);
        gpio_bit_reset(GPIO_SCL, GPIO_Pin_SCL);
        s_Delay_I2C(20);
        gpio_bit_reset(GPIO_SDA, GPIO_Pin_SDA);
        s_Delay_I2C(20);
        gpio_bit_set(GPIO_SCL, GPIO_Pin_SCL);
        s_Delay_I2C(20);
        gpio_bit_set(GPIO_SDA, GPIO_Pin_SDA);
        s_Delay_I2C(20);

        if (Timeout-- == 0)
        {
            return;
        }

    } while ((!gpio_input_bit_get(GPIO_SDA, GPIO_Pin_SDA))&(!gpio_input_bit_get(GPIO_SCL, GPIO_Pin_SCL)));
}

void s_I2cReset(I2cBaseDefine *poHandle)
{
    if (NULL == poHandle)
    {
        return;
    }
    if ((I2C0 == poHandle->m_dwRegBaseAddr) && (I2C1 == poHandle->m_dwRegBaseAddr))
    {
        s_Resume_IIC(10, poHandle->m_dwRegBaseAddr);
        i2c_clock_config(poHandle->m_dwRegBaseAddr, poHandle->m_dwDeviceSpeed, I2C_DTCY_2); // configure I2C clock
        i2c_mode_addr_config(poHandle->m_dwRegBaseAddr, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SELF_MASTER_ADDR); // configure I2C address
        i2c_enable(poHandle->m_dwRegBaseAddr);  // enable I2C
        i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_ENABLE); // enable acknowledge
    }
}


void I2cClose(I2cBaseDefine *poHandle)
{
    if (NULL == poHandle)
    {
        return ;
    }
    poHandle->m_byDeviceUseFlag = 0;
}


// 0:success, <0:failed
int I2cWriteBytes(I2cBaseDefine *poHandle, U8 *pbyBuffer, U8 byBufferLen, U8 byWriteOffset)
{
    U32 dwStart, dwCur;
    if (NULL == poHandle)
    {
        return -1;
    }
    dwCur = dwStart = GetTickCountMs();
    /* wait until I2C bus is idle */
    while (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_I2CBSY))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -2;
        }
        dwCur = GetTickCountMs();
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);
    dwCur = dwStart = GetTickCountMs();
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -3;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_TRANSMITTER);

    dwCur = dwStart = GetTickCountMs();

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -4;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    dwCur = dwStart = GetTickCountMs();

    /* wait until the transmit data buffer is empty */
    while (SET != i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_TBE))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -5;
        }
        dwCur = GetTickCountMs();
    }

    /* send the EEPROM's internal address to write to : only one byte address */
    i2c_data_transmit(poHandle->m_dwRegBaseAddr, byWriteOffset);

    dwCur = dwStart = GetTickCountMs();

    /* wait until BTC bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -6;
        }
        dwCur = GetTickCountMs();
    }

    /* while there is data to be written */
    while (byBufferLen--)
    {
        i2c_data_transmit(poHandle->m_dwRegBaseAddr, *pbyBuffer);

        /* point to the next byte to be written */
        pbyBuffer++;
        
        dwCur = dwStart = GetTickCountMs();

        /* wait until BTC bit is set */
        while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
        {
            if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
            {
                return -7;
            }
            dwCur = GetTickCountMs();
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until the stop condition is finished */
    while (I2C_CTL0(poHandle->m_dwRegBaseAddr) & I2C_CTL0_STOP)
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -8;
        }
        dwCur = GetTickCountMs();
    }
    return 0;

}

// 0:success, <0:failed
int I2cWriteByte(I2cBaseDefine *poHandle, U8 byOffsetAddr, U8 byData)
{
    U32 dwStart, dwCur;
    if (NULL == poHandle)
    {
        return -1;
    }
    dwCur = dwStart = GetTickCountMs();
    /* wait until I2C bus is idle */
    while (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_I2CBSY))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -2;
        }
        dwCur = GetTickCountMs();
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -3;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_TRANSMITTER);

    dwCur = dwStart = GetTickCountMs();

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -4;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    dwCur = dwStart = GetTickCountMs();

    /* wait until the transmit data buffer is empty */
    while (SET != i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_TBE))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -5;
        }
        dwCur = GetTickCountMs();
    }

    /* send the EEPROM's internal address to write to : only one byte address */
    i2c_data_transmit(poHandle->m_dwRegBaseAddr, byOffsetAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until BTC bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -6;
        }
        dwCur = GetTickCountMs();
    }

    /* while there is data to be written */
    i2c_data_transmit(poHandle->m_dwRegBaseAddr, byData);

    dwCur = dwStart = GetTickCountMs();

    /* wait until BTC bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -7;
        }
        dwCur = GetTickCountMs();
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until the stop condition is finished */
    while (I2C_CTL0(poHandle->m_dwRegBaseAddr) & I2C_CTL0_STOP)
    {
        if ((dwCur - dwStart) > I2C_WRITE_TIME_OUT_MS)
        {
            return -8;
        }
        dwCur = GetTickCountMs();
    }
    return 0;

}


int I2cReadBytes(I2cBaseDefine *poHandle, U8* pbyBuffer, U8 byReadOffset, U16 wWantReadBytes)
{
    U32 dwStart, dwCur, dwReadLen = 0;
    if (NULL == poHandle)
    {
        return -1;
    }
    dwCur = dwStart = GetTickCountMs();
    /* wait until I2C bus is idle */
    while (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_I2CBSY))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -2;
        }
        dwCur = GetTickCountMs();
    }

    if (2 == wWantReadBytes)
    {
        i2c_ackpos_config(poHandle->m_dwRegBaseAddr, I2C_ACKPOS_NEXT);
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -3;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_TRANSMITTER);

    dwCur = dwStart = GetTickCountMs();

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -4;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    dwCur = dwStart = GetTickCountMs();

    /* wait until the transmit data buffer is empty */
    while (SET != i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_TBE))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -5;
        }
        dwCur = GetTickCountMs();
    }

    /* enable I2C0*/
    i2c_enable(poHandle->m_dwRegBaseAddr);

    /* send the EEPROM's internal address to write to */
    i2c_data_transmit(poHandle->m_dwRegBaseAddr, byReadOffset);

    dwCur = dwStart = GetTickCountMs();

    /* wait until BTC bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -6;
        }
        dwCur = GetTickCountMs();
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);

    dwCur = dwStart = GetTickCountMs();

    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -7;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_RECEIVER);

    if (wWantReadBytes < 3)
    {
        /* disable acknowledge */
        i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_DISABLE);
    }

    dwCur = dwStart = GetTickCountMs();

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -8;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    if (1 == wWantReadBytes)
    {
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);
    }

    /* while there is data to be read */
    while (wWantReadBytes)
    {
        if (3 == wWantReadBytes)
        {
            dwCur = dwStart = GetTickCountMs();
            /* wait until BTC bit is set */
            while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
            {
                if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
                {
                    return -9;
                }
                dwCur = GetTickCountMs();
            }

            /* disable acknowledge */
            i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_DISABLE);
        }
        if (2 == wWantReadBytes) {
            dwCur = dwStart = GetTickCountMs();
            /* wait until BTC bit is set */
            while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
            {
                if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
                {
                    return -10;
                }
                dwCur = GetTickCountMs();
            }

            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);
        }

        /* wait until the RBNE bit is set and clear it */
        if (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_RBNE)) {
            /* read a byte from the EEPROM */
            *pbyBuffer = i2c_data_receive(poHandle->m_dwRegBaseAddr);

            /* point to the next location where the byte read will be saved */
            pbyBuffer++;

            /* decrement the read bytes counter */
            wWantReadBytes--;
            dwReadLen++;
        }
    }

    dwCur = dwStart = GetTickCountMs();

    /* wait until the stop condition is finished */
    while (I2C_CTL0(poHandle->m_dwRegBaseAddr) & I2C_CTL0_STOP)
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* enable acknowledge */
    i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_ENABLE);

    i2c_ackpos_config(poHandle->m_dwRegBaseAddr, I2C_ACKPOS_CURRENT);
    return dwReadLen;
}


int I2cWriteBytesAndRead(I2cBaseDefine *poHandle, U8 *pbyWriteBuff, U8 byWBytes, U8 *pbyReadBuff, U8 byRBytes)
{
    U32 dwStart, dwCur;
    int iReadLen;
    
    if (NULL == poHandle)
    {
        return -1;
    }
    dwCur = dwStart = GetTickCountMs();
    /* wait until I2C bus is idle */
    while (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_I2CBSY))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);

    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    /* while there is data to be written */
    while (byWBytes--)
    {
        i2c_data_transmit(poHandle->m_dwRegBaseAddr, *pbyWriteBuff);

        /* point to the next byte to be written */
        pbyWriteBuff++;

        /* wait until BTC bit is set */
        while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
        {
            if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
            {
                return -11;
            }
            dwCur = GetTickCountMs();
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);

    /* wait until the stop condition is finished */
    while (I2C_CTL0(poHandle->m_dwRegBaseAddr) & 0x0200)
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }


    /*******************************Read******************************************************************/

             /* wait until I2C bus is idle */
    while (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_I2CBSY))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    if (2 == byRBytes)
    {
        i2c_ackpos_config(poHandle->m_dwRegBaseAddr, I2C_ACKPOS_NEXT);
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(poHandle->m_dwRegBaseAddr);

    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_SBSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(poHandle->m_dwRegBaseAddr, poHandle->m_byDeviceAddr, I2C_RECEIVER);

    if (byRBytes < 3)
    {
        /* disable acknowledge */
        i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_DISABLE);
    }

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND))
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* clear the ADDSEND bit */
    i2c_flag_clear(poHandle->m_dwRegBaseAddr, I2C_FLAG_ADDSEND);

    if (1 == byRBytes)
    {
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);
    }

    iReadLen = 0;
    /* while there is data to be read */
    while (byRBytes)
    {
        if (3 == byRBytes)
        {
            /* wait until BTC bit is set */
            while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
            {
                if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
                {
                    return -11;
                }
                dwCur = GetTickCountMs();
            }

            /* disable acknowledge */
            i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_DISABLE);
        }
        if (2 == byRBytes)
        {
            /* wait until BTC bit is set */
            while (!i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_BTC))
            {
                if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
                {
                    return -11;
                }
                dwCur = GetTickCountMs();
            }

            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(poHandle->m_dwRegBaseAddr);
        }

        /* wait until the RBNE bit is set and clear it */
        if (i2c_flag_get(poHandle->m_dwRegBaseAddr, I2C_FLAG_RBNE))
        {
            /* read a byte from the EEPROM */
            *pbyReadBuff = i2c_data_receive(poHandle->m_dwRegBaseAddr);

            /* point to the next location where the byte read will be saved */
            pbyReadBuff++;

            /* decrement the read bytes counter */
            byRBytes--;
            iReadLen++;
        }
    }

    /* wait until the stop condition is finished */
    while (I2C_CTL0(poHandle->m_dwRegBaseAddr) & I2C_CTL0_STOP)
    {
        if ((dwCur - dwStart) > I2C_READ_TIME_OUT_MS)
        {
            return -11;
        }
        dwCur = GetTickCountMs();
    }

    /* enable acknowledge */
    i2c_ack_config(poHandle->m_dwRegBaseAddr, I2C_ACK_ENABLE);

    i2c_ackpos_config(poHandle->m_dwRegBaseAddr, I2C_ACKPOS_CURRENT);

    return iReadLen;

}












