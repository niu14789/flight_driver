
#include "SerialPort.h"
#include "BaseDefine.h"
#include "gd32f30x.h"


#define ENABLE_SBUS


// for usart0 with DMA0 RX/CH4 & TX/CH3
U8 g_abyUsart0DmaReceiveBuffer[1536]; // For GPS
U8 g_abyUsart0DmaSendBuff[256];
U8 g_abyUsart0ReceiveBuffer[2048]; // For GPS
SerialPortHandle g_oUsart0 = 
    {{g_abyUsart0ReceiveBuffer, sizeof(g_abyUsart0ReceiveBuffer), 0, 0, 9600, USART_WL_8BIT, USART_PM_NONE, USART_STB_1BIT, USART0},
    {g_abyUsart0DmaSendBuff, sizeof(g_abyUsart0DmaSendBuff), 0, 0, SEND_FINISHED, g_abyUsart0DmaReceiveBuffer, sizeof(g_abyUsart0DmaReceiveBuffer),DMA0, DMA_CH4, DMA_CH3}};

// for uart3 with DMA1 RX/CH2 & TX/CH4
U8 g_abyUart3DmaReceiveBuffer[100]; // For sbus
U8 g_abyUart3ReceiveBuffer[200]; // For sbus
SerialPortHandle g_oUart3 = 
    {{g_abyUart3ReceiveBuffer, sizeof(g_abyUart3ReceiveBuffer), 0, 0, 100000, USART_WL_8BIT, USART_PM_EVEN, USART_STB_2BIT, UART3},
    {NULL, 0, 0, 0, SEND_FINISHED, g_abyUart3DmaReceiveBuffer, sizeof(g_abyUart3DmaReceiveBuffer),DMA1, DMA_CH2, DMA_CH4}};


// for uart4 not used DMA

//#ifdef SUPPORT_UART4
SerialPortBaseDefine g_oPort2;  // 映射到物理端口UART4
//#endif



void MyMemCpy(U8 *pbyDst, const U8*pbySrc, int iLen)
{
    int i;

    for (i = 0; i < iLen; i++)
    {
        pbyDst[i] = pbySrc[i];
    }
}


/******************************************************************************
Function Name : void USART0_IRQHandler(void)
Description   : the USART0 interrupt proccessing function
Parameter     : none
Return        : none
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
void USART0_IRQHandler(void)
{
    U32 dwDmaBufferRemain, i, dwDmaReceiveNum;
    
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
    {
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
        i = USART_STAT0(USART0);
        i = USART_DATA(USART0);
        
        dma_channel_disable(DMA0, DMA_CH4);
        dwDmaBufferRemain = dma_transfer_number_get(DMA0, DMA_CH4);//获取DMA剩余缓存空间大小
        if (dwDmaBufferRemain != g_oUsart0.oDma.m_dwRecvBufferSize)
        {
            dwDmaReceiveNum = g_oUsart0.oDma.m_dwRecvBufferSize - dwDmaBufferRemain;
            for (i = 0; i < dwDmaReceiveNum; i++)
            {
                // carry dma receive buffer data to usart receive buffer
                g_oUsart0.oUsartParam.m_pbyRecvBuffer[g_oUsart0.oUsartParam.m_dwRecvIndexTail] = g_oUsart0.oDma.m_pbyRecvBuffer[i];
                g_oUsart0.oUsartParam.m_dwRecvIndexTail++;
                g_oUsart0.oUsartParam.m_dwRecvIndexTail %= g_oUsart0.oUsartParam.m_dwRecvBufferSize;
            }
        }
        dma_transfer_number_config(DMA0, DMA_CH4, g_oUsart0.oDma.m_dwRecvBufferSize);
        dma_channel_enable(DMA0, DMA_CH4);
    }
}

/******************************************************************************
Function Name : void UART3_IRQHandler(void)
Description   : the UART3 interrupt proccessing function
Parameter     : none
Return        : none
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
void UART3_IRQHandler(void)
{
    U32 dwDmaBufferRemain, i, dwDmaReceiveNum;
    
    if (RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_IDLE))
    {
        usart_interrupt_flag_clear(UART3, USART_INT_FLAG_IDLE);
        i = USART_STAT0(UART3);
        i = USART_DATA(UART3);
        
        dma_channel_disable(DMA1, DMA_CH2);
        dwDmaBufferRemain = dma_transfer_number_get(DMA1, DMA_CH2);//获取DMA剩余缓存空间大小
        if (dwDmaBufferRemain != g_oUart3.oDma.m_dwRecvBufferSize)
        {
            dwDmaReceiveNum = g_oUart3.oDma.m_dwRecvBufferSize - dwDmaBufferRemain;
            for (i = 0; i < dwDmaReceiveNum; i++)
            {
                // carry dma receive buffer data to usart receive buffer
                g_oUart3.oUsartParam.m_pbyRecvBuffer[g_oUart3.oUsartParam.m_dwRecvIndexTail] = g_oUart3.oDma.m_pbyRecvBuffer[i];
                g_oUart3.oUsartParam.m_dwRecvIndexTail++;
                g_oUart3.oUsartParam.m_dwRecvIndexTail %= g_oUart3.oUsartParam.m_dwRecvBufferSize;
            }
        }
        dma_transfer_number_config(DMA1, DMA_CH2, g_oUart3.oDma.m_dwRecvBufferSize);
        dma_channel_enable(DMA1, DMA_CH2);
    }
}

/******************************************************************************
Function Name : void UART4_IRQHandler(void)
Description   : the UART4 interrupt proccessing function
Parameter     : none
Return        : none
History :
Author              date                   version
Ryan Huang         2019-03-28               V1.0
******************************************************************************/
void UART4_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE))
    {
        // read one byte from the receive data register 
        g_oPort2.m_abyRecvBuffer[g_oPort2.m_dwRecvTail++] = (U8)usart_data_receive(UART4);
        g_oPort2.m_dwRecvTail %= BUFFER_SIZE;
        if (g_oPort2.m_dwRecvTail == g_oPort2.m_dwRecvHead)
        {
            g_oPort2.m_dwRecvHead++;
            g_oPort2.m_dwRecvHead %= BUFFER_SIZE;
        }
    }

    if (RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_TBE))
    {
        // write one byte to the transmit data register 
        if (g_oPort2.m_dwSendHead != g_oPort2.m_dwSendTail)
        {
            usart_data_transmit(UART4, (U8)g_oPort2.m_abySendBuffer[g_oPort2.m_dwSendHead++]);
            g_oPort2.m_dwSendHead %= BUFFER_SIZE;
            if (g_oPort2.m_dwSendHead == g_oPort2.m_dwSendTail)
            {
                g_oPort2.m_dwSendHead = 0;
                g_oPort2.m_dwSendTail = 0;
                usart_interrupt_disable(UART4, USART_INT_TBE); // disable transmitter buffer empty interrupt
            }
        }
        else
        {
            usart_interrupt_disable(UART4, USART_INT_TBE); // disable transmitter buffer empty interrupt
        }
    }
}


/******************************************************************************
Function Name : void DMA0_Channel3_IRQHandler(void)
Description   : the DMA0 channel 3 interrupt for USART0 proccessing function
Parameter     : none
Return        : none
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
void DMA0_Channel3_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
    {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_G);
        dma_channel_disable(DMA0, DMA_CH3);
        g_oUsart0.oDma.m_dwSendFinishFlag = SEND_FINISHED;
    }
}


/******************************************************************************
Function Name : void DMA1_Channel3_4_IRQHandler(void)
Description   : the DMA1 channel 4 interrupt for UART3 proccessing function
Parameter     : none
Return        : none
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
void DMA1_Channel3_4_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA1, DMA_CH4, DMA_INT_FLAG_FTF))
    {
        dma_interrupt_flag_clear(DMA1, DMA_CH4, DMA_INT_FLAG_G);
        dma_channel_disable(DMA1, DMA_CH4);
        g_oUart3.oDma.m_dwSendFinishFlag = SEND_FINISHED;
    }
}

void UsartDmaInit(USART_PORT ePort)
{
    dma_parameter_struct oDmaInitParam;

    switch (ePort)
    {
    case USART_PORT0:
        nvic_irq_enable(DMA0_Channel3_IRQn, 0, 0);

        rcu_periph_clock_enable(RCU_DMA0);

        // Send setting
        dma_deinit(DMA0, DMA_CH3);
        oDmaInitParam.direction = DMA_MEMORY_TO_PERIPHERAL;
        oDmaInitParam.memory_addr = 0;
        oDmaInitParam.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        oDmaInitParam.memory_width = DMA_MEMORY_WIDTH_8BIT;
        oDmaInitParam.number = 0;
        oDmaInitParam.periph_addr = USART0_DR_ADDRESS;
        oDmaInitParam.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        oDmaInitParam.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
        oDmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init(DMA0, DMA_CH3, &oDmaInitParam);
        /* configure DMA mode */
        dma_circulation_disable(DMA0, DMA_CH3);
        dma_memory_to_memory_disable(DMA0, DMA_CH3);
        /* USART DMA enable for transmission */
        usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
        /* enable DMA transfer complete interrupt */
        dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
        /* enable DMA channel1 */
        dma_channel_disable(DMA0, DMA_CH3);
        
        // Receive setting
        dma_deinit(DMA0, DMA_CH4);
        oDmaInitParam.direction = DMA_PERIPHERAL_TO_MEMORY;
        oDmaInitParam.memory_addr = (U32)g_oUsart0.oDma.m_pbyRecvBuffer;
        oDmaInitParam.number = g_oUsart0.oDma.m_dwRecvBufferSize;
        oDmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init(DMA0, DMA_CH4, &oDmaInitParam);
        dma_circulation_enable(DMA0, DMA_CH4);
        dma_memory_to_memory_disable(DMA0, DMA_CH4);
        dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF);
        usart_dma_receive_config(USART0, USART_DENR_ENABLE);
        dma_channel_enable(DMA0, DMA_CH4);
        break;
        
    case USART_PORT1:        
        nvic_irq_enable(DMA1_Channel3_Channel4_IRQn, 0, 0);
        
        rcu_periph_clock_enable(RCU_DMA1); // add by Filk in 2019-05-23
        
        dma_deinit(DMA1, DMA_CH4);
        oDmaInitParam.direction = DMA_MEMORY_TO_PERIPHERAL;
        oDmaInitParam.memory_addr = 0;
        oDmaInitParam.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        oDmaInitParam.memory_width = DMA_MEMORY_WIDTH_8BIT;
        oDmaInitParam.number = 0;
        oDmaInitParam.periph_addr = UART3_DR_ADDRESS;
        oDmaInitParam.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        oDmaInitParam.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
        oDmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init(DMA1, DMA_CH4, &oDmaInitParam);
        /* configure DMA mode */
        dma_circulation_disable(DMA1, DMA_CH4);
        dma_memory_to_memory_disable(DMA1, DMA_CH4);
        /* USART DMA enable for transmission */
        usart_dma_transmit_config(UART3, USART_DENT_ENABLE);
        /* enable DMA transfer complete interrupt */
        dma_interrupt_enable(DMA1, DMA_CH4, DMA_INT_FTF);
        /* enable DMA channel1 */
        dma_channel_disable(DMA1, DMA_CH4);

        dma_deinit(DMA1, DMA_CH2);
        oDmaInitParam.direction = DMA_PERIPHERAL_TO_MEMORY;
        oDmaInitParam.memory_addr = (U32)g_oUart3.oDma.m_pbyRecvBuffer;
        oDmaInitParam.number = g_oUart3.oDma.m_dwRecvBufferSize;
        oDmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init(DMA1, DMA_CH2, &oDmaInitParam);
        /* configure DMA mode */
        dma_circulation_disable(DMA1, DMA_CH2);
        dma_memory_to_memory_disable(DMA1, DMA_CH2);
        usart_dma_receive_config(UART3, USART_DENR_ENABLE);
        dma_channel_enable(DMA1, DMA_CH2);
        break;
        
    case USART_PORT2:
        break;

    default:
        return ;
    }
}

/******************************************************************************
Function Name : SerialPortBaseDefine *UsartOpen(USART_PORT ePort, U32 dwBauds)
Description   : Open the usart port with "bauds,8,n,1"
Parameter     : 1.ePort[in]:usart port
                2.dwMode[in]: 0:normal serial port mode, 1:sbus mode
                3.dwBauds[in]: the serial port bauds
Return        : return the usart device handle pointer if success and return null pointer if failed.
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
SerialPortHandle *UsartOpen(USART_PORT ePort)
{
    switch (ePort)
    {
    case USART_PORT0:
        
        // Port0 ----> Usart0 Rx:PA10 / Tx:PA9
        // USART interrupt configuration
        nvic_irq_enable(USART0_IRQn, 0, 1);


        // enable GPIO clock
        rcu_periph_clock_enable(RCU_GPIOA);

        // enable USART clock 
        rcu_periph_clock_enable(RCU_USART0);

        // connect port to USARTx_Tx
        gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

        // connect port to USARTx_Rx
        gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
        
        // UART configure
        usart_deinit(USART0); // reset uart/usart
        
        usart_baudrate_set(USART0, g_oUsart0.oUsartParam.m_dwBauds);
        usart_word_length_set(USART0, g_oUsart0.oUsartParam.m_dwDataBits);
        usart_parity_config(USART0, g_oUsart0.oUsartParam.m_dwParity);
        usart_stop_bit_set(USART0, g_oUsart0.oUsartParam.m_dwStopBits);
        
        usart_receive_config(USART0, USART_RECEIVE_ENABLE);     // enable receive
        usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);   // enable send 

        // enable uart/usart receive idle interrupt 
        usart_interrupt_enable(USART0, USART_INT_IDLE); // there has not sent idle interrupt, but has sent finished interrupt.

        usart_enable(USART0); // enable uart/usart
        
        // Initial DMA here...
        UsartDmaInit(ePort);
        
        return &g_oUsart0;

    case USART_PORT1:
        // Port1 ----> Uart3 Rx:PC11 / Tx:PC10
        // UART3 interrupt configuration
        nvic_irq_enable(UART3_IRQn, 1, 1);


        // enable GPIO clock
        rcu_periph_clock_enable(RCU_GPIOC);

        // enable UART clock 
        rcu_periph_clock_enable(RCU_UART3);

        // connect port to UART3_Tx
        gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

        // connect port to UART3_Rx
        gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
        
        // UART configure
        usart_deinit(UART3); // reset uart/usart
        
        usart_baudrate_set(UART3, g_oUart3.oUsartParam.m_dwBauds);
        usart_word_length_set(UART3, g_oUart3.oUsartParam.m_dwDataBits);
        usart_parity_config(UART3, g_oUart3.oUsartParam.m_dwParity);
        usart_stop_bit_set(UART3, g_oUart3.oUsartParam.m_dwStopBits);
        
        usart_receive_config(UART3, USART_RECEIVE_ENABLE);     // enable receive
        usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);   // enable send 
        usart_enable(UART3); // enable uart/usart

        // Initial DMA here...
        UsartDmaInit(ePort);
        
        // enable uart/usart receive idle interrupt 
        usart_interrupt_enable(UART3, USART_INT_IDLE); // there has not sent idle interrupt, but has sent finished interrupt.
        return &g_oUart3;
    default:
        return NULL;
    }
}

SerialPortBaseDefine *Uart4Open(U32 dwBauds)
{
    SerialPortBaseDefine *psBase;

    
    // Port2 ----> Uart4 Rx:PD2 / Tx:PC12
    // UART4 interrupt configuration
    nvic_irq_enable(UART4_IRQn, 1, 1);
    
    // enable GPIO clock
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    
    // enable UART clock 
    rcu_periph_clock_enable(RCU_UART4);
    
    // connect port to UART4_Tx
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_12);
    
    // connect port to UART4_Rx
    gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ, GPIO_PIN_2);
    

    psBase = &g_oPort2;
    psBase->m_dwAddr = UART4;

    psBase->m_dwRecvHead = 0;
    psBase->m_dwRecvTail = 0;
    psBase->m_dwSendHead = 0;
    psBase->m_dwSendTail = 0;

    // UART configure
    usart_deinit(psBase->m_dwAddr); // reset uart/usart
    usart_baudrate_set(psBase->m_dwAddr, dwBauds);  // set bauds
    usart_word_length_set(psBase->m_dwAddr, USART_WL_8BIT);  // send & receive word length is 8 bits
    usart_parity_config(psBase->m_dwAddr, USART_PM_NONE);  // not parity
    usart_stop_bit_set(psBase->m_dwAddr, USART_STB_1BIT);  // one stop bit
    usart_receive_config(psBase->m_dwAddr, USART_RECEIVE_ENABLE);     // enable receive
    usart_transmit_config(psBase->m_dwAddr, USART_TRANSMIT_ENABLE);   // enable send 
    usart_enable(psBase->m_dwAddr); // enable uart/usart

    // enable uart/usart receive interrupt 
    usart_interrupt_enable(psBase->m_dwAddr, USART_INT_RBNE); // start to response the read buffer not empty interrupt and overrun error

    return psBase;
}

void UsartChangeBauds(const SerialPortHandle *pPort, U32 dwBauds)
{
    usart_baudrate_set(pPort->oUsartParam.m_dwAddr, dwBauds);  // set bauds
}

/******************************************************************************
Function Name :void UsartClose(const SerialPortHandle *pPort)
Description   : close the usart and disable the usart interrupt
Parameter     : 1.ePort[in]:usart port
Return        : none
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
void UsartClose(const SerialPortHandle *pPort)
{
    if (NULL != pPort)
    {        
        // uart/usart configure
        usart_receive_config(pPort->oUsartParam.m_dwAddr, USART_RECEIVE_DISABLE);
        usart_transmit_config(pPort->oUsartParam.m_dwAddr, USART_TRANSMIT_DISABLE);
        usart_disable(pPort->oUsartParam.m_dwAddr);

        switch (pPort->oUsartParam.m_dwAddr)
        {
        case USART0:
            usart_interrupt_disable(pPort->oUsartParam.m_dwAddr, USART_INT_IDLE); // disable uart/usart receive idle interrupt        
            usart_deinit(pPort->oUsartParam.m_dwAddr); // reset uart/usart
            dma_deinit(DMA0, DMA_CH4);
            dma_deinit(DMA0, DMA_CH3);
            break;
            
        case UART3:
            usart_interrupt_disable(pPort->oUsartParam.m_dwAddr, USART_INT_IDLE); // disable uart/usart receive idle interrupt        
            usart_deinit(pPort->oUsartParam.m_dwAddr); // reset uart/usart
            dma_deinit(DMA1, DMA_CH2);
            dma_deinit(DMA1, DMA_CH4);
            break;
        }
    }
}

void Uart4Close(const SerialPortBaseDefine *pPort)
{
    if (NULL != pPort)
    {
        // uart/usart configure
        usart_receive_config(pPort->m_dwAddr, USART_RECEIVE_DISABLE);
        usart_transmit_config(pPort->m_dwAddr, USART_TRANSMIT_DISABLE);
        usart_disable(pPort->m_dwAddr);

        // enable uart/usart receive interrupt
        usart_interrupt_disable(pPort->m_dwAddr, USART_INT_RBNE);

        // enable uart/usart transmit interrupt
        usart_interrupt_disable(pPort->m_dwAddr, USART_INT_TBE);
    }
}


/******************************************************************************
Function Name : int UsartGetRecvBufferlen(SerialPortHandle *pPort)
Description   : receive/read data from usart
Parameter     : 1.pPort[in]:usart device handle
Return        : USART_ERR_PORT_NUM : usart handle is null
                 >=0 : the bytes of the data has been read
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int UsartGetRecvBufferlen(const SerialPortHandle *pPort)
{
    int iBufferLen;
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        __set_PRIMASK(1); /* 关中断 */
        if (pPort->oUsartParam.m_dwRecvIndexTail >= pPort->oUsartParam.m_dwRecvIndexHead)
        {
            iBufferLen = pPort->oUsartParam.m_dwRecvIndexTail - pPort->oUsartParam.m_dwRecvIndexHead;
        }
        else
        {
            iBufferLen = (pPort->oUsartParam.m_dwRecvIndexTail + pPort->oUsartParam.m_dwRecvBufferSize) - pPort->oUsartParam.m_dwRecvIndexHead;
        }
        __set_PRIMASK(0); /* 开中断 */
        return iBufferLen;
    }
}

/******************************************************************************
Function Name : int UsartGetRecvBufferlen(SerialPortBaseDefine *pPort)
Description   : receive/read data from usart
Parameter     : 1.pPort[in]:usart device handle
Return        : USART_ERR_PORT_NUM : usart handle is null
                 >=0 : the bytes of the data has been read
History :
Author              date                   version
Ryan Huang         2019-03-28               V1.0
******************************************************************************/
int Uart4GetRecvBufferlen(const SerialPortBaseDefine *pPort)
{
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        if (pPort->m_dwRecvTail >= pPort->m_dwRecvHead)
        {
            return (pPort->m_dwRecvTail - pPort->m_dwRecvHead);
        }
        else
        {
            return ((pPort->m_dwRecvTail + BUFFER_SIZE) - pPort->m_dwRecvHead);
        }
    }
}

/******************************************************************************
Function Name : int UsartGetSendBufferlen(SerialPortBaseDefine *pPort)
Description   : send/write data to usart
Parameter     : 1.pPort[in]:usart device handle
Return        : USART_ERR_PORT_NUM : usart handle is null
                >=0 : the bytes of the data has not sent yet
History :
Author              date                   version
Ryan Huang         2019-03-28               V1.0
******************************************************************************/
int Uart4GetSendBufferlen(const SerialPortBaseDefine *pPort)
{
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        if (pPort->m_dwSendTail >= pPort->m_dwSendHead)
        {
            return (pPort->m_dwSendTail - pPort->m_dwSendHead);
        }
        else
        {
            return ((pPort->m_dwSendTail + BUFFER_SIZE) - pPort->m_dwSendHead);
        }
    }
}

/******************************************************************************
Function Name : int UsartGetSendBufferlen(SerialPortBaseDefine *pPort)
Description   : send/write data to usart
Parameter     : 1.pPort[in]:usart device handle
Return        : USART_ERR_PORT_NUM : usart handle is null
                >=0 : the bytes of the data has not sent yet
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int UsartGetSendStatus(const SerialPortHandle *pPort)
{
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        return pPort->oDma.m_dwSendFinishFlag;
    }
}

/******************************************************************************
Function Name : int UsartRecvNoBlocking(SerialPortBaseDefine *pPort, U8 *pbyDataOut, int iWantToRead)
Description   : receive/read data from usart
Parameter     : 1.pPort[in/out]:usart device handle
                2.pbyDataOut[out]:save the receive data
                3.iWantToRead[in]:number of bytes want to read
Return        : USART_ERR_PORT_NUM : usart handle is null
                >=0 : the bytes of the data that has read
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int UsartRecvNoBlocking(SerialPortHandle *pPort, U8 *pbyDataOut, int iWantToRead)
{
    int iReadCount = 0;
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        __set_PRIMASK(1); /* 关中断 */
        while (iWantToRead > 0)
        {
            if (pPort->oUsartParam.m_dwRecvIndexHead == pPort->oUsartParam.m_dwRecvIndexTail)
            {
                pPort->oUsartParam.m_dwRecvIndexHead = 0;
                pPort->oUsartParam.m_dwRecvIndexTail = 0;
                break;
            }
            *pbyDataOut++ = pPort->oUsartParam.m_pbyRecvBuffer[pPort->oUsartParam.m_dwRecvIndexHead++];
            pPort->oUsartParam.m_dwRecvIndexHead %= pPort->oUsartParam.m_dwRecvBufferSize;
            iWantToRead--;
            iReadCount++;
        }
        __set_PRIMASK(0); /* 开中断 */
        return iReadCount;
    }
}


int UsartCleanReceiveBuffer(SerialPortHandle *pPort)
{
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        __set_PRIMASK(1); /* 关中断 */
        pPort->oUsartParam.m_dwRecvIndexHead = 0;
        pPort->oUsartParam.m_dwRecvIndexTail = 0;
        __set_PRIMASK(0); /* 开中断 */
        return 0;
    }
}

/******************************************************************************
Function Name : int UsartRecvNoBlocking(SerialPortBaseDefine *pPort, U8 *pbyDataOut, int iWantToRead)
Description   : receive/read data from usart
Parameter     : 1.pPort[in/out]:usart device handle
                2.pbyDataOut[out]:save the receive data
                3.iWantToRead[in]:number of bytes want to read
Return        : USART_ERR_PORT_NUM : usart handle is null
                >=0 : the bytes of the data that has read
History :
 Author              date                   version
Ryan Huang         2019-03-28               V1.0
******************************************************************************/
int Uart4RecvNoBlocking(SerialPortBaseDefine *pPort, U8 *pbyDataOut, int iWantToRead)
{
    int iReadCount = 0;
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        while (iWantToRead > 0)
        {
            if (pPort->m_dwRecvHead == pPort->m_dwRecvTail)
            {
                pPort->m_dwRecvHead = 0;
                pPort->m_dwRecvTail = 0;
                return iReadCount;
            }
            *pbyDataOut++ = pPort->m_abyRecvBuffer[pPort->m_dwRecvHead++];
            pPort->m_dwRecvHead %= BUFFER_SIZE;
            iWantToRead--;
            iReadCount++;
        }
        return iReadCount;
    }
}

/******************************************************************************
Function Name : int UsartSendNoBlocking(SerialPortBaseDefine *pPort, const U8 *pbyDataIn, int iWantToWrite)
Description   : send/write data to usart
Parameter     : 1.pPort[in/out]:usart device handle
                2.pbyDataIn[in]:save the data to be sent
                3.iWantToWrite[in]:number of bytes want to send
Return        : USART_ERR_PORT_NUM : usart handle is null
                 >=0 : the bytes of the data that want to send
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-22               V1.0
******************************************************************************/
int UsartSendNoBlocking(SerialPortHandle *pPort, const U8 *pbyDataIn, int iWantToWrite)
{
    int iWriteCount = 0;
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        pPort->oDma.m_dwSendFinishFlag = SEND_NOT_FINISHED;
//        dma_channel_disable(pPort->oDma.m_dwDmaNum, pPort->oDma.m_eDmaSendChn);
        DMA_CHMADDR(pPort->oDma.m_dwDmaNum, pPort->oDma.m_eDmaSendChn) = (U32)pbyDataIn;
        DMA_CHCNT(pPort->oDma.m_dwDmaNum, pPort->oDma.m_eDmaSendChn) = (iWantToWrite & DMA_CHANNEL_CNT_MASK);
        dma_channel_enable(pPort->oDma.m_dwDmaNum, pPort->oDma.m_eDmaSendChn);
        return iWriteCount;
    }
}

/******************************************************************************
Function Name : int UsartSendNoBlocking(SerialPortBaseDefine *pPort, const U8 *pbyDataIn, int iWantToWrite)
Description   : send/write data to usart
Parameter     : 1.pPort[in/out]:usart device handle
                2.pbyDataIn[in]:save the data to be sent
                3.iWantToWrite[in]:number of bytes want to send
Return        : USART_ERR_PORT_NUM : usart handle is null
                 >=0 : the bytes of the data that want to send
History :
Author              date                   version
Ryan Huang         2019-03-28               V1.0
******************************************************************************/
int Uart4SendNoBlocking(SerialPortBaseDefine *pPort, const U8 *pbyDataIn, int iWantToWrite)
{
    int iWriteCount = 0;
    if (NULL == pPort)
    {
        return USART_ERR_PORT_NUM;
    }
    else
    {
        U8 *pbyTemp = (U8*)pbyDataIn;
        while (iWantToWrite > 0)
        {
            pPort->m_abySendBuffer[pPort->m_dwSendTail++] = *pbyTemp++;
            pPort->m_dwSendTail %= BUFFER_SIZE;
            iWriteCount++;
            iWantToWrite--;
            if (pPort->m_dwSendHead == pPort->m_dwSendTail)
            {
                usart_interrupt_enable(pPort->m_dwAddr, USART_INT_TBE); // enable transmitter buffer empty interrupt
                return iWriteCount;
            }
        }
        usart_interrupt_enable(pPort->m_dwAddr, USART_INT_TBE); // enable transmitter buffer empty interrupt
        return iWriteCount;
    }
}

