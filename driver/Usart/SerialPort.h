
#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include "gd32f30x.h"


#define BUFFER_SIZE   2048

typedef enum _USART_PORT_{
    USART_PORT0 = 0,   // for GPS
    USART_PORT1,       // for WIFI(APP)
    USART_PORT2,       // for Optical flow
    USART_PORT_LAST
}USART_PORT;

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif

#define USART_OK              0
#define USART_ERR_PORT_NUM   -1


#define SEND_FINISHED       1
#define SEND_NOT_FINISHED   0


typedef struct _SerialPortParameter_ {
    U8 *m_pbyRecvBuffer;
    U32 m_dwRecvBufferSize;
    U32 m_dwRecvIndexHead;
    U32 m_dwRecvIndexTail;
    U32 m_dwBauds;
    U32 m_dwDataBits;
    U32 m_dwParity;
    U32 m_dwStopBits;
    U32 m_dwAddr;
}SerialPortParameter;

typedef struct _SerialPortDmaBuffer_ {
    U8 *m_pbySendBuffer;
    U32 m_dwSendBufferSize;
    U32 m_dwSendLength;
    U32 m_dwCurSendIndex;
    U32 m_dwSendFinishFlag;
    
    U8 *m_pbyRecvBuffer;
    U32 m_dwRecvBufferSize;
    U32 m_dwDmaNum;
    U32 m_dwDmaReceiveChn;
    dma_channel_enum m_eDmaSendChn;
}SerialPortDmaBuffer;

typedef struct _SerialPortHandle_ {
    SerialPortParameter oUsartParam;
    SerialPortDmaBuffer oDma;
}SerialPortHandle;


typedef struct _SerialPortBaseDefine_ {
    U8 m_abySendBuffer[BUFFER_SIZE];
    U8 m_abyRecvBuffer[BUFFER_SIZE];
    U32 m_dwSendHead;
    U32 m_dwSendTail;
    U32 m_dwRecvHead;
    U32 m_dwRecvTail;
    U32 m_dwAddr;
}SerialPortBaseDefine;

#define USART_REG_ADDR_OFFSET_STAT0  0x00
#define USART_REG_ADDR_OFFSET_DATA   0x04
#define USART_REG_ADDR_OFFSET_BAUD   0x08
#define USART_REG_ADDR_OFFSET_CTL0   0x0C
#define USART_REG_ADDR_OFFSET_CTL1   0x10
#define USART_REG_ADDR_OFFSET_CTL2   0x14
#define USART_REG_ADDR_OFFSET_GP     0x18
#define USART_REG_ADDR_OFFSET_CTL3   0x80
#define USART_REG_ADDR_OFFSET_RT     0x84
#define USART_REG_ADDR_OFFSET_STAT1  0x88

/* flags in STAT0 register */
#define USART_FLAG_BIT_CTSF  0x00000200   /*!< CTS change flag */
#define USART_FLAG_BIT_LBDF  0x00000100   /*!< LIN break detected flag */
#define USART_FLAG_BIT_TBE   0x00000080   /*!< transmit data buffer empty */
#define USART_FLAG_BIT_TC    0x00000040   /*!< transmission complete */
#define USART_FLAG_BIT_RNBE  0x00000020   /*!< read data buffer not empty */
#define USART_FLAG_BIT_IDLEF 0x00000010   /*!< IDLE frame detected flag */
#define USART_FLAG_BIT_ORERR 0x00000008   /*!< overrun error */
#define USART_FLAG_BIT_NERR  0x00000004   /*!< noise error flag */
#define USART_FLAG_BIT_FERR  0x00000002   /*!< frame error flag */
#define USART_FLAG_BIT_PERR  0x00000001   /*!< parity error flag */

  
/* flags in STAT1 register */
#define USART_FLAG_BIT_BSY  0x00010000   /*!< busy flag */
#define USART_FLAG_BIT_EBF  0x00001000   /*!< end of block flag */
#define USART_FLAG_BIT_RTF  0x00000800   /*!< receiver timeout flag */


#define USART0_DR_ADDRESS      0x40013804


#define UART3_DR_ADDRESS      0x40004c04//((uint32_t)0x40004804)//((uint32_t)0x40013804)





SerialPortHandle *UsartOpen(USART_PORT ePort);
SerialPortBaseDefine *Uart4Open(U32 dwBauds);
void UsartChangeBauds(const SerialPortHandle *pPort, U32 dwBauds);

int UsartGetRecvBufferlen(const SerialPortHandle *pPort);
int Uart4GetRecvBufferlen(const SerialPortBaseDefine *pPort);
int UsartCleanReceiveBuffer(SerialPortHandle *pPort);

int UsartGetSendStatus(const SerialPortHandle *pPort);

int Uart4GetSendBufferlen(const SerialPortBaseDefine *pPort);

int UsartRecvNoBlocking(SerialPortHandle *pPort, U8 *pbyDataOut, int iWantToRead);
int Uart4RecvNoBlocking(SerialPortBaseDefine *pPort, U8 *pbyDataOut, int iWantToRead);

int UsartSendNoBlocking(SerialPortHandle *pPort, const U8 *pbyDataIn, int iWantToWrite);
int Uart4SendNoBlocking(SerialPortBaseDefine *pPort, const U8 *pbyDataIn, int iWantToWrite);

void UsartClose(const SerialPortHandle *pPort);
void Uart4Close(const SerialPortBaseDefine *pPort);

#endif

