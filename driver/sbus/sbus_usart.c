/*********************************************************************************************************
*
*	模块名称 : 串口以及subs模块
*	文件名称 : sbus_usart.c
*	版    本 : V1.0
*	说    明 : 串口以及subs模块
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2019-04-17 Ryan Huang    无
*
*
**********************************************************************************************************/


#include "sbus_usart.h"
#include "serialport.h"
#include "systick.h"


SerialPortHandle *g_pSbusPort;  // 映射到物理端口UART3

/******************************************************************************
Function Name : void UART4_IRQHandler(void)
Description   : the UART4 interrupt proccessing function
Parameter     : none
Return        : none
History :
Author              date                   version
Ryan Huang         2019-05-14               V1.0
******************************************************************************/
void SbusInit(void)
{
    g_pSbusPort = UsartOpen(USART_PORT1);
}

extern void MyMemCpy(U8 *pbyDst, const U8*pbySrc, int iLen);

/******************************************************************************
Function Name : int SbusStateMachinePushOneByte(U8 byTemp, U8*pbyDataOut)
Description   : parse sbus receive state machine
Parameter     : 1. U8 byTemp[in]:data push the state machine
                      2. U8*pbyDataOut[out]: if there has packet, it will be save in this buffer and the buffer is 23 bytes.
Return        : 1:there has one valid sbus packet, 0: not valid packet
History :
Author              date                   version
Ryan Huang         2019-05-14               V1.0
******************************************************************************/
#define SBUS_PACKET_LEN 25
int SbusStateMachinePushOneByte(U8 byTemp, U8*pbyDataOut)
{
    #define SBUS_START_FLAG 0x0f
    #define SBUS_STOP_FLAG 0x00
    
    static U8 s_abyPacket[SBUS_PACKET_LEN];
    static int s_iState = 0;
    static int s_iOffset = 0;
    static U32 s_dwLastTimeMs = 0;
    U32 dwCurTimeMs;
    
    if (0 == s_dwLastTimeMs)
    {
        s_dwLastTimeMs = GetTickCountMs();
    }

    dwCurTimeMs = GetTickCountMs();
    if ((dwCurTimeMs - s_dwLastTimeMs) > 4)
    {
        s_iState = 0; // clear old data
    }
    s_dwLastTimeMs = dwCurTimeMs;

    switch (s_iState)
    {
    case 0:
        if (SBUS_START_FLAG == byTemp)
        {
            s_iState = 1;
            s_iOffset = 0;
        }
        return 0;
    case 1:
        s_abyPacket[s_iOffset] = byTemp;
        s_iOffset++;
        if ((SBUS_PACKET_LEN-2) == s_iOffset)
        {
            s_iState = 2;
        }
        return 0;
    case 2:
        s_iState = 0;
        if ((0x14 == byTemp) || (0x34 == byTemp) || (0x24 == byTemp) || (0x04 == byTemp))
        {
            MyMemCpy(pbyDataOut, s_abyPacket, SBUS_PACKET_LEN-2);
            return 1;
        }
        return 0;
    default:
        return 0;
    }
}


/******************************************************************************
Function Name : int SbusReceiveData(U16*pbyChnOut)
Description   : check and receive the sbus data, call less 7ms
Parameter     : 1.U16*pbyChnOut[out]:the sbus channel data
Return        : 1:there has one valid sbus packet, 0: not valid packet.  If packet valid the output is the sbus data.
History :
Author              date                   version
Ryan Huang         2019-05-14               V1.0
******************************************************************************/
int SbusReceiveData(U16*pbyChnOut)
{
    
    int iRet = 0, i, iReceiveLen, iFlag = 0;
    U8 abyTemp[200], abySbusData[32];

    iRet = UsartGetRecvBufferlen(g_pSbusPort);
    if (iRet > 0)
    {
        iReceiveLen = UsartRecvNoBlocking(g_pSbusPort, abyTemp, sizeof(abyTemp));
        if (iReceiveLen > 0)
        {
            for (i = 0; i < iReceiveLen; i++)
            {
                iRet = SbusStateMachinePushOneByte(abyTemp[i], abySbusData);
                if (1 == iRet)
                {
                    pbyChnOut[0]  = (U16)((abySbusData[0] | abySbusData[1] << 8) & 0x07FF);
                    pbyChnOut[1]  = (U16)(((abySbusData[1] >> 3) | (abySbusData[2] << 5)) & 0x07FF);
                    pbyChnOut[2]  = (U16)(((abySbusData[2] >> 6) | (abySbusData[3] << 2) | (abySbusData[4] << 10)) & 0x07FF);
                    pbyChnOut[3]  = (U16)(((abySbusData[4] >> 1) | (abySbusData[5] << 7)) & 0x07FF);
                    pbyChnOut[4]  = (U16)(((abySbusData[5] >> 4) | (abySbusData[6] << 4)) & 0x07FF);
                    pbyChnOut[5]  = (U16)(((abySbusData[6] >> 7) | (abySbusData[7] << 1) | (abySbusData[8] << 9)) & 0x07FF);
                    pbyChnOut[6]  = (U16)(((abySbusData[8] >> 2) | (abySbusData[9] << 6)) & 0x07FF);
                    pbyChnOut[7]  = (U16)(((abySbusData[9] >> 5) | (abySbusData[10] << 3)) & 0x07FF); // & the other 8 + 2 channels if you need them
                    pbyChnOut[8]  = (U16)((abySbusData[11] | (abySbusData[12] << 8))  & 0x07FF);
                    pbyChnOut[9]  = (U16)(((abySbusData[12] >> 3) | (abySbusData[13] << 5)) & 0x07FF);
                    pbyChnOut[10] = (U16)(((abySbusData[13] >> 6) | (abySbusData[14] << 2) | (abySbusData[15] << 10)) & 0x07FF);
                    pbyChnOut[11] = (U16)(((abySbusData[15] >> 1) | (abySbusData[16] << 7)) & 0x07FF);
                    pbyChnOut[12] = (U16)(((abySbusData[16] >> 4) | (abySbusData[17] << 4)) & 0x07FF);
                    pbyChnOut[13] = (U16)(((abySbusData[17] >> 7) | (abySbusData[18] << 1) | (abySbusData[19] << 9)) & 0x07FF);
                    pbyChnOut[14] = (U16)(((abySbusData[19] >> 2) | (abySbusData[20] << 6)) & 0x07FF);
                    pbyChnOut[15] = (U16)(((abySbusData[20] >> 5) | (abySbusData[21] << 3)) & 0x07FF);
                    iFlag = 1;
                }
            }
        }
    }
    return iFlag;
}


