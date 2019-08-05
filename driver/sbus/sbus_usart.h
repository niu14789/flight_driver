#ifndef  _SD_USART_H_
#define _SD_USART_H_



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

void SbusInit(void);

int SbusReceiveData(U16*pbyChnOut);



#endif



