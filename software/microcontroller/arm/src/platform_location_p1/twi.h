/*-----------------------------------------------------------------------------
*      ATMEL Microcontroller Software Support  -  ROUSSET  -
*------------------------------------------------------------------------------
* The software is delivered "AS IS" without warranty or condition of any
* kind, either express, implied or statutory. This includes without
* limitation any warranty or condition with respect to merchantability or
* fitness for any particular purpose, or against the infringements of
* intellectual property rights of others.
*------------------------------------------------------------------------------
* File Name           : lib_twi.h
* Object
* Version | mm | dd | yy | author :
*   1.0     11   07   06    PFi   : Creation
*----------------------------------------------------------------------------*/
#ifndef _LIB_TWI_H_
#define _LIB_TWI_H_

#include "Board.h"
#include <math.h>

/* Global declarations */
#define TWI_BUS_CLOCK  (unsigned int)8000  //Hz

/* TWI Init functions */
extern void AT91F_TWI_Open(int TwiClock);

extern int AT91F_TWI_WriteSingle(const AT91PS_TWI pTwi, int SlaveAddr, char *data);

extern int AT91F_TWI_WriteSingleIadr(const AT91PS_TWI pTwi, int SlaveAddr, int IntAddr, int IntAddrSize, char *data);

extern int AT91F_TWI_WriteMultiple(const AT91PS_TWI pTwi, int SlaveAddr, char *data, unsigned int NumOfBytes);

extern int AT91F_TWI_WriteMultipleIadr(const AT91PS_TWI pTwi, int SlaveAddr, int IntAddr, int IntAddrSize, char *data, unsigned int NumOfBytes);

extern int AT91F_TWI_ReadSingle(const AT91PS_TWI pTwi,
                       int SlaveAddr,
                       char *data);
extern int AT91F_TWI_ReadSingleIadr(const AT91PS_TWI pTwi,
                       int SlaveAddr,
                       int IntAddr,
                       int IntAddrSize,
                       char *data);
extern int AT91F_TWI_ReadMultiple(const AT91PS_TWI pTwi,
                       int SlaveAddr,
                       unsigned int NumOfBytes,
                       char *data);
extern int AT91F_TWI_ReadMultipleIadr(const AT91PS_TWI pTwi,
                       int SlaveAddr,
                       unsigned int NumOfBytes,
                       int IntAddr,
                       int IntAddrSize,
                       char *data);
extern void AT91F_TWI_BusRecovery (const AT91PS_PIO pPio,
                            int PioID,
                            int Twck,
                            int Twd);
extern void AT91F_TWI_WaitMicroSecond (unsigned int MicroSecond);
extern int AT91F_TWI_ProbeDevices(const AT91PS_TWI pTwi, int SlaveAddr);

extern void AT91F_SetTwiClock(int TwiClock);

#endif

