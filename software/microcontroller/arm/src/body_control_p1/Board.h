/*----------------------------------------------------------------------------
*         ATMEL Microcontroller Software Support  -  ROUSSET  -
*----------------------------------------------------------------------------
* The software is delivered "AS IS" without warranty or condition of any
* kind, either express, implied or statutory. This includes without
* limitation any warranty or condition with respect to merchantability or
* fitness for any particular purpose, or against the infringements of
* intellectual property rights of others.
*----------------------------------------------------------------------------
* File Name           : Board.h
* Object              : AT91SAM7S Evaluation Board Features Definition File.
*
* Creation            : JPP   16/Jun/2004
*  1.1 14/Oct/05 JPP  : Change MCK
*----------------------------------------------------------------------------
*/
#ifndef Board_h
#define Board_h

#include "AT91SAM7S64.h"
//mtA:
// #define __inline inline
#define __inline static inline
//mtE
#include "lib_AT91SAM7S64.h"

//mtA
//#define __ramfunc __attribute__ ((long_call, section (".fastrun")))
#define __ramfunc
//mtE

#define true    -1
#define false   0

/*-------------------------------*/
/* SAM7Board Memories Definition */
/*-------------------------------*/
// The AT91SAM7S64 embeds a 16-Kbyte SRAM bank, and 64 K-Byte Flash

#define  INT_SARM           0x00200000
#define  INT_SARM_REMAP     0x00000000

#define  INT_FLASH          0x00000000
#define  INT_FLASH_REMAP    0x01000000

#define  FLASH_PAGE_NB      512
#define  FLASH_PAGE_SIZE    128

/*--------------*/
/* Master Clock */
/*--------------*/

#define EXT_OC          18432000   // External oscillator MAINCK
#define MCK             48054857   // MCK (PLLRC divided by 2)
#define MCKKHz          (MCK/1000) //

#endif /* Board_h */
