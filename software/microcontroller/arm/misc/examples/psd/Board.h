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
* V 1.0 21/Feb/05 JPP : Define __ramfunc
* V 1.1 21/Feb/05 JPP : add Lib definition
* V 1.2 22/Feb/05 JPP : Add DBGU inline definition
*----------------------------------------------------------------------------
*/
#ifndef Board_h
#define Board_h

#include "AT91SAM7S64.h"
#define __inline static inline
#include "lib_AT91SAM7S64.h"

#define __ramfunc

#define AT91C_US_ASYNC_MODE ( AT91C_US_USMODE_NORMAL + AT91C_US_NBSTOP_1_BIT + AT91C_US_PAR_NONE + AT91C_US_CHRL_8_BITS + AT91C_US_CLKS_CLOCK )

#define true	-1
#define false	0

/*-------------------------------*/
/* SAM7Board Memories Definition */
/*-------------------------------*/
// The AT91SAM7S64 embeds a 16-Kbyte SRAM bank, and 64 K-Byte Flash

#define  INT_SARM           0x00200000
#define  INT_SARM_REMAP	    0x00000000

#define  INT_FLASH          0x00000000
#define  INT_FLASH_REMAP    0x01000000

#define  FLASH_PAGE_NB	    512
#define  FLASH_PAGE_LOCK    32
#define  FLASH_PAGE_SIZE    128

/*-----------------*/
/* Leds Definition */
/*-----------------*/
#define LED1            (1<<3)
#define LED2            (1<<4)
#define LED3            (1<<5)
/* on Board leds */
#define LED_YELLOW            (1<<17)
#define LED_GREEN            (1<<18)
/*-----------------*/
/* L298 Direction */
/*-----------------*/
#define DIRC            (1<<6)
#define DIRD            (1<<7)

#define OUTPUT_MASK        (LED1 | LED2 | LED3 | DIRC | DIRD | LED_GREEN | LED_YELLOW)

/*-------------------------*/
/* Push Buttons Definition */
/*-------------------------*/

#define SW1_MASK        (1<<19)
#define SW2_MASK        (1<<20)
#define SW_MASK         (SW1_MASK|SW2_MASK)


/*------------------*/
/* USART Definition */
/*------------------*/
/* SUB-D 9 points J3 DBGU*/
#define DBGU_RXD		AT91C_PA9_DRXD	  /* JP11 must be close */
#define DBGU_TXD		AT91C_PA10_DTXD	  /* JP12 must be close */
#define AT91C_DBGU_BAUD	   115200   // Baud rate

/*
#define US_RXD_PIN		AT91C_PA5_RXD0
#define US_TXD_PIN		AT91C_PA6_TXD0
#define US_RTS_PIN		AT91C_PA7_RTS0
#define US_CTS_PIN		AT91C_PA8_CTS0
*/

/*--------------*/
/* Master Clock */
/*--------------*/

#define EXT_OC          18432000   // Exetrnal ocilator MAINCK
#define MCK             47923200   // MCK (PLLRC div by 2)
#define MCKKHz          (MCK/1000) //

#endif /* Board_h */
