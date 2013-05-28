/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "../common/applet.h"
#include <board.h>
#include <board_lowlevel.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <dbgu/dbgu.h>
#include <utility/assert.h>
#include <utility/trace.h>

#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/* Initialization mode */
#define EK_MODE                 ((unsigned int) 0x00000000)
#define USER_DEFINED_CRYSTAL    ((unsigned int) 0x00000001)
#define BYPASS_MODE             ((unsigned int) 0x00000002)

/*----------------------------------------------------------------------------
 *        Local structures
 *----------------------------------------------------------------------------*/
/** 
 * \brief Structure for storing parameters for each command that can be performed by the applet.
 */
struct _Mailbox {

    /* Command send to the monitor to be executed. */
    unsigned int command;
    /* Returned status, updated at the end of the monitor execution. */
    unsigned int status;

    /* Input Arguments in the argument area */
    union {

        /* Input arguments for the Init command. */
        struct {

            /* Communication link used. */
            unsigned int comType;
            /* Trace level. */
            unsigned int traceLevel;
            /* Low initialization mode */
            unsigned int Mode;
            /* frequency of user-defined crystal */
            unsigned int crystalFreq;
            /* frequency of external clock in bypass mode */
            unsigned int extClk;

        } inputInit;

        /* Output arguments for the Init command. */
        /* None */

        /* Input arguments for the Write command. */
        /* None */

        /* Output arguments for the Write command. */
        /* None */

        /* Input arguments for the Read command. */
        /* None */

        /* Output arguments for the Read command. */
        /* None */

        /* Input arguments for the Full Erase command. */
        /* NONE */

        /* Output arguments for the Full Erase command. */
        /* NONE */

        /* Input arguments for the Buffer Erase command. */
        /* None */

        /* Output arguments for the Buffer Erase command. */
        /* NONE */
    } argument;
};

/*----------------------------------------------------------------------------
 *         Global variables
 *----------------------------------------------------------------------------*/

/* End of program space (code + data). */
extern unsigned int end;
extern unsigned int _sstack;

/* Startup time of main oscillator (in number of slow clock ticks). */
#define USER_OSCOUNT(a)           ((a) << 8)
/* USB PLL divisor value to obtain a 48MHz clock. */
#define USER_USBDIV(a)            ((a) << 28)
/* PLL frequency range. */
#define USER_CKGR_PLL(a)          ((a) << 14)
/* PLL startup time (in number of slow clock ticks). */
#define USER_PLLCOUNT(a)          ((a) << 8)
/* PLL MUL value. */
#define USER_MUL(a)               ((a) << 16)
/* PLL DIV value. */
#define USER_DIV(a)               (a)
/* Master clock prescaler value. */
#if defined(at91sam3u4)
#define USER_PRESCALER(a)         ((a) << 4)
#else
#define USER_PRESCALER(a)         ((a) << 2)
#endif
/* Define clock timeout */
#define CLOCK_TIMEOUT             0xFFFFFFFF

#if defined(at91sam7x512)
/**
 * \brief  Configure the PMC in bypass mode. An external clock should be input to XIN as the source clock.
 *
 * \param extClk  The frequency of the external clock
 */
static void bypass_LowLevelInit (unsigned int extClk)
{
    unsigned char i;
    int oscount, usbdiv, range, pllcount, mul, div, prescaler;

    /* Set flash wait states in the EFC */
    AT91C_BASE_EFC0->EFC_FMR = AT91C_MC_FWS_1FWS;
    AT91C_BASE_EFC1->EFC_FMR = AT91C_MC_FWS_1FWS;

    switch (extClk) {
    /* When external clock frequency is 12MHz */
    case 12000000:
        /* Master Clock = extClk * (mul + 1) / div = 12MHz * (7 + 1) / 1 = 96MHz */
        mul = USER_MUL(7);
        div = USER_DIV(1);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 96MHz / 2 = 48MHz */
        usbdiv = USER_USBDIV(1);
        /* Please refer to Power Management Controller of the product datasheet */
        range = USER_CKGR_PLL(0);
        pllcount = USER_PLLCOUNT(16);
        oscount = USER_OSCOUNT(0x40);
        break;

    /* When external clock frequency is 18.432MHz */
    case 18432000:
        /* Master Clock = extClk * (mul + 1) / div = 18.432MHz * (72 + 1) / 14 = 96MHz */
        mul = USER_MUL(72);
        div = USER_DIV(14);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 96MHz / 2 = 48MHz */
        usbdiv = USER_USBDIV(1);
        /* Please refer to Power Management Controller of the product datasheet ¡Á/
        range = USER_CKGR_PLL(0);
        pllcount = USER_PLLCOUNT(16);
        oscount = USER_OSCOUNT(0x40);
        break;
    
    /* other master clock configurations */
    
    default:
        break;
    }

    AT91C_BASE_PMC->PMC_MOR = oscount | AT91C_CKGR_OSCBYPASS;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    /* Initialize PLL at 96MHz (96.109) and USB clock to 48MHz */
    AT91C_BASE_PMC->PMC_PLLR = (1 << 29) | usbdiv | range | pllcount | mul | div;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK));

    /* Wait for the master clock if it was already initialized */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to fast clock
     **********************/
    /* Switch to slow clock + prescaler */
    AT91C_BASE_PMC->PMC_MCKR = prescaler;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to fast clock + prescaler */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Initialize AIC
     ****************/
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_SVR[0] = 0;
    for (i = 1; i < 31; i++) {

        AT91C_BASE_AIC->AIC_SVR[i] = 0;
    }
    AT91C_BASE_AIC->AIC_SPU = 0;

    /* Unstack nested interrupts */
    for (i = 0; i < 8 ; i++) {

        AT91C_BASE_AIC->AIC_EOICR = 0;
    }

    /* Enable Debug mode */
    AT91C_BASE_AIC->AIC_DCR = AT91C_AIC_DCR_PROT;

    /* Watchdog initialization
     *************************/
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Remap
     *******/
    BOARD_RemapRam();

    /**
     * Disable RTT and PIT interrupts (potential problem when program A
     * configures RTT, then program B wants to use PIT only, interrupts
     * from the RTT will still occur since they both use AT91C_ID_SYS)
     */
    AT91C_BASE_RTTC->RTTC_RTMR &= ~(AT91C_RTTC_ALMIEN | AT91C_RTTC_RTTINCIEN);
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN;

}

/**
 * \brief  Configure the PMC if the frequency of the external oscillator is different from the one mounted on EK
 * \param crystalFreq  The frequency of the external oscillator
 */
static void user_defined_LowlevelInit (unsigned int crystalFreq)
{
    unsigned char i;
    int oscount, usbdiv, range, pllcount, mul, div, prescaler;

    /* Set flash wait states in the EFC */
    AT91C_BASE_EFC0->EFC_FMR = AT91C_MC_FWS_1FWS;
    AT91C_BASE_EFC1->EFC_FMR = AT91C_MC_FWS_1FWS;

    switch (crystalFreq) {
    /* When the frequency of on-board crystal oscillator is 12MHz */
    case 12000000:
        /* Master Clock = extClk * (mul + 1) / div = 12MHz * (7 + 1) / 1 = 96MHz */
        mul = USER_MUL(7);
        div = USER_DIV(1);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 96MHz / 2 = 48MHz */
        usbdiv = USER_USBDIV(1);
        /* Please refer to Power Management Controller of the product datasheet */
        range = USER_CKGR_PLL(0);
        pllcount = USER_PLLCOUNT(16);
        oscount = USER_OSCOUNT(0x40);
        break;
        
     /* other master clock configurations */
    
    default:
        break;
    }

    /* Initialize main oscillator */
    AT91C_BASE_PMC->PMC_MOR = oscount | AT91C_CKGR_MOSCEN;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));
    
    /* Initialize PLL at 96MHz (96.109) and USB clock to 48MHz */
    AT91C_BASE_PMC->PMC_PLLR = (1 << 29) | usbdiv | range | pllcount | mul | div;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK));

    /* Wait for the master clock if it was already initialized */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to slow clock + prescaler */
    AT91C_BASE_PMC->PMC_MCKR = prescaler;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to fast clock + prescaler */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Initialize AIC */
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_SVR[0] = 0;
    for (i = 1; i < 31; i++) {
        AT91C_BASE_AIC->AIC_SVR[i] = 0;
    }
    AT91C_BASE_AIC->AIC_SPU = 0;

    /* Unstack nested interrupts */
    for (i = 0; i < 8 ; i++) {
        AT91C_BASE_AIC->AIC_EOICR = 0;
    }

    /* Enable Debug mode */
    AT91C_BASE_AIC->AIC_DCR = AT91C_AIC_DCR_PROT;

    /* Watchdog initialization */
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Remap the internal SRAM at 0x0 */
    BOARD_RemapRam();

    /**
     * Disable RTT and PIT interrupts (potential problem when program A
     * configures RTT, then program B wants to use PIT only, interrupts
     * from the RTT will still occur since they both use AT91C_ID_SYS)
     */
    AT91C_BASE_RTTC->RTTC_RTMR &= ~(AT91C_RTTC_ALMIEN | AT91C_RTTC_RTTINCIEN);
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN;
}

#elif defined(at91sam9xe512)
/**
 * \brief  Configure the PMC in bypass mode. An external clock should be input to XIN as the source clock.
 *
 * \param extClk  The frequency of the external clock
 */
static void bypass_LowLevelInit (unsigned int extClk)
{
    unsigned char i;
    int oscount, usbdiv, range1, range2, mula, mulb, diva, divb, pllacount, pllbcount, prescaler;

    switch (extClk) {
    /* When external clock frequency is 12MHz */
    case 12000000:
        /* Master Clock A = extClk * (mula + 1) / diva = 12MHz * (49 + 1) / 3 = 200MHz */
        mula = USER_MUL(49);
        diva = USER_DIV(3);
        /* Master Clock A = extClk * (mulb + 1) / divb = 12MHz * (15 + 1) / 1 = 192MHz */
        mulb = USER_MUL(15);
        divb = USER_DIV(1);
        /* Processor Clock = Master Clock / prescaler = 200MHz / 2 = 100MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 192MHz / 4 = 48MHz */
        usbdiv = USER_USBDIV(2);
        /* Please refer to Power Management Controller of the product datasheet */
        range1 = USER_CKGR_PLL(2);
        range2 = USER_CKGR_PLL(0);
        pllacount = USER_PLLCOUNT(63);
        pllbcount = USER_PLLCOUNT(63);
        oscount = USER_OSCOUNT(64);
        break;

    /* When external clock frequency is 18.432MHz */
    case 18432000:
        /* Master Clock A = extClk * (mula + 1) / diva = 18.432MHz * (96 + 1) / 9 = 200MHz */
        mula = USER_MUL(96);
        diva = USER_DIV(9);
        /* Master Clock A = extClk * (mulb + 1) / divb = 18.432MHz * (124 + 1) / 12 = 192MHz */
        mulb = USER_MUL(124);
        divb = USER_DIV(12);
        /* Processor Clock = Master Clock / prescaler = 200MHz / 2 = 100MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 192MHz / 4 = 48MHz */
        usbdiv = USER_USBDIV(2);
        /* Please refer to Power Management Controller of the product datasheet */
        range1 = (0x1 << 29) | USER_CKGR_PLL(2);
        range2 = USER_CKGR_PLL(0);
        pllacount = USER_PLLCOUNT(63);
        pllbcount = USER_PLLCOUNT(63);
        oscount = USER_OSCOUNT(64);
        break;
    
    /* other master clock configurations */
    
    default:
        break;
    }

    /* Set flash wait states */
    AT91C_BASE_EFC->EFC_FMR = 6 << 8;

#if !defined(sdram)
    /* Initialize main oscillator */
    AT91C_BASE_PMC->PMC_MOR = oscount | AT91C_CKGR_OSCBYPASS;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    /* Initialize PLLA at 200MHz (198.656) */
    AT91C_BASE_PMC->PMC_PLLAR = (1 << 29) | range1 | pllacount | mula | diva;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKA));

    /* Initialize PLLB for USB usage (if not already locked) */
    if (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKB)) {
        AT91C_BASE_PMC->PMC_PLLBR = usbdiv | range2 | pllbcount | mulb | divb;
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKB));
    }

    /* Wait for the master clock if it was already initialized */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to main oscillator + prescaler */
    AT91C_BASE_PMC->PMC_MCKR = prescaler;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to PLL + prescaler */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLLA_CLK;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));
#endif //#if !defined(sdram)

    /* Initialize AIC */
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_SVR[0] = 0;
    for (i = 1; i < 31; i++) {
        AT91C_BASE_AIC->AIC_SVR[i] = 0;
    }
    AT91C_BASE_AIC->AIC_SPU = 0;

    /* Unstack nested interrupts */
    for (i = 0; i < 8 ; i++) {
        AT91C_BASE_AIC->AIC_EOICR = 0;
    }

    /* Watchdog initialization */
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Remap */
    BOARD_RemapRam();

    /** 
     * Disable RTT and PIT interrupts (potential problem when program A
     * configures RTT, then program B wants to use PIT only, interrupts
     * from the RTT will still occur since they both use AT91C_ID_SYS)
     */
    AT91C_BASE_RTTC->RTTC_RTMR &= ~(AT91C_RTTC_ALMIEN | AT91C_RTTC_RTTINCIEN);
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN;
}

/**
 * \brief  Configure the PMC if the frequency of the external oscillator is different from the one mounted on EK
 * \param crystalFreq  The frequency of the external oscillator
 */
static void user_defined_LowlevelInit (unsigned int crystalFreq)
{
    unsigned char i;
    int oscount, usbdiv, range1, range2, mula, mulb, diva, divb, pllacount, pllbcount, prescaler;

    switch (crystalFreq) {
    /* When external clock frequency is 12MHz */
    case 12000000:
        /* Master Clock A = extClk * (mula + 1) / diva = 12MHz * (49 + 1) / 3 = 200MHz */
        mula = USER_MUL(49);
        diva = USER_DIV(3);
        /* Master Clock A = extClk * (mulb + 1) / divb = 12MHz * (15 + 1) / 1 = 192MHz */
        mulb = USER_MUL(15);
        divb = USER_DIV(1);
        /* Processor Clock = Master Clock / prescaler = 200MHz / 2 = 100MHz */
        prescaler = USER_PRESCALER(1);
        /* USB Clock = Master Clock / usbdiv = 192MHz / 4 = 48MHz */
        usbdiv = USER_USBDIV(2);
        /* Please refer to Power Management Controller of the product datasheet */
        range1 = (0x1 << 29) | USER_CKGR_PLL(2);
        range2 = USER_CKGR_PLL(0);
        pllacount = USER_PLLCOUNT(63);
        pllbcount = USER_PLLCOUNT(63);
        oscount = USER_OSCOUNT(64);
        break;

    /* other master clock configurations */
    
    default:
        break;
    }

    /* Set flash wait states */
    AT91C_BASE_EFC->EFC_FMR = 6 << 8;

#if !defined(sdram)
    /* Initialize main oscillator */
    AT91C_BASE_PMC->PMC_MOR = oscount | AT91C_CKGR_MOSCEN;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    /* Initialize PLLA at 200MHz (198.656) */
    AT91C_BASE_PMC->PMC_PLLAR = (1 << 29) | range1 | pllacount | mula | diva;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKA));

    /* Initialize PLLB for USB usage (if not already locked) */
    if (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKB)) {
        AT91C_BASE_PMC->PMC_PLLBR = usbdiv | range2 | pllbcount | mulb | divb;
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKB));
    }

    /* Wait for the master clock if it was already initialized */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to main oscillator + prescaler */
    AT91C_BASE_PMC->PMC_MCKR = prescaler;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    /* Switch to PLL + prescaler */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLLA_CLK;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));
#endif /* #if !defined(sdram) */

    /* Initialize AIC */
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_SVR[0] = 0;
    for (i = 1; i < 31; i++) {
        AT91C_BASE_AIC->AIC_SVR[i] = 0;
    }
    AT91C_BASE_AIC->AIC_SPU = 0;

    /* Unstack nested interrupts */
    for (i = 0; i < 8 ; i++) {
        AT91C_BASE_AIC->AIC_EOICR = 0;
    }

    /* Watchdog initialization */
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Remap */
    BOARD_RemapRam();

    /** 
     * Disable RTT and PIT interrupts (potential problem when program A
     * configures RTT, then program B wants to use PIT only, interrupts
     * from the RTT will still occur since they both use AT91C_ID_SYS)
     */
    AT91C_BASE_RTTC->RTTC_RTMR &= ~(AT91C_RTTC_ALMIEN | AT91C_RTTC_RTTINCIEN);
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN;
}

#elif defined(at91sam3u4)
/**
 * \brief  Configure the PMC in bypass mode. An external clock should be input to XIN as the source clock.
 *
 * \param extClk  The frequency of the external clock
 */
static void bypass_LowLevelInit (unsigned int extClk)
{
    int oscount, pllcount, mul, div, prescaler, mck;
    unsigned int timeout = 0;

    /* Enable NRST reset
     ************************************/
    AT91C_BASE_RSTC->RSTC_RMR |= (0xA5 << 24) | AT91C_RSTC_URSTEN;

#if defined(SAM_BA)
    { asm volatile ("cpsid i"); }    
#endif
    
    /* Set 2 WS for Embedded Flash Access
     ************************************/
    AT91C_BASE_EFC0->EFC_FMR = (0x6 <<  8);
#if defined(AT91C_BASE_EFC1)    
    AT91C_BASE_EFC1->EFC_FMR = (0x6 <<  8);
#endif

    switch (extClk) {
    /* When external clock frequency is 12MHz */
    case 12000000:
        /* Master Clock = extClk * (mul + 1) / div = 12MHz * (7 + 1) / 1 = 96MHz */
        mul = USER_MUL(7);
        div = USER_DIV(1);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* Please refer to Power Management Controller of the product datasheet */
        pllcount = USER_PLLCOUNT(1);
        oscount = USER_OSCOUNT(8);
        break;

    /* When external clock frequency is 18.432MHz */
    case 18432000:
        /* Master Clock = extClk * (mul + 1) / div = 18.432MHz * (72 + 1) / 14 = 96MHz */
        mul = USER_MUL(72);
        div = USER_DIV(14);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* Please refer to Power Management Controller of the product datasheet */
        pllcount = USER_PLLCOUNT(1);
        oscount = USER_OSCOUNT(8);
        break;
    
    /* other master clock configurations */
    
    default:
        break;
    }
    mck = prescaler | AT91C_PMC_CSS_PLLA_CLK;

    /* Watchdog initialization
     *************************/
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Initialize main oscillator
     ****************************/
    if(!(AT91C_BASE_PMC->PMC_MOR & AT91C_CKGR_MOSCSEL))
    {

        AT91C_BASE_PMC->PMC_MOR = (0x37 << 16) | oscount | AT91C_CKGR_MOSCXTBY;
        timeout = 0;
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCXTS) && (timeout++ < CLOCK_TIMEOUT));
        
    }

    /* Switch to 3-20MHz Xtal oscillator */
    AT91C_BASE_PMC->PMC_MOR = (0x37 << 16) | oscount | AT91C_CKGR_MOSCRCEN | AT91C_CKGR_MOSCXTBY;
    AT91C_BASE_PMC->PMC_MOR |= (0x37 << 16) | AT91C_CKGR_MOSCSEL;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCSELS) && (timeout++ < CLOCK_TIMEOUT));
    AT91C_BASE_PMC->PMC_MCKR = (AT91C_BASE_PMC->PMC_MCKR & ~AT91C_PMC_CSS) | AT91C_PMC_CSS_MAIN_CLK;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    /* Initialize PLLA */
    AT91C_BASE_PMC->PMC_PLLAR = (1 << 29) | mul | pllcount | div;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKA) && (timeout++ < CLOCK_TIMEOUT));

    /* Initialize UTMI for USB usage, can be disabled if not using USB for the sake of saving power*/
    AT91C_BASE_CKGR->CKGR_UCKR |= (AT91C_CKGR_UPLLCOUNT & (3 << 20)) | AT91C_CKGR_UPLLEN;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKU) && (timeout++ < CLOCK_TIMEOUT));

    /* Switch to fast clock
     **********************/
    AT91C_BASE_PMC->PMC_MCKR = (mck & ~AT91C_PMC_CSS) | AT91C_PMC_CSS_MAIN_CLK;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    AT91C_BASE_PMC->PMC_MCKR = mck;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    /* Enable clock for UART
     ************************/
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_DBGU);

    /* Optimize CPU setting for speed,  for engineering samples only */
    SetDefaultMaster(1);
}

/**
 * \brief  Configure the PMC if the frequency of the external oscillator is different from the one mounted on EK
 * \param crystalFreq  The frequency of the external oscillator
 */
static void user_defined_LowlevelInit (unsigned int crystalFreq)
{
    int oscount, pllcount, mul, div, prescaler, mck;
    unsigned int timeout = 0;

    /* Enable NRST reset
     ************************************/
    AT91C_BASE_RSTC->RSTC_RMR |= (0xA5 << 24) | AT91C_RSTC_URSTEN;

#if defined(SAM_BA)
    { asm volatile ("cpsid i"); }    
#endif
    
    /* Set 2 WS for Embedded Flash Access
     ************************************/
    AT91C_BASE_EFC0->EFC_FMR = (0x6 <<  8);
#if defined(AT91C_BASE_EFC1)    
    AT91C_BASE_EFC1->EFC_FMR = (0x6 <<  8);
#endif

    switch (crystalFreq) {
    /* When crystal oscillator frequency is 18.432MHz */
    case 18432000:
        /* Master Clock = crystalFreq * (mul + 1) / div = 18.432MHz * (72 + 1) / 14 = 96MHz */
        mul = USER_MUL(72);
        div = USER_DIV(14);
        /* Processor Clock = Master Clock / prescaler = 96MHz / 2 = 48MHz */
        prescaler = USER_PRESCALER(1);
        /* Please refer to Power Management Controller of the product datasheet */
        pllcount = USER_PLLCOUNT(1);
        oscount = USER_OSCOUNT(8);
        break;
    
    /* other master clock configurations */
    
    default:
        break;
    }
    mck = prescaler | AT91C_PMC_CSS_PLLA_CLK;

    /* Watchdog initialization
     *************************/
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Initialize main oscillator
     ****************************/
    if(!(AT91C_BASE_PMC->PMC_MOR & AT91C_CKGR_MOSCSEL))
    {

        AT91C_BASE_PMC->PMC_MOR = (0x37 << 16) | oscount | AT91C_CKGR_MOSCRCEN | AT91C_CKGR_MOSCXTEN;
        timeout = 0;
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCXTS) && (timeout++ < CLOCK_TIMEOUT));
        
    }
 
    /* Switch to 3-20MHz Xtal oscillator */
    AT91C_BASE_PMC->PMC_MOR = (0x37 << 16) | oscount | AT91C_CKGR_MOSCRCEN | AT91C_CKGR_MOSCXTEN | AT91C_CKGR_MOSCSEL;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCSELS) && (timeout++ < CLOCK_TIMEOUT));
    AT91C_BASE_PMC->PMC_MCKR = (AT91C_BASE_PMC->PMC_MCKR & ~AT91C_PMC_CSS) | AT91C_PMC_CSS_MAIN_CLK;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    /* Initialize PLLA */
    AT91C_BASE_PMC->PMC_PLLAR = (1 << 29) | mul | div | pllcount;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKA) && (timeout++ < CLOCK_TIMEOUT));

    /* Initialize UTMI for USB usage, can be disabled if not using USB for the sake of saving power*/
    AT91C_BASE_CKGR->CKGR_UCKR |= (AT91C_CKGR_UPLLCOUNT & (3 << 20)) | AT91C_CKGR_UPLLEN;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCKU) && (timeout++ < CLOCK_TIMEOUT));

    /* Switch to fast clock
     **********************/
    AT91C_BASE_PMC->PMC_MCKR = (mck & ~AT91C_PMC_CSS) | AT91C_PMC_CSS_MAIN_CLK;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    AT91C_BASE_PMC->PMC_MCKR = mck;
    timeout = 0;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) && (timeout++ < CLOCK_TIMEOUT));

    /* Enable clock for UART
     ************************/
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_DBGU);

    /* Optimize CPU setting for speed,  for engineering samples only */
    SetDefaultMaster(1);

}

#else
/**
 * \brief  Configure the PMC in bypass mode. An external clock should be input to XIN as the source clock.
 *
 * \param extClk  The frequency of the external clock
 */
static void user_defined_LowlevelInit (unsigned int freq)
{
    /* Put your own low level initialization here */
}

/**
 * \brief  Configure the PMC if the frequency of the external oscillator is different from the one mounted on EK
 * \param crystalFreq  The frequency of the external oscillator
 */
static void bypass_LowLevelInit (unsigned int freq)
{
    /* Put your own low level initialization here */
}
#endif //defined(at91sam7x512)

/**
 * \brief  Configure the PMC as EK setting
 */
static void EK_LowLevelInit (void)
{
    LowLevelInit();
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief  Applet main entry. This function decodes received command and executes it.
 *
 * \param argc  always 1
 * \param argv  Address of the argument area..
 */
int main(int argc, char **argv)
{
    struct _Mailbox *pMailbox = (struct _Mailbox *) argv;
    unsigned int mode, crystalFreq, extClk, comType;
    
    mode = pMailbox->argument.inputInit.Mode;
    crystalFreq = pMailbox->argument.inputInit.crystalFreq;
    extClk = pMailbox->argument.inputInit.extClk;
    comType = pMailbox->argument.inputInit.comType;
    
    switch (mode) {
        case EK_MODE: 
            EK_LowLevelInit();
            pMailbox->status = APPLET_SUCCESS;
            break;
        case USER_DEFINED_CRYSTAL:
            user_defined_LowlevelInit(crystalFreq);
#if defined(at91sam3u4) || defined(at91sam7x512) || defined(at91sam9ex512)
            pMailbox->status = APPLET_SUCCESS;
#else
            pMailbox->status = APPLET_DEV_UNKNOWN;
#endif
            break;
        case BYPASS_MODE:
            bypass_LowLevelInit(extClk);
#if defined(at91sam3u4) || defined(at91sam7x512) || defined(at91sam9ex512)
            pMailbox->status = APPLET_SUCCESS;
#else
            pMailbox->status = APPLET_DEV_UNKNOWN;
#endif
            break;
        default:
            pMailbox->status = APPLET_DEV_UNKNOWN;
            break;
    }
    DBGU_Configure(DBGU_STANDARD, 115200, BOARD_MCK);
    
    /* Notify the host application of the end of the command processing */
    pMailbox->command = ~(pMailbox->command);
    /* Send ACK character */
    if (comType == DBGU_COM_TYPE) {
        DBGU_PutChar(0x6);
    }

    return 0;
}

