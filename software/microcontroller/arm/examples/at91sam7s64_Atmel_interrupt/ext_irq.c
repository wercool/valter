//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : ext_irq.c
//* Object              : External interrupt handler for irq
//*                     : Use LED5 & LED6 for status interrupt
//* 1.0 24/Jun/04 JPP   : Creation
//* 1.2 29/Aug/05 JPP   : Update AIC definion
//*----------------------------------------------------------------------------

// Include Standard LIB  files
#include "Board.h"

int status_irq = 0 ;

//*----------------------------------------------------------------------------
//* Function Name       : at91_IRQ0_handler
//* Object              : Irq Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void at91_IRQ0_handler(void)
{
    if (status_irq == 0)
        status_irq = 1;
    else
        status_irq = 0;

    // Read the output state
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & LED2 ) == LED2 )
    {
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2 );
    }
    else
    {
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2 );
    }

/*
    // while SW3 is push loop
    while ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & SW3_MASK ) != SW3_MASK );
*/
}

//*----------------------------------------------------------------------------
//* Function Name       : FIQ_init_handler
//* Object              : Irq Handler called by the FIQ interrupt with AT91
//*                       compatibility
///*----------------------------------------------------------------------------
__ramfunc void FIQ_init_handler(void)
{
    //* Read the output state
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & LED3 ) == LED3 )
    {
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3 );
    }
    else
    {
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3 );
    }

}

