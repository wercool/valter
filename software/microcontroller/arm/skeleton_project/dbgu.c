//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : Debug.c
//* Object              : Debug menu
//* Creation            : JPP   16/May/2004
//*----------------------------------------------------------------------------

#include <stdio.h>

// Include Standard files
#include "Board.h"
#include "dbgu.h"


#if AT91_DBGU_SCANF_ENABLE
#if 0
// mthomas: workaround missing entry for siscanf in stdio.h of
//          newlib 1.14.0
extern int	_EXFUN(siscanf, (const char *, const char *, ...));
// extern int	siscanf(const char *, const char *, ...);
#endif
#endif


/*---------------------------- Global Variable ------------------------------*/

//*--------------------------1--------------------------------------------------
//* \fn    AT91F_DBGU_Init
//* \brief This function is used to send a string through the DBGU channel (Very low level debugging)
//*----------------------------------------------------------------------------
void AT91F_DBGU_Init(void)
{
    //* Open PIO for DBGU
        AT91F_DBGU_CfgPIO();

    //* Configure DBGU
	AT91F_US_Configure (
		(AT91PS_USART) AT91C_BASE_DBGU,       // DBGU base address
		MCK,
		AT91C_US_ASYNC_MODE ,                 // Mode Register to be programmed
		AT91C_DBGU_BAUD ,                     // Baudrate to be programmed
		0);                                   // Timeguard to be programmed

    //* Enable Transmitter & receivier
       ((AT91PS_USART)AT91C_BASE_DBGU)->US_CR = AT91C_US_RXEN | AT91C_US_TXEN;
}
//*--------------------------1--------------------------------------------------
//* \fn    AT91F_DBGU_Printk
//* \brief This function is used to send a string through the DBGU channel (Very low level debugging)
//*----------------------------------------------------------------------------
void AT91F_DBGU_Printk(	char *buffer)
{
    while(*buffer != '\0') {
	while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_DBGU));
	AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_DBGU, *buffer++);
    }
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_US_Get
//* \brief Get a Char to USART
//*----------------------------------------------------------------------------
 int AT91F_US_Get( char *val)
{
    if ((AT91F_US_RxReady((AT91PS_USART)AT91C_BASE_DBGU)) == 0) return (false);
    else
    {
	*val= AT91F_US_GetChar((AT91PS_USART)AT91C_BASE_DBGU);
        return (true);
    }
}


#if AT91_DBGU_SCANF_ENABLE

//*----------------------------------------------------------------------------
//* \fn    AT91F_DBGU_scanf
//* \brief Get a string to USART manage Blackspace and echo
//*----------------------------------------------------------------------------
void AT91F_DBGU_scanf(char * type,unsigned int * val)
{//* Begin
    unsigned int read = 0;
    char buff[10];
    unsigned int nb_read =0;

    while( (read != 0x0D) & (nb_read != sizeof(buff)) ) {
        //* wait the USART Ready for reception
	 while((AT91C_BASE_DBGU->DBGU_CSR  & AT91C_US_RXRDY) == 0 ) ;
        //* Get a char
	read = AT91C_BASE_DBGU->DBGU_RHR ;
        buff[nb_read]= (char)read;
        //* Manage Blackspace
        while((AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXRDY) ==0)  {}
        if ((char)read == 0x08) {
            if ( nb_read != 0 ) {
              nb_read--;
              AT91C_BASE_DBGU->DBGU_THR = read;
            }
        }
        else {
          //* echo
          AT91C_BASE_DBGU->DBGU_THR = read;
          nb_read++;
        }
    }
    
	//* scan the value - mthomas: sscanf -> siscanf
    siscanf(buff, type, val);
		
}//* End


#endif
