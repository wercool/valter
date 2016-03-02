/******************************************************************************/
/*                                                                            */
/*  HELLO.C:  Hello World Example                                             */
/*                                                                            */
/******************************************************************************/
/*  ported and extended for arm-elf-gcc / WinARM by                           */
/*  Martin Thomas, KL, .de <eversmith@heizung-thomas.de>                      */
/*  modifications Copyright Martin Thomas 2005                                */
/*                                                                            */
/*  Based on file that has been a part of the uVision/ARM development         */
/*  tools, Copyright KEIL ELEKTRONIK GmbH 2002-2004                           */
/******************************************************************************/

/* see file Abstract.txt for more info on the gcc-port */

#include <stdio.h>                          /* I/O Functions */
#include <AT91SAM7S64.h>                    /* AT91SAMT7S64 definitions */
#include "Board.h"
// #include "lib_AT91SAM7S64.h"
#include "serial.h"

extern void init_timer  (void);             /* Initialize Timer */
extern volatile unsigned long timeval;      /* Current Time Tick */

extern void init_extint (void);             /* Initialize External Interrupt */

AT91S_PIO * pPIO = AT91C_BASE_PIOA;         /* Global Pointer to PIO */


void wait(unsigned long time)
{
	unsigned long tick;
	
	tick = timeval;
	
	/* Wait for specified Time */
	while ((unsigned long)(timeval - tick) < time);
}

#if 0
static void test(void)
{
	unsigned short q = 0;
	volatile unsigned long* p = (unsigned long)(0);
	do {
		iprintf("Addr %i %08x\n", q, *p);
		q++;
		p++;
	} while (q<22);
}
#endif


/*** Main Program ***/

int main (void) {
  unsigned long n;
  char flag;
   
	// enable reset-button (mt)
	// AT91F_RSTSetMode( AT91C_BASE_RSTC , AT91C_RSTC_URSTEN );
	*AT91C_RSTC_RMR = ( 0xA5000000 | AT91C_RSTC_URSTEN );
	
	*AT91C_PMC_PCER = (1 << AT91C_ID_PIOA) |  /* Enable Clock for PIO    */
		(1 << AT91C_ID_IRQ0) |  /* Enable Clock for IRQ0   */
		(1 << AT91C_ID_US0);    /* Enable Clock for USART0 */
	
	pPIO->PIO_PER  = LED_MASK;                /* Enable PIO for LED1..4  */
	pPIO->PIO_OER  = LED_MASK;                /* LED1..4 are Outputs     */
	pPIO->PIO_SODR = LED_MASK;                /* Turn off LED's ("1")    */
	pPIO->PIO_OWER = LED4;                    /* LED4 ODSR Write Enable  */
	
	uart0_init();                             /* Initialize Serial Interface */
	
	unsigned short q = 4;
	iprintf("Hello from the WinARM example!  (1 2 %i %i)\r\n", 3, q);
	// *AT91C_PIOA_CODR = LED4; 
	// test();
	
	init_timer ();                            /* Initialize Timer */
	init_extint();                            /* Initialize External Interrupt */
	
	flag = 0;
	
	while (1) {                              /* Loop forever */
	
		n = pPIO->PIO_PDSR;                   /* Read Pin Data */
		if ((n & SW1) == 0) {                /* Check if SW1 is pressed */
		pPIO->PIO_CODR = LED1;                /* Turn On LED1 */
		iprintf ("Hello World !\n");          /* Print "Hello World !" */
		wait(100);                            /* Wait 100ms */
		pPIO->PIO_SODR = LED1;                /* Turn Off LED1 */
		wait(100);                            /* Wait 100ms */
		}
		if (((n & SW3) == 0) && (n & SW4)) {   /* Check if SW3 is pressed */
			pPIO->PIO_CODR = LED3;              /* Turn On LED3 */
		}
		if ((n & SW4) == 0) {                   /* Check if SW4 is pressed */
			pPIO->PIO_SODR = LED3;               /* Turn Off LED3 */
		}
		
		if ( uart0_kbhit() ) {
			int c = uart0_getc();
			iprintf ("You've pressed the \"%c\" key\n", (char)(c));
		}
		
	} // while
	
}
