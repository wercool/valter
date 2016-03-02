//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : main.c
//* Object              : main application written in C
//* 1.0 24/Jun/04 JPP   : Creation
//* 1.1 21/Feb/05 JPP   : Update AT91C_RSTC_URSTEN
//* 1.2 29/Aug/05 JPP   : Update AIC definion
//* slightly modified for the WinARM example - M.Thomas (not Atmel)
//*----------------------------------------------------------------------------

// Include Standard files
#include "stdio.h"
#include "string.h"

// Include Standard LIB  files
#include "Board.h"

#include "interrupt_timer.h"
#include "cdc_enumerate.h"

//*   Waiting time between LED1 and LED2
#define     WAIT_TIME       MCK

#define PIO_INTERRUPT_LEVEL	6
#define IRQ0_INTERRUPT_LEVEL	2
#define SOFT_INTERRUPT_LEVEL	5
#define FIQ_INTERRUPT_LEVEL	0

#define MSG_SIZE        1000

char msg[MSG_SIZE];
static volatile unsigned int cnt;
static volatile unsigned int status_irq = 0 ;

// Use the Library Handler defined in file periph/pio/pio_irq/irq_pio.s

extern void FIQ_init_handler(void);

// External Function Prototype
extern void Usart_init (void);

struct _AT91S_CDC   pCDC;

void printTrace(char * trace)
{
    pCDC.Write(&pCDC, trace, strlen(trace));
}

//*----------------------------------------------------------------------------
//* Function Name       : at91_IRQ0_handler
//* Object              : Irq Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void at91_IRQ0_handler(void)
{
    status_irq = 1;
}

//*----------------------------------------------------------------------------
//* Function Name       : aic_software_interrupt
//* Object              : Software interrupt function
//* Input Parameters    : none
//* Output Parameters   : none
//* Functions called    : at91_pio_write
//*----------------------------------------------------------------------------
__ramfunc void aic_software_interrupt(void)
{
    //* Read the output state
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & LED2 ) == LED2 )
    {
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2 );
    }
    else
    {
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2 );
    }
}

//*----------------------------------------------------------------------------
//* Function Name       : pio_c_irq_handler
//* Object              : Irq Handler called by the irq_pio.s
//* Input Parameters    : none
//* Output Parameters   : none
//* Functions called    : at91_pio_read, at91_pio_write
//*----------------------------------------------------------------------------
__ramfunc void pio_c_irq_handler ( void )
{
int dummy;
    //* Read the output state
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & LED2 ) == LED2 )
    {
       AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
    }
    else
    {
          AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2);
    }
    //* enable the next PIO IRQ
    dummy =AT91C_BASE_PIOA->PIO_ISR;
    //* suppress the compilation warning
    dummy =dummy;
    //* while SW1 is push wait
    while ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & SW1_MASK ) != SW1_MASK );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_Open
//* \brief This function Open the USB device
//*----------------------------------------------------------------------------
void AT91F_USB_Open(void)
{
    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;

    // Specific Chip USB Initialisation
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);

    // Enable UDP PullUp (USB_DP_PUP) : enable & Clear of the corresponding PIO
    // Set in PIO mode and Configure in Output
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA,AT91C_PIO_PA16);
    // Clear for set the Pul up resistor
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA,AT91C_PIO_PA16);

    // CDC Open by structure initialization
    AT91F_CDC_Open(&pCDC, AT91C_BASE_UDP);
}

//*----------------------------------------------------------------------------
//* Function Name       : delay
//* Object              : Wait
//* Input Parameters    : none
//* Output Parameters   : none
//* Functions called    : none
//*----------------------------------------------------------------------------
void delay(void)
{
//* Set in Volatile for Optimisation
    volatile unsigned int    i ;
//* loop delay
    for ( i = 0 ;(i < WAIT_TIME/100 );i++ ) ;
}

void Delay(int delayVal)
{
  volatile unsigned int waiting_time ;
  for(waiting_time = 0; waiting_time < delayVal; waiting_time++) ;
}

//----------------------------------------------------------------------------
// Function Name       : main
// Object              : Main interrupt function
//		level timer 0 => 1
//	SW2	level Irq0    => 2
//		level timer 1 => 4
//	SW4	level PIOA    => 6
//		level USART   => 7
//		LEVEL FIQ     => MAX
// Input Parameters    : none
// Output Parameters   : TRUE
//----------------------------------------------------------------------------
int main( void )
// Begin
{
    AT91PS_AIC     pAic;
    // Load System pAic Base address
        pAic = AT91C_BASE_AIC;

    // Enable User Reset and set its minimal assertion to 960 us
	AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);

	// Init
    cnt = 0;
    status_irq = 0;

    // Init USB device
    AT91F_USB_Open();
    // Init USB device
    // Wait for the end of enumeration
    while (!pCDC.IsConfigured(&pCDC));

	// First, enable the clock of the PIOB
	AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA ) ;

   	// then, we configure the PIO Lines corresponding to LED1 to LED8
   	// to be outputs. No need to set these pins to be driven by the PIO because it is GPIO pins only.
    	AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, LED_MASK ) ;
   	// Clear the LED's. On the EB55 we must apply a "1" to turn off LEDs
   	AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED_MASK ) ;

	// mt: added reset enable to make the board reset-button "useful"
	AT91F_RSTSetMode( AT91C_BASE_RSTC , AT91C_RSTC_URSTEN );

    // open external PIO interrupt
    // define switch SW1 at PIO input for interrupt IRQ loop
	AT91F_PIO_CfgInput(AT91C_BASE_PIOA, SW1_MASK | SW2_MASK);

/*
	AT91F_AIC_ConfigureIt ( pAic, AT91C_ID_PIOA, PIO_INTERRUPT_LEVEL,AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, pio_c_irq_handler);
	AT91F_PIO_InterruptEnable(AT91C_BASE_PIOA,SW1_MASK);
	// set the interrupt by software
	AT91F_AIC_EnableIt (pAic, AT91C_ID_PIOA);
*/
    // open external IRQ interrupt
   	AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, SW2_MASK, 0);
   	// open external IRQ0 interrupt
	AT91F_AIC_ConfigureIt ( pAic, AT91C_ID_IRQ0, IRQ0_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, at91_IRQ0_handler);
	AT91F_AIC_EnableIt (pAic, AT91C_ID_IRQ0);
/*
    // Open the software interrupt on the AIC
	AT91F_AIC_ConfigureIt ( pAic, AT91C_ID_SYS, SOFT_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE,  aic_software_interrupt);
	AT91F_AIC_EnableIt (pAic, AT91C_ID_SYS);

    // open  FIQ interrupt
	AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA,SW1_MASK,0);
	AT91F_AIC_ConfigureIt ( pAic, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL,AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, FIQ_init_handler);
	AT91F_AIC_EnableIt (pAic, AT91C_ID_FIQ);
	// generate FIQ interrupt by software
	AT91F_AIC_Trig (pAic,AT91C_ID_FIQ) ;

    // Init timer interrupt
        timer_init();

    // Init Usart
        Usart_init();

    // generate software interrupt
        AT91F_AIC_Trig(pAic,AT91C_ID_SYS) ;
*/
    for (;;)
    {
        if (status_irq == 0)
        {
            AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1 );
            Delay(10);
            AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1 );
            AT91F_AIC_EnableIt(pAic, AT91C_ID_IRQ0);
        }

        while (status_irq == 0)
        {
            cnt++;
            if (cnt > 50000)
            break;
        }

        if (status_irq == 1 || cnt > 50000)
        {
            AT91F_AIC_DisableIt(pAic, AT91C_ID_IRQ0);
            if (cnt > 50000)
            {
                sprintf((char *)msg, "FAILED\n");
            }
            else
            {
                sprintf((char *)msg, "COUNTER: %d\n", cnt);
            }
            printTrace(msg);

            status_irq = 0;
            cnt = 0;

            AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2 );
            Delay(100000);
            AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2 );
        }
    }

//* End
}
