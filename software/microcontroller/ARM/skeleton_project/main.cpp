/*
 AT91SAM7S example application "C++" - simple version
*/

#include <stdint.h>
#include <stdio.h>
// newlib 1.14.0 workaround
#if 0
extern "C"
{
    int	_EXFUN(iscanf, (const char *, ...) _ATTRIBUTE ((__format__ (__scanf__, 1, 2))));
}
#endif
#include "Board.h"
#include "dbgu.h"
#include "swi.h"


#define RTTC_INTERRUPT_LEVEL   0
#define PIV_200_MS             600000  //* 200 ms for 48 MHz

void Periodic_Interval_Timer_handler(void)
{
    volatile uint32_t status;

    // Interrupt Acknowledge
    status = AT91C_BASE_PITC->PITC_PIVR;
    // status = status;

    // toggle LED1
    if ((AT91F_PIO_GetInput(AT91C_BASE_PIOA) & LED1 ) == LED1 )
    {
        AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1 );
    }
    else
    {
        AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1 );
    }
}

static void device_init(void)
{
    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);

    // Set-up the PIO
    // First, enable the clock of the PIO and set the LEDs in output
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA ) ;

    // then, we configure the PIO Lines corresponding to LEDs
    // to be outputs. No need to set these pins to be driven by the PIO because it is GPIO pins only.
    /// AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, LED_MASK ) ;
    AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, LED1 ) ;

    // Clear the LED's. On the SAM7S-EK we must apply a "1" to turn off LEDs
    /// AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED_MASK ) ;
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1 ) ;

    // define switch SW1 at PIO input
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA,SW1_MASK);

    // Set-up PIT interrupt
    AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_SYS, RTTC_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE, Periodic_Interval_Timer_handler);
    AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN | AT91C_PITC_PITIEN | PIV_200_MS;  //  IRQ enable CPC
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_SYS);

    // Set-up DBGU Usart ("UART2")
    AT91F_DBGU_Init();
}


class LED2_class
{
public:
    LED2_class();
    void on();
    void off ();
};

LED2_class::LED2_class()
{
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2 );
    AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, LED2 );
}

void LED2_class::on()
{
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2 );
}


void LED2_class::off()
{
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2 );
}

LED2_class led2;

int main(void)
{

    device_init();

    IntEnable();  // the swi-call

    while (1)
    {

    }

    return 0; /* never reached */
}

