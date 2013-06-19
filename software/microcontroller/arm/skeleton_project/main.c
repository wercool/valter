/*
 AT91SAM7S example application "C++" - simple version
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
// newlib 1.14.0 workaround
#if 0
extern "C"
{
    int _EXFUN(iscanf, (const char *, ...) _ATTRIBUTE ((__format__ (__scanf__, 1, 2))));
}
#endif
#include "Board.h"
#include "dbgu.h"
#include "swi.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "delay.h"


#define RTTC_INTERRUPT_LEVEL    0
#define FIQ_INTERRUPT_LEVEL     0
#define PIV_200_MS              600000  //* 200 ms for 48 MHz

/*
 *  CDC Functions
 */

struct _AT91S_CDC               pCDC;
//*----------------------------------------------------------------------------
//* function     AT91F_USB_Open
//*              This function Open the USB device
//*----------------------------------------------------------------------------
void AT91FUSBOpen(void)
{
    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;

    // Specific Chip USB Initialization
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);

    // Enable UDP PullUp (USB_DP_PUP) : enable & Clear of the corresponding PIO
    // Set in PIO mode and Configure in Output
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    // Clear for set the Pull up resistor
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);

    // CDC Open by structure initialization
    AT91F_CDC_Open(&pCDC, AT91C_BASE_UDP);
}

#define CDC_MSG_SIZE            512

unsigned char msg[CDC_MSG_SIZE];
unsigned char cmd[CDC_MSG_SIZE];
/*
 * Structure to keep CDC message
 */
struct cdcMessage
{
    unsigned char data[CDC_MSG_SIZE];
    unsigned int length;
};

/*
 *  Fill-in CDC Message Structure
 */
struct cdcMessage getCDCMEssage(void)
{
    struct cdcMessage cdcMessageObj;

    for(int r = 0; r < CDC_MSG_SIZE; r++)
    {
        cdcMessageObj.data[r] = '\0';
    }

    cdcMessageObj.length = pCDC.Read(&pCDC, cdcMessageObj.data, CDC_MSG_SIZE);

    return cdcMessageObj;
}


//*----------------------------------------------------------------------------
//* Function Name       : FIQInitHandler
//* Object              : IRQ Handler called by the FIQ interrupt with AT91
//*                       compatibility
///*----------------------------------------------------------------------------
void FIQInitHandler(void)
{
    AT91F_AIC_DisableIt (AT91C_BASE_AIC, AT91C_ID_SYS);
}

//*----------------------------------------------------------------------------
//* Function Name       : PeriodicIntervalTimerHandler
//* Object              : IRQ Handler called by the SYS interrupt with AT91
//*                       compatibility
///*----------------------------------------------------------------------------
void PeriodicIntervalTimerHandler(void)
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

static void InitPIO(void)
{
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15); //mainAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //inputChannel1 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //inputChannel1 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //inputChannel1 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //inputChannel1 D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //leftAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //rightAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //dcdc5VEnable

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA17);  // ADC Channel 0

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
}

//*----------------------------------------------------------------------------
//* Function Name       : DeviceInit
//* Object              : Device peripherals initialization
///*----------------------------------------------------------------------------
static void DeviceInit(void)
{
    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);

    // Set-up the PIO
    // First, enable the clock of the PIO and set the LEDs in output
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );

    InitPIO();

    // Initialize USB device
    AT91FUSBOpen();
    // Wait for the end of enumeration
    while (!pCDC.IsConfigured(&pCDC));
/*
    // then, we configure the PIO Lines corresponding to LEDs
    // to be outputs. No need to set these pins to be driven by the PIO because it is GPIO pins only.
    /// AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, LED_MASK ) ;
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, LED1);

    // Clear the LED's. On the SAM7S-EK we must apply a "1" to turn off LEDs
    /// AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED_MASK ) ;
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, LED1);

    // define switch SW1 at PIO input
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, SW1_MASK);
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA16);

    // Set-up PIT interrupt
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_SYS, RTTC_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE, PeriodicIntervalTimerHandler);
    AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN | AT91C_PITC_PITIEN | PIV_200_MS;  //  IRQ enable CPC
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_SYS);

    // open  FIQ interrupt
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, SW1_MASK, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, FIQInitHandler);
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);

    // Set-up DBGU USART ("UART2")
    //AT91F_DBGU_Init();
*/
    IntEnable();  // the swi-call
    InitPIO();
    InitADC();
}

/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;
    unsigned int input1Readings = 0;
    unsigned int input1Channel = 0;
    unsigned int input1Reading = 0;

    DeviceInit();

    while (1)
    {
        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
/*
            memcpy(msg, cdcMessageObj.data, cdcMessageObj.length);

            sprintf((char *)cmd,"MSG:%s\n", msg);
            pCDC.Write(&pCDC, cmd, strlen(cmd));
*/
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, msg, strlen(msg));

            if (strcmp(cdcMessageObj.data, "MAINACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA15 );
                sprintf((char *)msg,"MAIN ACCUMULATOR RELAY SET ON\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "MAINACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
                sprintf((char *)msg,"MAIN ACCUMULATOR RELAY SET OFF\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "DCDC5VENABLEON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA3 );
                sprintf((char *)msg,"DC/DC 5V ENABLED\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "DCDC5VENABLEOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                sprintf((char *)msg,"DC/DC 5V DISABLED\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "LEFTACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA8 );
                sprintf((char *)msg,"LEFT ACCUMULATOR RELAY SET ON\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "LEFTACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"LEFT ACCUMULATOR RELAY SET OFF\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "RIGHTACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA7 );
                sprintf((char *)msg,"RIGHT ACCUMULATOR RELAY SET ON\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "RIGHTACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"RIGHT ACCUMULATOR RELAY SET OFF\n", cdcMessageObj.data);
                pCDC.Write(&pCDC, msg, strlen(msg));
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 0;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 1;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 2;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 3;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 4;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 5;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 6;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 7;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "GETINPUT1CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 8;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "STARTINPUT1READINGS") == 0)
            {
                input1Readings = 1;
                continue;
            }
            if (strcmp(cdcMessageObj.data, "STOPINPUT1READINGS") == 0)
            {
                input1Readings = 0;
                continue;
            }
        }
        if (input1Readings)
        {
            input1Reading = getValueChannel0();
            sprintf((char *)msg,"CHANNEL[%u]: %u\n", input1Channel, input1Reading);
            pCDC.Write(&pCDC, msg, strlen(msg));
            delay_ms(100);
        }
    }

    return 0; /* never reached */
}

