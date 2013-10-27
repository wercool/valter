#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Board.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "pwm.h"
#include "aic.h"
#include "async.h"
#include "delay.h"

#define FIQ_INTERRUPT_LEVEL 0


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ0Handler(void)
{

}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{

}


//*----------------------------------------------------------------------------
//* Function Name       : FIQHandler
//* Object              : Interrupt Handler called by the FIQ interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void FIQHandler(void)
{

}


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

char *msg[CDC_MSG_SIZE];
char *cmd[CDC_MSG_SIZE];
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

    cdcMessageObj.length = pCDC.Read(&pCDC, (char *)cdcMessageObj.data, CDC_MSG_SIZE);

    return cdcMessageObj;
}

static void InitPWM(void)
{
    // enable PWM peripherals on PA0, PA1, PA2
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA0_PWM0, 0);
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA1_PWM1, 0);
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA2_PWM2, 0);

    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

    // enable PWM clock in PMC
    AT91F_PWMC_CfgPMC();

    // disable PWM channels 0, 1, 2
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    pwmFreqSet(0, 8000);
    pwmFreqSet(1, 8000);
    pwmFreqSet(2, 8000);

    pwmDutySet_u8(0, 1);
    pwmDutySet_u8(1, 1);
    pwmDutySet_u8(2, 1);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);

    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    // enable PWM channels 0, 1, 2
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
}

static void InitPIO(void)
{
    // U2, U3, U4, U8 PIO configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);   //U2, U3, U4, U8 EN
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7); //U2, U3, U4, U8 EN - HIGH to disable

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);  //leftUSSonar  PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);  //rightUSSonar PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);  //usSensors    PWM2

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

    // US Sonars configuration
    // US Sonars interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5); //leftUSSonarTrigger
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6); //rightUSSonarTrigger

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

    //Input channel 1 configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U12 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);  //U12 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U12 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D

    // U7 Digital Pot control
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U7 INC
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U7 U/D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U7 CS

    // U9, U10, U11, U13, U14 Shift registers control pins
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //U9, U10, U11, U13, U14 ST_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);   //U9, U10, U11 DS
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //U9, U10, U11 U7 SH_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U13, U14 DS
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U13, U14 SH_CP

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);

    // disable all pull-ups
    AT91C_BASE_PIOA->PIO_PPUDR = ~0;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ0Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ1Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) FIQHandler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
}


//*----------------------------------------------------------------------------
//* Function Name       : DeviceInit
//* Object              : Device peripherals initialization
///*----------------------------------------------------------------------------
static void DeviceInit(void)
{
    InitPIO();

    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4 << 8) | (unsigned int)(0xA5 << 24);

    // Set-up the PIO
    // First, enable the clock of the PIO and set the LEDs in output
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );

    // Initialize USB device
    AT91FUSBOpen();
    // Wait for the end of enumeration
    while (!pCDC.IsConfigured(&pCDC));

    InitPIO();
    InitADC();
    InitPWM();
    InitIRQ();
}


/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;

    DeviceInit();

    while (1)
    {
        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "DEMOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"PARALLEL EXECUTION MANIPULATOR DEMO\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
        }
    }

    return 0; /* never reached */
}

