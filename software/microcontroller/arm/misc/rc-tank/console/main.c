#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Board.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "pwm.h"
#include "aic.h"
#include "async.h"
#include "delay.h"
#include "util_math.h"

#include "thresholds.h"

#define FIQ_INTERRUPT_LEVEL     0
#define TIMER0_INTERRUPT_LEVEL  1
#define TIMER1_INTERRUPT_LEVEL  1


/*-----------------*/
/* Clock Selection */
/*-----------------*/
#define TC_CLKS                  0x7
#define TC_CLKS_MCK2             0x0
#define TC_CLKS_MCK8             0x1
#define TC_CLKS_MCK32            0x2
#define TC_CLKS_MCK128           0x3
#define TC_CLKS_MCK1024          0x4


//*------------------------- Internal Function --------------------------------
//*----------------------------------------------------------------------------
//* Function Name       : AT91F_TC_Open
//* Object              : Initialize Timer Counter Channel and enable is clock
//* Input Parameters    : <tc_pt> = TC Channel Descriptor Pointer
//*                       <mode> = Timer Counter Mode
//*                     : <TimerId> = Timer peripheral ID definitions
//* Output Parameters   : None
//*----------------------------------------------------------------------------
void AT91F_TC_Open ( AT91PS_TC TC_pt, unsigned int Mode, unsigned int TimerId)
{
    unsigned int dummy;

    // First, enable the clock of the TIMER
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1<< TimerId ) ;

    // Disable the clock and the interrupts
    TC_pt->TC_CCR = AT91C_TC_CLKDIS ;
    TC_pt->TC_IDR = 0xFFFFFFFF ;

    // Clear status bit
        dummy = TC_pt->TC_SR;
    // Suppress warning variable "dummy" was set but never used
        dummy = dummy;
    // Set the Mode of the Timer Counter
    TC_pt->TC_CMR = Mode ;

    // Enable the clock
    TC_pt->TC_CCR = AT91C_TC_CLKEN ;
}

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

__ramfunc void SoftIQRHandlerTC0(void)
{
    volatile AT91PS_TC pTC = AT91C_BASE_TC0; /* pointer to timer channel 0 register structure */
    volatile unsigned int dummy; /* temporary */

    /* read TC0 Status Register to clear interrupt */
    dummy = pTC->TC_SR;
    //* Suppress warning variable "dummy" was set but never used
    dummy = dummy;

}

__ramfunc void SoftIQRHandlerTC1(void)
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
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, 0, AT91C_PA7_PWM3);

    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

    // enable PWM clock in PMC
    AT91F_PWMC_CfgPMC();

    // disable PWM channels 0, 1, 2
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    pwmFreqSet(0, 10);
    pwmFreqSet(1, 10);
    pwmFreqSet(2, 10);
    pwmFreqSet(3, 10);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 3, AT91C_PWMC_CHID3);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
}

static void InitPIO(void)
{
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);


    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);    //PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);    //PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);    //PWM2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //PWM3
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);

    // Platform wheels encoder interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);    // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);    // IRQ1

    //FIQ
    //AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);    // FIQ
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);      // AD2

    // disable all pull-ups
    AT91C_BASE_PIOA->PIO_PPUDR = ~0;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ0Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ1Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    //AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) FIQHandler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);


    // Init timer1 (USE THIS!!!)
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC1;         // Enable peripheral clock
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;             // Disable TC clock
    AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;                  // Disable interrupts
    unsigned int dummy = AT91C_BASE_TC1->TC_SR;               // Clear status register
    dummy = dummy;
    AT91C_BASE_TC1->TC_CMR = TC_CLKS_MCK32 | AT91C_TC_CPCTRG;    // Set mode

    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN ;             // Enable the Clock counter
    AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;

    AT91C_BASE_AIC->AIC_IDCR = 0x1 << AT91C_ID_TC1;
    AT91C_BASE_AIC->AIC_SVR[AT91C_ID_TC1] = (unsigned int) SoftIQRHandlerTC1;
    AT91C_BASE_AIC->AIC_SMR[AT91C_ID_TC1] = AT91C_AIC_SRCTYPE_EXT_LOW_LEVEL | 4;
    AT91C_BASE_AIC->AIC_ICCR = 0x1 << AT91C_ID_TC1;
    AT91C_BASE_AIC->AIC_IECR = 0x1 << AT91C_ID_TC1;

    AT91C_BASE_TC1->TC_RC = 100000;
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG;

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
    //InitADC();
    //InitPWM();
    //InitIRQ();

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);

}


/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;

    DeviceInit();

    pwmDutySetPercent(0, 1);
    pwmDutySetPercent(1, 1);
    pwmDutySetPercent(2, 1);
    pwmDutySetPercent(3, 1);

    while (1)
    {

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            //sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            //pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"RC-Tank\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "ONOFF") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                delay_ms(1000);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                continue;
            }
            if (strcmp((char*) cmdParts, "LF") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
                continue;
            }
            if (strcmp((char*) cmdParts, "RF") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
                continue;
            }
            if (strcmp((char*) cmdParts, "LB") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
                continue;
            }
            if (strcmp((char*) cmdParts, "RB") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                continue;
            }
            if (strcmp((char*) cmdParts, "LS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
                continue;
            }
            if (strcmp((char*) cmdParts, "RS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                continue;
            }
            //Turret left start
            if (strcmp((char*) cmdParts, "TL") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
                continue;
            }
            if (strcmp((char*) cmdParts, "TR") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
                continue;
            }
            if (strcmp((char*) cmdParts, "TS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
                continue;
            }
        }
    }

    return 0; /* never reached */
}

