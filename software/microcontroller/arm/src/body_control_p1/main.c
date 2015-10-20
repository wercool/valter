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

#define FIQ_INTERRUPT_LEVEL 0

unsigned int forearmCCWLimit = 0;
unsigned int forearmCWLimit = 0;

//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ0Handler(void)
{

}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ1Handler
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
    //reserved
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
    /////////////////////////////////////////////////////////////////////// PWM3 output on PA7 used as PIO
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, 0, AT91C_PA7_PWM3);
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA7_PWM3, 0);

    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    //AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

    // enable PWM clock in PMC
    AT91F_PWMC_CfgPMC();

    // disable PWM channels 0, 1, 2
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    //AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    pwmFreqSet(0, 8000);
    pwmFreqSet(1, 8000);
    pwmFreqSet(2, 8000);
    //pwmFreqSet(3, 8000);

    pwmDutySet_u8(0, 1);
    pwmDutySet_u8(1, 1);
    pwmDutySet_u8(2, 1);
    //pwmDutySet_u8(3, 1);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
    //AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 3, AT91C_PWMC_CHID3);

    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    //AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    // enable PWM channels 0, 1, 2
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);
}

static void InitPIO(void)
{
    // U1 PIO (hand sensors) configuration (perephirals on valter/electronics/circuits/arm_interface.dsn)
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);   //U2 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //U2 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //U2 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);   //U2 D


    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);




    // disable all pull-ups
    AT91C_BASE_PIOA->PIO_PPUDR = ~0;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ0Handler);
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ1Handler);
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) FIQHandler);
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
    //InitIRQ();
}

// Specific functions


/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;
    unsigned int armSensorsChannel = 0;
    unsigned char armSensorsReadings = 0;
    unsigned int armSensorsReading = 0;

    unsigned int sensorsChannel = 0;
    unsigned char sensorsReadings = 0;
    unsigned int sensorsReading = 0;

    unsigned int forearmLiftUpDriveDuty = 1;
    unsigned int armLiftUpDriveDuty = 1;
    unsigned int limbLiftUpDriveDuty = 1;

    unsigned int forearmLiftUpPositionReading = 0;
    unsigned int armLiftUpPositionReading = 0;
    unsigned int limbLiftUpPositionReading = 0;

    //directions
    unsigned char forearmLiftUpDirection = 2;
    unsigned char armLiftUpDirection = 2;
    //unsigned char limbLiftUpDirection = 0;

    //thresholds
    unsigned int forearmLiftUpPositionThreshold = 90;
    unsigned int armLiftUpPositionThreshold = 750;
    //unsigned int limbLiftUpPositionReading = 0;

    unsigned char forearmLiftUpPositionReadings = 0;
    unsigned char armLiftUpPositionReadings = 0;
    unsigned char limbLiftUpPositionReadings = 0;

    unsigned int forearmRollSpeed = 100;

    unsigned char forearmDirection = 2; // 0 - CW, 1 - CCW
    unsigned char forearmCWLimitReadings = 0;
    unsigned char forearmCCWLimitReadings = 0;
    unsigned int forearmSteps = 0;
    unsigned char forearmStepsReadings = 0;

    //hand
    //watchers
    unsigned char handYawWatch = 0;
    unsigned char handPitchWatch = 0;
    //directions
    unsigned char handYawDirection = 0;     //0 - CW, 1 - CCW
    unsigned char handPitchDirection = 0;   //0 - CW, 1 - CCW
    //thresholds
    unsigned int handYawCWThreshold = 10;
    unsigned int handYawCCWThreshold = 950;
    unsigned int handPitchCWThreshold = 130;
    unsigned int handPitchCCWThreshold = 585;
    //positions
    unsigned int handYawPosition = 0;
    unsigned int handPitchPosition = 0;

    //forearm
    //watchers
    unsigned char forearmYawWatch = 0;
    //directions
    unsigned char forearmYawDirection = 0;      //0 - CW, 1 - CCW
    //thresholds
    unsigned int forearmYawCWThreshold = 15;
    unsigned int forearmYawCCWThreshold = 970;
    //positions
    unsigned int forearmYawPosition = 0;

    DeviceInit();

    //forearm roll off
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);

    while (1)
    {
        //updating logic variables
        forearmCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA20) ? 0 : 1;
        forearmCCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA30)? 0 : 1;

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"BODY-CONTROL-P1\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 15;
                continue;
            }
    }

    return 0; /* never reached */
}

