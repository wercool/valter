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
    //forearm.cw.limit
}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ1Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{
    //forearm.ccw.limit
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
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);   //U1 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //U1 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //U1 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);   //U1 D

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //U2 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //U2 D

    //hand pitch and yaw drive
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);    //U3.IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U3.IN3
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //U3.ENA
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);   //U3.ENB

    //forearm yaw drive
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U4.IN3
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U4.ENB

    //forearm liftup
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);   //U4.IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);   //U4.IN2

    //arm liftup
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);   //U5.IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);   //U5.IN2

    //limb liftup
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U6.IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U6.IN2

    //forearm.roll cnc driver
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);

    //arm leds
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);  // FIQ (reserved)

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA17);  // ADC Channel 0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA18);  // ADC Channel 1

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);



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
    unsigned char forearmLiftUpDirection = 0;
    unsigned char armLiftUpDirection = 0;
    unsigned char limbLiftUpDirection = 0;

    //thresholds
    unsigned int forearmLiftUpPositionThreshold = 90;
    unsigned int armLiftUpPositionThreshold = 750;
    //unsigned int limbLiftUpPositionReading = 0;

    unsigned char forearmLiftUpPositionReadings = 0;
    unsigned char armLiftUpPositionReadings = 0;
    unsigned char limbLiftUpPositionReadings = 0;

    unsigned int forearmRollSpeed = 100;

    unsigned char forearmDirection = 0; // 0 - CW, 1 - CCW
    unsigned char forearmCWLimitReadings = 0;
    unsigned char forearmCCWLimitReadings = 0;
    unsigned int forearmSteps = 0;
    unsigned char forearmStepsReadings = 0;

    //hand
    //watchers
    unsigned char handYawWatch = 0;
    //directions
    unsigned char handYawDirection = 0; //0 - CW, 1 - CCW
    //thresholds
    unsigned int handYawCWThreshold = 10;
    unsigned int handYawCCWThreshold = 950;
    //positions
    unsigned int handYawPosition = 0;


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
                sprintf((char *)msg,"ARM-CONTROL-LEFT\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //U1   D
                armSensorsChannel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMCH15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U1   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U1   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U1   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);    //U1   D
                armSensorsChannel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMSENSORSREADSTART") == 0)
            {
                armSensorsReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMSENSORSREADSTOP") == 0)
            {
                armSensorsReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETARMSENSORREADING") == 0)
            {
                armSensorsReading = getValueChannel0();
                sprintf((char *)msg,"ARM SENSORS CHANNEL [%u]: %u\n", armSensorsChannel, armSensorsReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            //hand.yaw position
            if (strcmp((char*) cmdParts, "SENSORCH0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 0;
                continue;
            }
            //hand.pitch position
            if (strcmp((char*) cmdParts, "SENSORCH1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 1;
                continue;
            }
            //forearm.yaw position
            if (strcmp((char*) cmdParts, "SENSORCH2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 2;
                continue;
            }
            //limb.liftup current
            if (strcmp((char*) cmdParts, "SENSORCH3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 3;
                continue;
            }
            //arm.liftup current
            if (strcmp((char*) cmdParts, "SENSORCH4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 4;
                continue;
            }
            //forearm.liftup current
            if (strcmp((char*) cmdParts, "SENSORCH5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 5;
                continue;
            }
            //forearm.yaw current
            if (strcmp((char*) cmdParts, "SENSORCH6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 6;
                continue;
            }
            //hand.yaw current
            if (strcmp((char*) cmdParts, "SENSORCH7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 7;
                continue;
            }
            //hand.pitch current
            if (strcmp((char*) cmdParts, "SENSORCH8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);    //U2   D
                sensorsChannel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORREADSTART") == 0)
            {
                sensorsReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORREADSTOP") == 0)
            {
                sensorsReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETSENSORREADING") == 0)
            {
                sensorsReading = getValueChannel1();
                sprintf((char *)msg,"SENSOR CHANNEL [%u]: %u\n", sensorsChannel, sensorsReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            //L298 U3
            //HAND YAW
            if (strcmp((char*) cmdParts, "HANDYAWCW") == 0)
            {
                handYawDirection = 0;
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWCCW") == 0)
            {
                handYawDirection = 1;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWON") == 0)
            {
                armSensorsReadings = 0;
                handYawWatch = 1;
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWOFF") == 0)
            {
                handYawWatch = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                continue;
            }
            //HAND PITCH
            if (strcmp((char*) cmdParts, "HANDPITCHCCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);
                continue;
            }
            //L298 U4
            //FOREARM YAW
            if (strcmp((char*) cmdParts, "FOREARMYAWCCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMYAWCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMYAWON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMYAWOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
                continue;
            }
            //L298 U4
            //FOREARM LIFT UP
            //SETFOREARMLIFTUPDUTY#1
            //SETFOREARMLIFTUPDUTY#50
            //SETFOREARMLIFTUPDUTY#60
            //SETFOREARMLIFTUPDUTY#70 -- normal
            //SETFOREARMLIFTUPDUTY#80
            //SETFOREARMLIFTUPDUTY#90
            //SETFOREARMLIFTUPDUTY#100
            if (strcmp((char*) cmdParts, "SETFOREARMLIFTUPDUTY") == 0)
            {
                forearmLiftUpDriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(0, forearmLiftUpDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMLIFTUPDOWN") == 0)
            {
                forearmLiftUpPositionReading = getValueChannel4();
                if (forearmLiftUpPositionReading <= forearmLiftUpPositionThreshold)
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //U4.IN1
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //U4.IN2

                    sprintf((char *)msg,"FOREARM LIFTUP POSITION LIMIT [T: %u, V: %u] REACHED\n", forearmLiftUpPositionThreshold, forearmLiftUpPositionReading);
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                    delay_ms(100);
                }
                else
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //U4.IN1
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //U4.IN2
                    forearmLiftUpDirection = 0;
                }
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMLIFTUPUP") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //U4.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //U4.IN2
                forearmLiftUpDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMLIFTUPSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //U4.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //U4.IN2
                continue;
            }
            //L298 U5
            //ARM LIFT UP
            //SETARMLIFTUPDUTY#50
            //SETARMLIFTUPDUTY#60
            //SETARMLIFTUPDUTY#70
            //SETARMLIFTUPDUTY#80 -- normal
            //SETARMLIFTUPDUTY#90
            //SETARMLIFTUPDUTY#100
            if (strcmp((char*) cmdParts, "SETARMLIFTUPDUTY") == 0)
            {
                armLiftUpDriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, armLiftUpDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMLIFTUPDOWN") == 0)
            {
                armLiftUpPositionReading = getValueChannel5();
                if (armLiftUpPositionReading >= armLiftUpPositionThreshold)
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //U5.IN1
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U5.IN2

                    sprintf((char *)msg,"ARM LIFTUP POSITION LIMIT [T: %u, V: %u] REACHED\n", armLiftUpPositionThreshold, armLiftUpPositionReading);
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                    delay_ms(100);
                }
                else
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //U5.IN1
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U5.IN2
                    armLiftUpDirection = 0;
                }
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMLIFTUPUP") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //U5.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U5.IN2
                armLiftUpDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMLIFTUPSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //U5.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U5.IN2
                continue;
            }
            //L298 U6
            //LIMB LIFT UP
            //SETLIMBLIFTUPDUTY#10
            //SETLIMBLIFTUPDUTY#20
            //SETLIMBLIFTUPDUTY#30
            //SETLIMBLIFTUPDUTY#40
            //SETLIMBLIFTUPDUTY#50
            //SETLIMBLIFTUPDUTY#60
            //SETLIMBLIFTUPDUTY#70 -- normal
            //SETLIMBLIFTUPDUTY#80
            //SETLIMBLIFTUPDUTY#90
            //SETLIMBLIFTUPDUTY#100
            if (strcmp((char*) cmdParts, "SETLIMBLIFTUPDUTY") == 0)
            {
                limbLiftUpDriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(2, limbLiftUpDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                continue;
            }
            if (strcmp((char*) cmdParts, "LIMBLIFTUPUP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);  //U6.IN1
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);    //U6.IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "LIMBLIFTUPDOWN") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);  //U6.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);    //U6.IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "LIMBLIFTUPSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);  //U6.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);  //U6.IN2
                continue;
            }
            //forearm liftup position
            if (strcmp((char*) cmdParts, "FOREARMLIFTUPPOSITIONSTART") == 0)
            {
                forearmLiftUpPositionReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMLIFTUPPOSITIONSTOP") == 0)
            {
                forearmLiftUpPositionReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETFOREARMLIFTUPPOSITIONREADING") == 0)
            {
                forearmLiftUpPositionReading = getValueChannel4();
                sprintf((char *)msg,"FOREARM LIFTUP POSITION: %u\n", forearmLiftUpPositionReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            //arm liftup position
            if (strcmp((char*) cmdParts, "ARMLIFTUPPOSITIONSTART") == 0)
            {
                armLiftUpPositionReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMLIFTUPPOSITIONSTOP") == 0)
            {
                armLiftUpPositionReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETARMLIFTUPPOSITIONREADING") == 0)
            {
                armLiftUpPositionReading = getValueChannel5();
                sprintf((char *)msg,"ARM LIFTUP POSITION: %u\n", armLiftUpPositionReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            //limb liftup position
            if (strcmp((char*) cmdParts, "LIMBLIFTUPPOSITIONSTART") == 0)
            {
                limbLiftUpPositionReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LIMBLIFTUPPOSITIONSTOP") == 0)
            {
                limbLiftUpPositionReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLIMBLIFTUPPOSITIONREADING") == 0)
            {
                limbLiftUpPositionReading = getValueChannel6();
                sprintf((char *)msg,"LIMB LIFTUP POSITION: %u\n", limbLiftUpPositionReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            //forearm roll
            if (strcmp((char*) cmdParts, "FOREARMROLLON") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMROLLOFF") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMROLLCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
                forearmDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMROLLCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
                forearmDirection = 1;
                continue;
            }
            //FOREARMROLLSPEED#200
            //FOREARMROLLSPEED#300
            //FOREARMROLLSPEED#400
            //FOREARMROLLSPEED#500
            //FOREARMROLLSPEED#600
            //FOREARMROLLSPEED#700
            //FOREARMROLLSPEED#800
            //FOREARMROLLSPEED#900
            //FOREARMROLLSPEED#1000
            //FOREARMROLLSPEED#5000
            if (strcmp((char*) cmdParts, "FOREARMROLLSPEED") == 0)
            {
                forearmRollSpeed = atoi(strtok( NULL, "#" ));
                continue;
            }
            //FOREARMROLLSTEPS#1
            //FOREARMROLLSTEPS#5
            //FOREARMROLLSTEPS#10
            //FOREARMROLLSTEPS#20
            //FOREARMROLLSTEPS#30
            //FOREARMROLLSTEPS#40
            //FOREARMROLLSTEPS#50
            //FOREARMROLLSTEPS#70
            //FOREARMROLLSTEPS#100
            //FOREARMROLLSTEPS#200
            //FOREARMROLLSTEPS#500
            //FOREARMROLLSTEPS#1000
            //FOREARMROLLSTEPS#1500
            if (strcmp((char*) cmdParts, "FOREARMROLLSTEPS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
                unsigned int forearmRollSteps = atoi(strtok( NULL, "#" ));
                for(int s = 0; s < forearmRollSteps; s++)
                {
                    forearmCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA20) ? 0 : 1;
                    forearmCCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA30)? 0 : 1;
                    if (forearmDirection == 0 && (forearmCWLimit == 1 || forearmSteps >= 2000))
                    {
                        sprintf((char *)msg,"FOREARM CW LIMIT REACHED %s\n", (forearmSteps >= 2000) ? "(STEPS LIMIT)" : "");
                        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                        delay_ms(100);
                        break;
                    }
                    else if (forearmDirection == 1 && forearmCCWLimit == 1)
                    {
                        sprintf((char *)msg,"FOREARM CCW LIMIT REACHED\n");
                        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                        delay_ms(100);
                        break;
                    }
                    else
                    {
                        delay_us(forearmRollSpeed);
                        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
                        delay_us(forearmRollSpeed);
                        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
                        if (forearmDirection == 0)
                        {
                            forearmSteps++;
                        }
                        else
                        {
                            forearmSteps--;
                        }
                    }
                }
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMROLLINIT") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
                forearmDirection = 1;
                forearmCCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA30)? 0 : 1;
                while (forearmCCWLimit == 0)
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
                    delay_us(200);
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
                    delay_us(200);
                    forearmCCWLimit = AT91F_PIO_IsInputSet(AT91C_BASE_PIOA, AT91C_PIO_PA30)? 0 : 1;
                }
                forearmSteps = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMSTEPSREADINGSSTART") == 0)
            {
                forearmStepsReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMSTEPSREADINGSSTOP") == 0)
            {
                forearmStepsReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETFOREARMSTEPS") == 0)
            {
                sprintf((char *)msg,"FOREARM STEPS: %u\n", forearmSteps);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            if (strcmp((char*) cmdParts, "GETFOREARMCCWLIMIT") == 0)
            {
                sprintf((char *)msg,"FOREARM CCW LIMIT: %u\n", forearmCCWLimit);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            if (strcmp((char*) cmdParts, "GETFOREARMCWLIMIT") == 0)
            {
                sprintf((char *)msg,"FOREARM CW LIMIT: %u\n", forearmCWLimit);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMCWLIMITREADINGSSTART") == 0)
            {
                forearmCWLimitReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMCWLIMITREADINGSSTOP") == 0)
            {
                forearmCWLimitReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMCCWLIMITREADINGSSTART") == 0)
            {
                forearmCCWLimitReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "FOREARMCCWLIMITREADINGSSTOP") == 0)
            {
                forearmCCWLimitReadings = 0;
                continue;
            }

            //arm leds
            if (strcmp((char*) cmdParts, "ARMLEDSON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
            if (strcmp((char*) cmdParts, "ARMLEDSOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
        }
        //check thresholds
        if (armLiftUpDirection == 0)
        {
            armLiftUpPositionReading = getValueChannel5();
            if (armLiftUpPositionReading >= armLiftUpPositionThreshold)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //U5.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U5.IN2

                sprintf((char *)msg,"ARM LIFTUP POSITION LIMIT [T: %u, V: %u] REACHED\n", armLiftUpPositionThreshold, armLiftUpPositionReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
            }
        }
        if (forearmLiftUpDirection == 0)
        {
            forearmLiftUpPositionReading = getValueChannel4();
            if (forearmLiftUpPositionReading <= forearmLiftUpPositionThreshold)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //U4.IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //U4.IN2

                sprintf((char *)msg,"FOREARM LIFTUP POSITION LIMIT [T: %u, V: %u] REACHED\n", forearmLiftUpPositionThreshold, forearmLiftUpPositionReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
            }
        }
        if (handYawWatch == 1)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D

            handYawPosition = getValueChannel1();
            if ((handYawDirection == 0 && handYawPosition <= handYawCWThreshold) || (handYawDirection == 1 && handYawPosition >= handYawCCWThreshold))
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                handYawWatch = 0;
            }
        }
        //readings
        if (armSensorsReadings)
        {
            armSensorsReading = getValueChannel0();
            sprintf((char *)msg,"ARM SENSORS CHANNEL [%u]: %u\n", armSensorsChannel, armSensorsReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (sensorsReadings)
        {
            sensorsReading = getValueChannel1();
            sprintf((char *)msg,"SENSOR CHANNEL [%u]: %u\n", sensorsChannel, sensorsReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (forearmLiftUpPositionReadings)
        {
            forearmLiftUpPositionReading = getValueChannel4();
            sprintf((char *)msg,"FOREARM LIFTUP POSITION: %u\n", forearmLiftUpPositionReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (armLiftUpPositionReadings)
        {
            armLiftUpPositionReading = getValueChannel5();
            sprintf((char *)msg,"ARM LIFTUP POSITION: %u\n", armLiftUpPositionReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (limbLiftUpPositionReadings)
        {
            limbLiftUpPositionReading = getValueChannel6();
            sprintf((char *)msg,"LIMB LIFTUP POSITION: %u\n", limbLiftUpPositionReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (forearmCWLimitReadings)
        {
            sprintf((char *)msg,"FOREARM CW LIMIT: %u\n", forearmCWLimit);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (forearmCCWLimitReadings)
        {
            sprintf((char *)msg,"FOREARM CCW LIMIT: %u\n", forearmCCWLimit);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (forearmStepsReadings)
        {
            sprintf((char *)msg,"FOREARM STEPS: %u\n", forearmSteps);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
    }

    return 0; /* never reached */
}

