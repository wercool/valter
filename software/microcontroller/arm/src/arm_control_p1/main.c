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

unsigned char thresholdSigma = 2;
unsigned char graspCurrentMeasureStep = 0;
unsigned char graspCurrentSUM = 0;


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
    /////////////////////////////////////////////////////////////////////// TODO check and set correctly!
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, 0, AT91C_PA7_PWM3);
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA7_PWM3, 0);

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

    pwmFreqSet(0, 8000);
    pwmFreqSet(1, 8000);
    pwmFreqSet(2, 60); // set by default to control Camera Servo
    pwmFreqSet(3, 8000);

    pwmDutySet_u8(0, 1);
    pwmDutySet_u8(1, 1);
    pwmDutySet_u8(2, 1);
    pwmDutySet_u8(3, 1);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 3, AT91C_PWMC_CHID3);

    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

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

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);  // FIQ (reserved)

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA17);  // ADC Channel 0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA18);  // ADC Channel 1

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);

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
    InitIRQ();
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

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"ARM-CONTROL-P1\n");
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
            //hand.yaw
            if (strcmp((char*) cmdParts, "SENSORCH0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 0;
                continue;
            }
            //hand.pitch
            if (strcmp((char*) cmdParts, "SENSORCH1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 1;
                continue;
            }
            //forearm.yaw
            if (strcmp((char*) cmdParts, "SENSORCH2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "SENSORCH7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);     //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //U2   D
                sensorsChannel = 7;
                continue;
            }
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
            if (strcmp((char*) cmdParts, "HANDPITCHCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDPITCHOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);
                continue;
            }
            if (strcmp((char*) cmdParts, "HANDYAWOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);
                continue;
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
    }

    return 0; /* never reached */
}

