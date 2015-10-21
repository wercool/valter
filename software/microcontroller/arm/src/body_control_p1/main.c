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

#define FIQ_INTERRUPT_LEVEL 0

unsigned int shiftRegistersStates[24] = { [0 ... 23] = 0 };

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

struct _AT91S_CDC pCDC;
//*----------------------------------------------------------------------------
//* function     AT91F_USB_Open
//*              This function Open the USB device
//*----------------------------------------------------------------------------
void AT91FUSBOpen(void)
{
    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1;

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

    for (int r = 0; r < CDC_MSG_SIZE; r++)
    {
        cdcMessageObj.data[r] = '\0';
    }

    cdcMessageObj.length = pCDC.Read(&pCDC, (char *) cdcMessageObj.data, CDC_MSG_SIZE);

    return cdcMessageObj;
}

static void InitPWM(void)
{
    // enable PWM peripherals on PA0, PA1, PA2
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA0_PWM0, 0);
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA1_PWM1, 0);
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA2_PWM2, 0);

    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, 0, AT91C_PA7_PWM3);
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA7_PWM3, 0);

    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    //AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);

    AT91F_PMC_EnablePeriphClock(AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

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
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);   //U2 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);   //U2 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);   //U2 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);  //U2 D

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //5.5V Power Source switch

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);  //left  arm yaw PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);  //right arm yaw PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);  //body pitch    PWM2

    //Shift registers initialization
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //ST_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //SH_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);   //DS

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //12V enable left arm
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);  //12V enable right arm

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //head yaw step
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //head pitch step

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);

    // disable all pull-ups
    AT91C_BASE_PIOA->PIO_PPUDR = ~0;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void (*)(void)) IRQ0Handler);AT91F_AIC_EnableIt
    (AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void (*)(void)) IRQ1Handler);AT91F_AIC_EnableIt
    (AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void (*)(void)) FIQHandler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
}    //*----------------------------------------------------------------------------
//* Function Name       : DeviceInit
//* Object              : Device peripherals initialization
///*----------------------------------------------------------------------------
static void DeviceInit(void)
{
    InitPIO();

    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4 << 8) | (unsigned int) (0xA5 << 24);

    // Set-up the PIO
    // First, enable the clock of the PIO and set the LEDs in output
    AT91F_PMC_EnablePeriphClock(AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

    // Initialize USB device
    AT91FUSBOpen();
    // Wait for the end of enumeration
    while (!pCDC.IsConfigured(&pCDC))
        ;

    InitPIO();
    InitADC();
    InitPWM();
    //InitIRQ();
}

// Specific functions

void setShiftRegisterBit(unsigned char index, unsigned int value)
{
    /*
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //ST_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //SH_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);   //DS
    */

    shiftRegistersStates[index] = value;

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    for (signed char i = 23; i >= 0; i--)
    {
        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        if (shiftRegistersStates[i] == 1)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
        }
        delay_us(50);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        delay_us(50);
    }
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
}

void setShiftRegister()
{
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    for (signed char i = 23; i >= 0; i--)
    {
        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        if (shiftRegistersStates[i] == 1)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
        }
        delay_us(50);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        delay_us(50);
    }
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
}

/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;

    unsigned int channel = 0;
    unsigned char channelReadings = 0;
    unsigned int channelReading = 0;

    unsigned char bodyPitchReadings = 0;
    unsigned int bodyPitchReading = 0;
    unsigned int bodyPitchDriveDuty = 1;
    unsigned char bodyPitchDirection = 0; //0 - up, 1 - down
    unsigned char bodyPitchINaIndex = 13;
    unsigned char bodyPitchINbIndex = 14;

    unsigned char rightArmYawReadings = 0;
    unsigned int rightArmYawReading = 0;
    unsigned int rightArmYawDriveDuty = 1;
    unsigned char rightArmYawDirection = 0; //0 - open, 1 - close
    unsigned char rightArmYawINaIndex = 11;
    unsigned char rightArmYawINbIndex = 12;

    unsigned char leftArmYawReadings = 0;
    unsigned int leftArmYawReading = 0;
    unsigned int leftArmYawDriveDuty = 1;
    unsigned char leftArmYawDirection = 0; //0 - open, 1 - close
    unsigned char leftArmYawINaIndex = 9;
    unsigned char leftArmYawINbIndex = 10;

    unsigned char head24VSwitchIndex = 0;
    unsigned char rightArm24VSwitchIndex = 1;
    unsigned char leftArm24VSwitchIndex = 2;
    unsigned char bodyLEDSwitchIndex = 15;
    unsigned char leftAccumulatorSwitchIndex = 16;
    unsigned char rightAccumulatorSwitchIndex = 17;

    unsigned char headYawENIndex = 3;
    unsigned char headYawDIRIndex = 4;
    unsigned char headYawREFIndex = 4;
    unsigned char headYawDirection = 0; //0 - left, 1 - right
    unsigned int  headYawStepTime = 1000;

    unsigned char headPitchENIndex = 6;
    unsigned char headPitchDIRIndex = 7;
    unsigned char headPitchREFIndex = 8;
    unsigned char headPitchDirection = 0; //0 - down, 1 - up
    unsigned int  headPitchStepTime = 1000;

    DeviceInit();

    shiftRegistersStates[headYawENIndex] = 1;
    shiftRegistersStates[headYawREFIndex] = 1;
    shiftRegistersStates[headPitchENIndex] = 1;
    shiftRegistersStates[headPitchREFIndex] = 1;

    setShiftRegister();

    while (1)
    {

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            sprintf((char *) msg, "MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#");

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *) msg, "BODY-CONTROL-P1\n");
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            //Body pitch current
            if (strcmp((char*) cmdParts, "CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 0;
                continue;
            }
            //Right arm current
            if (strcmp((char*) cmdParts, "CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 1;
                continue;
            }
            //Left arm current
            if (strcmp((char*) cmdParts, "CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 4;
                continue;
            }
            //turret initial position
            if (strcmp((char*) cmdParts, "CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U2   D
                channel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNELREADSTART") == 0)
            {
                channelReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "CHANNELREADSTOP") == 0)
            {
                channelReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "5V5SOURCEON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
                continue;
            }
            if (strcmp((char*) cmdParts, "5V5SOURCEOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHREADING") == 0)
            {
                bodyPitchReading = getValueChannel4();
                sprintf((char *) msg, "BODY PITCH POSITION: %u\n", bodyPitchReading);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHREADINGSSTART") == 0)
            {
                bodyPitchReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHREADINGSSTOP") == 0)
            {
                bodyPitchReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWREADING") == 0)
            {
                rightArmYawReading = getValueChannel5();
                sprintf((char *) msg, "RIGHT ARM YAW POSITION: %u\n", rightArmYawReading);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWREADINGSSTART") == 0)
            {
                rightArmYawReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWREADINGSSTOP") == 0)
            {
                rightArmYawReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWREADING") == 0)
            {
                leftArmYawReading = getValueChannel6();
                sprintf((char *) msg, "LEFT ARM YAW POSITION: %u\n", leftArmYawReading);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWREADINGSSTART") == 0)
            {
                leftArmYawReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWREADINGSSTOP") == 0)
            {
                leftArmYawReadings = 0;
                continue;
            }
            //SETBODYPITCHDRIVEDUTY#1
            //SETBODYPITCHDRIVEDUTY#10
            //SETBODYPITCHDRIVEDUTY#20
            //SETBODYPITCHDRIVEDUTY#30
            //SETBODYPITCHDRIVEDUTY#40
            //SETBODYPITCHDRIVEDUTY#50
            //SETBODYPITCHDRIVEDUTY#60
            //SETBODYPITCHDRIVEDUTY#70
            //SETBODYPITCHDRIVEDUTY#80
            //SETBODYPITCHDRIVEDUTY#90
            //SETBODYPITCHDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETBODYPITCHDRIVEDUTY") == 0)
            {
                bodyPitchDriveDuty = atoi(strtok(NULL, "#"));
                pwmDutySetPercent(2, bodyPitchDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHGETUP") == 0)
            {
                setShiftRegisterBit(bodyPitchINaIndex, 1);
                setShiftRegisterBit(bodyPitchINbIndex, 0);
                bodyPitchDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHGETDOWN") == 0)
            {
                setShiftRegisterBit(bodyPitchINaIndex, 0);
                setShiftRegisterBit(bodyPitchINbIndex, 1);
                bodyPitchDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYPITCHSTOP") == 0)
            {
                setShiftRegisterBit(bodyPitchINaIndex, 0);
                setShiftRegisterBit(bodyPitchINbIndex, 0);
                bodyPitchDriveDuty = 1;
                continue;
            }
            //SETRIGHTARMYAWDRIVEDUTY#1
            //SETRIGHTARMYAWDRIVEDUTY#10
            //SETRIGHTARMYAWDRIVEDUTY#20
            //SETRIGHTARMYAWDRIVEDUTY#30
            //SETRIGHTARMYAWDRIVEDUTY#40
            //SETRIGHTARMYAWDRIVEDUTY#50
            //SETRIGHTARMYAWDRIVEDUTY#60
            //SETRIGHTARMYAWDRIVEDUTY#70
            //SETRIGHTARMYAWDRIVEDUTY#80
            //SETRIGHTARMYAWDRIVEDUTY#90
            //SETRIGHTARMYAWDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETRIGHTARMYAWDRIVEDUTY") == 0)
            {
                rightArmYawDriveDuty = atoi(strtok(NULL, "#"));
                pwmDutySetPercent(1, bodyPitchDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWOPEN") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 1);
                setShiftRegisterBit(rightArmYawINbIndex, 0);
                rightArmYawDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWCLOSE") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 0);
                setShiftRegisterBit(rightArmYawINbIndex, 1);
                rightArmYawDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWSTOP") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 0);
                setShiftRegisterBit(rightArmYawINbIndex, 0);
                rightArmYawDriveDuty = 1;
                continue;
            }
            //SETLEFTARMYAWDRIVEDUTY#1
            //SETLEFTARMYAWDRIVEDUTY#10
            //SETLEFTARMYAWDRIVEDUTY#20
            //SETLEFTARMYAWDRIVEDUTY#30
            //SETLEFTARMYAWDRIVEDUTY#40
            //SETLEFTARMYAWDRIVEDUTY#50
            //SETLEFTARMYAWDRIVEDUTY#60
            //SETLEFTARMYAWDRIVEDUTY#70
            //SETLEFTARMYAWDRIVEDUTY#80
            //SETLEFTARMYAWDRIVEDUTY#90
            //SETLEFTARMYAWDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETLEFTARMYAWDRIVEDUTY") == 0)
            {
                leftArmYawDriveDuty = atoi(strtok(NULL, "#"));
                pwmDutySetPercent(0, bodyPitchDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWOPEN") == 0)
            {
                setShiftRegisterBit(leftArmYawINaIndex, 1);
                setShiftRegisterBit(leftArmYawINbIndex, 0);
                leftArmYawDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWCLOSE") == 0)
            {
                setShiftRegisterBit(leftArmYawINaIndex, 0);
                setShiftRegisterBit(leftArmYawINbIndex, 1);
                leftArmYawDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARMYAWSTOP") == 0)
            {
                setShiftRegisterBit(leftArmYawINaIndex, 0);
                setShiftRegisterBit(leftArmYawINbIndex, 0);
                leftArmYawDriveDuty = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARM12VENABLE") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARM12VDISABLE") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARM12VENABLE") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARM12VDISABLE") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEAD24VON") == 0)
            {
                setShiftRegisterBit(head24VSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEAD24VOFF") == 0)
            {
                setShiftRegisterBit(head24VSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARM24VON") == 0)
            {
                setShiftRegisterBit(rightArm24VSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARM24VOFF") == 0)
            {
                setShiftRegisterBit(rightArm24VSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARM24VON") == 0)
            {
                setShiftRegisterBit(leftArm24VSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTARM24VOFF") == 0)
            {
                setShiftRegisterBit(leftArm24VSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYLEDON") == 0)
            {
                setShiftRegisterBit(bodyLEDSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "BODYLEDOFF") == 0)
            {
                setShiftRegisterBit(bodyLEDSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATORON") == 0)
            {
                setShiftRegisterBit(leftAccumulatorSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATOROFF") == 0)
            {
                setShiftRegisterBit(leftAccumulatorSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATORON") == 0)
            {
                setShiftRegisterBit(rightAccumulatorSwitchIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATOROFF") == 0)
            {
                setShiftRegisterBit(rightAccumulatorSwitchIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWENABLE") == 0)
            {
                setShiftRegisterBit(headYawENIndex, 0);
                setShiftRegisterBit(headYawREFIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWDISABLE") == 0)
            {
                setShiftRegisterBit(headYawREFIndex, 1);
                setShiftRegisterBit(headYawENIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWLEFT") == 0)
            {
                setShiftRegisterBit(headYawDIRIndex, 0);
                headYawDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWRIGHT") == 0)
            {
                setShiftRegisterBit(headYawDIRIndex, 1);
                headYawDirection = 1;
                continue;
            }
            //HEADYAWSTEPTIME#200
            //HEADYAWSTEPTIME#300
            //HEADYAWSTEPTIME#400
            //HEADYAWSTEPTIME#500
            //HEADYAWSTEPTIME#600
            //HEADYAWSTEPTIME#700
            //HEADYAWSTEPTIME#800
            //HEADYAWSTEPTIME#900
            //HEADYAWSTEPTIME#1000
            //HEADYAWSTEPTIME#1500
            //HEADYAWSTEPTIME#2000
            //HEADYAWSTEPTIME#5000
            if (strcmp((char*) cmdParts, "HEADYAWSTEPTIME") == 0)
            {
                headYawStepTime = atoi(strtok( NULL, "#" ));
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHENABLE") == 0)
            {
                setShiftRegisterBit(headPitchENIndex, 0);
                setShiftRegisterBit(headPitchREFIndex, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHDISABLE") == 0)
            {
                setShiftRegisterBit(headPitchREFIndex, 1);
                setShiftRegisterBit(headPitchENIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHDOWN") == 0)
            {
                setShiftRegisterBit(headPitchDIRIndex, 0);
                headPitchDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHUP") == 0)
            {
                setShiftRegisterBit(headPitchDIRIndex, 1);
                headPitchDirection = 1;
                continue;
            }
            //HEADPITCHSTEPTIME#200
            //HEADPITCHSTEPTIME#300
            //HEADPITCHSTEPTIME#400
            //HEADPITCHSTEPTIME#500
            //HEADPITCHSTEPTIME#600
            //HEADPITCHSTEPTIME#700
            //HEADPITCHSTEPTIME#800
            //HEADPITCHSTEPTIME#900
            //HEADPITCHSTEPTIME#1000
            //HEADPITCHSTEPTIME#1500
            //HEADPITCHSTEPTIME#2000
            //HEADPITCHSTEPTIME#5000
            if (strcmp((char*) cmdParts, "HEADPITCHSTEPTIME") == 0)
            {
                headPitchStepTime = atoi(strtok( NULL, "#" ));
                continue;
            }
        }

        if (channelReadings)
        {
            channelReading = getValueChannel0();
            sprintf((char *) msg, "CHANNEL [%u]: %u\n", channel, channelReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }
        if (bodyPitchReadings)
        {
            bodyPitchReading = getValueChannel4();
            sprintf((char *) msg, "BODY PITCH POSITION: %u\n", bodyPitchReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }
        if (rightArmYawReadings)
        {
            rightArmYawReading = getValueChannel5();
            sprintf((char *) msg, "RIGHT ARM YAW POSITION: %u\n", rightArmYawReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }
        if (leftArmYawReadings)
        {
            leftArmYawReading = getValueChannel6();
            sprintf((char *) msg, "LEFT ARM YAW POSITION: %u\n", leftArmYawReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }

        return 0; /* never reached */
    }
}
