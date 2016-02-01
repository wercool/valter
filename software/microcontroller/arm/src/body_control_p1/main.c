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
#include "serial.h"

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

    AT91F_PMC_EnablePeriphClock(AT91C_BASE_PMC, ( 1 << AT91C_ID_PIOA ) | ( 1 << AT91C_ID_US0 ));

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
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //shift registers enable

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

    uart0_init();
}

// Specific functions

void setShiftRegisterBit(unsigned char idx, unsigned int value)
{
    /*
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);  //ST_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);  //SH_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);   //DS
    */

    shiftRegistersStates[idx] = value;

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
    unsigned int  bodyPitchReading = 0;
    unsigned int  bodyPitchDriveDuty = 1;
    unsigned char bodyPitchDirection = 0; //0 - up, 1 - down
    unsigned char bodyPitchINaIndex = 13;
    unsigned char bodyPitchINbIndex = 14;

    unsigned char rightArmYawReadings = 0;
    unsigned int  rightArmYawReading = 0;
    unsigned int  rightArmYawDriveDuty = 1;
    unsigned char rightArmYawDirection = 0; //0 - open, 1 - close
    unsigned char rightArmYawINaIndex = 11;
    unsigned char rightArmYawINbIndex = 12;

    unsigned char leftArmYawReadings = 0;
    unsigned int  leftArmYawReading = 0;
    unsigned int  leftArmYawDriveDuty = 1;
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
    unsigned char headYawREFIndex = 5;
    unsigned char headYawDirection = 0; //0 - left, 1 - right
    unsigned int  headYawStepTime = 2500;
    unsigned char headYawHoldStep = 0;
    unsigned char headYawReadings = 0;
    unsigned int  headYawReading = 0;

    unsigned char headPitchENIndex = 6;
    unsigned char headPitchDIRIndex = 7;
    unsigned char headPitchREFIndex = 8;
    unsigned char headPitchDirection = 0; //1 - down, 0 - up
    unsigned int  headPitchStepTime = 2500;
    unsigned char headPitchHoldStep = 0;
    unsigned char headPitchReadings = 0;
    unsigned int  headPitchReading = 0;

    unsigned char kinect1Power12VIndex = 18;
    unsigned char kinect2Power12VIndex = 19;

    unsigned char usb1Index = 22;
    unsigned char usb2Index = 23;

    unsigned char wifi12VPowerIndex = 20;

    char uart0Buff[32];
    unsigned int uart0BuffCnt = 1;

    DeviceInit();

    shiftRegistersStates[headYawENIndex] = 1;
    shiftRegistersStates[headYawREFIndex] = 1;
    shiftRegistersStates[headPitchENIndex] = 1;
    shiftRegistersStates[headPitchREFIndex] = 1;

    setShiftRegister();

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);//SHIFTREGENABLE

    for (unsigned c = 0; c < 32; c++)
    {
        uart0Buff[c] = '\0';
    }

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
            if (strcmp((char*) cmdParts, "SHIFTREGENABLE") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                continue;
            }
            if (strcmp((char*) cmdParts, "SHIFTREGDISABLE") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                continue;
            }
            if (strcmp((char*) cmdParts, "SHIFTREGRESET") == 0)
            {
                for (unsigned char s = 0; s < 23; s++)
                {
                    shiftRegistersStates[s] = 0;
                }
                shiftRegistersStates[headYawENIndex] = 1;
                shiftRegistersStates[headYawREFIndex] = 1;
                shiftRegistersStates[headPitchENIndex] = 1;
                shiftRegistersStates[headPitchREFIndex] = 1;
                setShiftRegister();
                continue;
            }
            //Left arm current
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
            //Body pitch current
            if (strcmp((char*) cmdParts, "CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 2;
                continue;
            }
            //left accumulator
            if (strcmp((char*) cmdParts, "CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //U2   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U2   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 3;
                continue;
            }
            //right accumulator
            if (strcmp((char*) cmdParts, "CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //U2   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //U2   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U2   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //U2   D
                channel = 4;
                continue;
            }
            //body pitch initial position
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
            //max 1000 (up)
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
            //min 40 (opened)
            //max 904 (closed)
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
            //max 584 (closed)
            //min 0 (opened)
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
            //SETRIGHTARMYAWDRIVEDUTY#90
            //SETRIGHTARMYAWDRIVEDUTY#95
            //SETRIGHTARMYAWDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETRIGHTARMYAWDRIVEDUTY") == 0)
            {
                rightArmYawDriveDuty = atoi(strtok(NULL, "#"));
                pwmDutySetPercent(1, rightArmYawDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWOPEN") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 0);
                setShiftRegisterBit(rightArmYawINbIndex, 1);
                rightArmYawDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWCLOSE") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 1);
                setShiftRegisterBit(rightArmYawINbIndex, 0);
                rightArmYawDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTARMYAWSTOP") == 0)
            {
                setShiftRegisterBit(rightArmYawINaIndex, 0);
                setShiftRegisterBit(rightArmYawINbIndex, 0);
                rightArmYawDriveDuty = 1;
                continue;
            }
            //SETLEFTARMYAWDRIVEDUTY#90
            //SETLEFTARMYAWDRIVEDUTY#95
            //SETLEFTARMYAWDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETLEFTARMYAWDRIVEDUTY") == 0)
            {
                leftArmYawDriveDuty = atoi(strtok(NULL, "#"));
                pwmDutySetPercent(0, leftArmYawDriveDuty);
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
            if (strcmp((char*) cmdParts, "HEADYAWHOLDSTEPON") == 0)
            {
                headYawHoldStep = 1;
                setShiftRegisterBit(headYawREFIndex, 0);
                continue;
            }
            //!!!!
            //USE HOLD STEP OFF (as working solution)!
            if (strcmp((char*) cmdParts, "HEADYAWHOLDSTEPOFF") == 0)
            {
                headYawHoldStep = 0;
                setShiftRegisterBit(headYawREFIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWENABLE") == 0)
            {
                setShiftRegisterBit(headYawENIndex, 0);
                if (headYawHoldStep)
                {
                    setShiftRegisterBit(headYawREFIndex, 0);
                }
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWDISABLE") == 0)
            {
                setShiftRegisterBit(headYawENIndex, 1);
                setShiftRegisterBit(headYawREFIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWLEFT") == 0)
            {
                setShiftRegisterBit(headYawDIRIndex, 1);
                headYawDirection = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWRIGHT") == 0)
            {
                setShiftRegisterBit(headYawDIRIndex, 0);
                headYawDirection = 1;
                continue;
            }
            //HEADYAWSTEPTIME#2000
            //HEADYAWSTEPTIME#2500
            //HEADYAWSTEPTIME#3000
            //HEADYAWSTEPTIME#5000
            //HEADYAWSTEPTIME#10000
            //HEADYAWSTEPTIME#50000
            if (strcmp((char*) cmdParts, "HEADYAWSTEPTIME") == 0)
            {
                headYawStepTime = atoi(strtok( NULL, "#" ));
                continue;
            }
            //HEADYAWSTEPS#1
            //HEADYAWSTEPS#5
            //HEADYAWSTEPS#10
            //HEADYAWSTEPS#20
            //HEADYAWSTEPS#30
            //HEADYAWSTEPS#40
            //HEADYAWSTEPS#50
            //HEADYAWSTEPS#100
            //HEADYAWSTEPS#200
            //HEADYAWSTEPS#300
            //HEADYAWSTEPS#350
            //HEADYAWSTEPS#400
            //HEADYAWSTEPS#500
            //HEADYAWSTEPS#600
            //HEADYAWSTEPS#700
            // << 600 >>
            if (strcmp((char*) cmdParts, "HEADYAWSTEPS") == 0)
            {
                if (headYawHoldStep)
                {
                    setShiftRegisterBit(headYawREFIndex, 0);
                }
                unsigned int headYawSteps = atoi(strtok( NULL, "#" ));
                for(int s = 0; s < headYawSteps; s++)
                {
                    delay_us(headYawStepTime);
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
                    delay_us(headYawStepTime);
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
                    headYawReading = getValueChannel1();
                    sprintf((char *) msg, "HEAD YAW POSITION: %u\n", headYawReading);
                    pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                }
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWREADING") == 0)
            {
                headYawReading = getValueChannel1();
                sprintf((char *) msg, "HEAD YAW POSITION: %u\n", headYawReading);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            //max 970 (left)
            //center 885
            //min 540 (right)
            if (strcmp((char*) cmdParts, "HEADYAWREADINGSSTART") == 0)
            {
                headYawReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADYAWREADINGSTOP") == 0)
            {
                headYawReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHHOLDSTEPON") == 0)
            {
                headPitchHoldStep = 1;
                setShiftRegisterBit(headPitchREFIndex, 0);
                continue;
            }
            //!!!!
            //USE HOLD STEP OFF (as working solution)!
            if (strcmp((char*) cmdParts, "HEADPITCHHOLDSTEPOFF") == 0)
            {
                headPitchHoldStep = 0;
                setShiftRegisterBit(headPitchREFIndex, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHENABLE") == 0)
            {
                setShiftRegisterBit(headPitchENIndex, 0);
                if (headPitchHoldStep)
                {
                    setShiftRegisterBit(headPitchREFIndex, 0);
                }
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
                setShiftRegisterBit(headPitchDIRIndex, 1);
                headPitchDirection = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHUP") == 0)
            {
                setShiftRegisterBit(headPitchDIRIndex, 0);
                headPitchDirection = 0;
                continue;
            }
            //HEADPITCHSTEPTIME#1500
            //HEADPITCHSTEPTIME#2000
            //HEADPITCHSTEPTIME#2500
            //HEADPITCHSTEPTIME#5000
            if (strcmp((char*) cmdParts, "HEADPITCHSTEPTIME") == 0)
            {
                headPitchStepTime = atoi(strtok( NULL, "#" ));
                continue;
            }
            //HEADPITCHSTEPS#1
            //HEADPITCHSTEPS#5
            //HEADPITCHSTEPS#10
            //HEADPITCHSTEPS#20
            //HEADPITCHSTEPS#30
            //HEADPITCHSTEPS#40
            //HEADPITCHSTEPS#50
            //HEADPITCHSTEPS#100
            //HEADPITCHSTEPS#200
            //HEADPITCHSTEPS#400
            //HEADPITCHSTEPS#500
            //HEADPITCHSTEPS#600
            //HEADPITCHSTEPS#700
            // << 700 >>
            if (strcmp((char*) cmdParts, "HEADPITCHSTEPS") == 0)
            {
                if (headPitchHoldStep)
                {
                    setShiftRegisterBit(headPitchREFIndex, 0);
                }
                unsigned int headPitchSteps = atoi(strtok( NULL, "#" ));
                for(int s = 0; s < headPitchSteps; s++)
                {
                    delay_us(headPitchStepTime);
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
                    delay_us(headPitchStepTime);
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
                    headPitchReading = getValueChannel2();
                    sprintf((char *) msg, "HEAD PITCH POSITION: %u\n", headPitchReading);
                    pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                }
                continue;
            }
            //max 940 (up)
            //min 25 (down)
            if (strcmp((char*) cmdParts, "HEADPITCHREADING") == 0)
            {
                headPitchReading = getValueChannel2();
                sprintf((char *) msg, "HEAD PITCH POSITION: %u\n", headPitchReading);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHREADINGSSTART") == 0)
            {
                headPitchReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "HEADPITCHREADINGSTOP") == 0)
            {
                headPitchReadings = 0;
                continue;
            }
            //UART0
            //UART0#message
            if (strcmp((char*) cmdParts, "UART0") == 0)
            {
                char* uart0msg = strtok( NULL, "#" );
                sprintf((char *) msg, "UART0 ->: %s\n", uart0msg);
                pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));

                uart0_putc(0xFF);
                uart0_putc(0xFF);
                uart0_putc(0xFF);
                uart0_putc(0xFF);

                uart0_puts((char *)uart0msg);
            }

            if (strcmp((char*) cmdParts, "USB1ON") == 0)
            {
                setShiftRegisterBit(usb1Index, 1);
                delay_ms(500);
                setShiftRegisterBit(usb1Index, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "USB2ON") == 0)
            {
                setShiftRegisterBit(usb2Index, 1);
                delay_ms(500);
                setShiftRegisterBit(usb2Index, 0);
                continue;
            }
            if (strcmp((char*) cmdParts, "KINECT1POWER12VON") == 0)
            {
                setShiftRegisterBit(kinect1Power12VIndex, 1);
            }
            if (strcmp((char*) cmdParts, "KINECT1POWER12VOFF") == 0)
            {
                setShiftRegisterBit(kinect1Power12VIndex, 0);
            }
            if (strcmp((char*) cmdParts, "KINECT2POWER12VON") == 0)
            {
                setShiftRegisterBit(kinect2Power12VIndex, 1);
            }
            if (strcmp((char*) cmdParts, "KINECT2POWER12VOFF") == 0)
            {
                setShiftRegisterBit(kinect2Power12VIndex, 0);
            }
            if (strcmp((char*) cmdParts, "WIFIPOWER12VON") == 0)
            {
                setShiftRegisterBit(wifi12VPowerIndex, 1);
            }
            if (strcmp((char*) cmdParts, "WIFIPOWER12VOFF") == 0)
            {
                setShiftRegisterBit(wifi12VPowerIndex, 0);
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
        if (headYawReadings)
        {
            headYawReading = getValueChannel1();
            sprintf((char *) msg, "HEAD YAW POSITION: %u\n", headYawReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }
        if (headPitchReadings)
        {
            headPitchReading = getValueChannel2();
            sprintf((char *) msg, "HEAD PITCH POSITION: %u\n", headPitchReading);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            delay_ms(100);
        }

        //UART0
        while (uart0_kbhit())
        {
            char uart0Char = uart0_getc();
            if (uart0Char != 0xFF)
            {
                if (uart0Char != '*')
                {
                    if (uart0BuffCnt < 32)
                    {
                        uart0Buff[uart0BuffCnt++] = uart0Char;
                    }
                    else
                    {
                        for (unsigned c = 0; c < 32; c++)
                        {
                            uart0Buff[c] = '\0';
                        }
                        uart0BuffCnt = 1;
                    }
                }
                else
                {
                    uart0Buff[0] = '*';
                }
            }
        }
        if (uart0Buff[0] == '*')
        {
            for (unsigned c = 0; c < 31; c++)
            {
                uart0Buff[c] = uart0Buff[c + 1];
            }
            sprintf((char *) msg, "UART0 <-: %s\n", uart0Buff);
            pCDC.Write(&pCDC, (char *) msg, strlen((char *) msg));
            for (unsigned c = 0; c < 32; c++)
            {
                uart0Buff[c] = '\0';
            }
            uart0BuffCnt = 1;
        }

    }
    return 0; /* never reached */
}
