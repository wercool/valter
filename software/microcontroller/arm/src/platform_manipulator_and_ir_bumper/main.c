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
    // U4 PIO (input1) configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //U4 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //U4 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //U4 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);   //U4 D
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);    //link1DrivePWM                         PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);    //link2DrivePWM                         PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);    //cameraServoPWM, IRBumper36kHz         PWM2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //gripperRotateDrive                    PWM3
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

    // Platform wheels encoder interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);    // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);    // IRQ1

    //24V DCDC Converter
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);   //DCDC24VENABLEON / DCDC24VENABLEOFF
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);

    //link1 and link2 INa / INb configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);    //link1DriveINa
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //link1DriveINb
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //link2DriveINa
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);   //link2DriveINb

    //gripper rotate IN1, IN2, IN3, IN4, ENA, ENB configuration U3
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);    //gripperRotate IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //gripperRotate IN2

    //gripper tilt IN1, IN2, ENA configuration U2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //gripperTilt IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //gripperTilt IN2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

    //gripper position IN3, IN4, ENB configuration U2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //gripperPosition IN3
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //gripperPosition IN4
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);

    // IR bumber interrurp FIQ
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);    // FIQ
    // IR bumper set IR radiation OFF
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
    

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

    unsigned int input1Channel = 0;
    unsigned int input1Readings = 0;
    unsigned int input1Reading = 0;
    unsigned int cameraServoDuty = 1;
    unsigned int link1DriveDuty = 1;
    unsigned int link2DriveDuty = 1;
    unsigned int gripperRotateDriveDuty = 1;
    unsigned int parallelDemo = 0;
    unsigned int serialDemo = 0;

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

            if (strcmp((char*) cmdParts, "DCDC24VENABLEON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                sprintf((char *)msg,"24V DCDC Voltage Regulator is ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DCDC24VENABLEOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                sprintf((char *)msg,"24V DCDC Voltage Regulator is OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
                input1Channel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
                input1Channel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTINPUT1READINGS") == 0)
            {
                input1Readings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPINPUT1READINGS") == 0)
            {
                input1Readings = 0;
                continue;
            }
            //SETCAMERASERVODUTY#5
            //SETCAMERASERVODUTY#8
            //SETCAMERASERVODUTY#20
            //SETCAMERASERVODUTY#30
            //SETCAMERASERVODUTY#40
            if (strcmp((char*) cmdParts, "SETCAMERASERVODUTY") == 0)
            {
                cameraServoDuty = atoi(strtok( NULL, "#" ));

                AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA2_PWM2, 0);
                AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
                pwmFreqSet(2, 60);
                pwmDutySet_u8(2, cameraServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLECAMERASERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                sprintf((char *)msg,"CAMERA SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //SETLINK1DRIVEDUTY#1
            //SETLINK1DRIVEDUTY#5
            //SETLINK1DRIVEDUTY#10
            //SETLINK1DRIVEDUTY#20
            //SETLINK1DRIVEDUTY#30
            //SETLINK1DRIVEDUTY#40
            //SETLINK1DRIVEDUTY#50
            //SETLINK1DRIVEDUTY#60
            //SETLINK1DRIVEDUTY#70
            //SETLINK1DRIVEDUTY#80
            //SETLINK1DRIVEDUTY#90
            //SETLINK1DRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETLINK1DRIVEDUTY") == 0)
            {
                link1DriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(0, link1DriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK1GETUP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //link1DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK1GETDOWN") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);    //link1DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK1STOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
                continue;
            }
            //SETLINK2DRIVEDUTY#1
            //SETLINK2DRIVEDUTY#5
            //SETLINK2DRIVEDUTY#10
            //SETLINK2DRIVEDUTY#20
            //SETLINK2DRIVEDUTY#30
            //SETLINK2DRIVEDUTY#40
            //SETLINK2DRIVEDUTY#50
            //SETLINK2DRIVEDUTY#60
            //SETLINK2DRIVEDUTY#70
            //SETLINK2DRIVEDUTY#80
            //SETLINK2DRIVEDUTY#90
            //SETLINK2DRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETLINK2DRIVEDUTY") == 0)
            {
                link2DriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, link2DriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK2GETUP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //link2DriveINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //link2DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK2GETDOWN") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //link2DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK2STOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //link2DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
                continue;
            }
            //SETGRIPPERROTATEDRIVEDUTY#1
            //SETGRIPPERROTATEDRIVEDUTY#5
            //SETGRIPPERROTATEDRIVEDUTY#10
            //SETGRIPPERROTATEDRIVEDUTY#20
            //SETGRIPPERROTATEDRIVEDUTY#30
            //SETGRIPPERROTATEDRIVEDUTY#40
            //SETGRIPPERROTATEDRIVEDUTY#50
            //SETGRIPPERROTATEDRIVEDUTY#60
            //SETGRIPPERROTATEDRIVEDUTY#70
            //SETGRIPPERROTATEDRIVEDUTY#80
            //SETGRIPPERROTATEDRIVEDUTY#90
            //SETGRIPPERROTATEDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETGRIPPERROTATEDRIVEDUTY") == 0)
            {
                gripperRotateDriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(3, gripperRotateDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERROTATECW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //gripperRotate IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERROTATECCW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);    //gripperRotate IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERROTATESTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTGETUP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //gripperTilt IN2
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTGETDOWN") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //gripperTilt IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //gripperTilt ENA
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONCLOSE") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9); //gripperPosition IN3
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //gripperPosition IN4
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONOPEN") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //gripperPosition IN3
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //gripperPosition IN3
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOPSTART") == 0)
            {
                parallelDemo = 1;
                sprintf((char *)msg,"PARALLEL EXECUTION MANIPULATOR DEMO STARTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOPSTOP") == 0)
            {
                parallelDemo = 0;
                sprintf((char *)msg,"PARALLEL EXECUTION MANIPULATOR DEMO STOPPED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOSSTART") == 0)
            {
                serialDemo = 1;
                sprintf((char *)msg,"SERIAL EXECUTION MANIPULATOR DEMO STARTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOSSTOP") == 0)
            {
                serialDemo = 0;
                sprintf((char *)msg,"SERIAL EXECUTION MANIPULATOR DEMO STOPPED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
        }
        if (input1Readings)
        {
            input1Reading = getValueChannel6();
            sprintf((char *)msg,"INPUT1 CHANNEL [%u]: %u\n", input1Channel, input1Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
    }

    return 0; /* never reached */
}

