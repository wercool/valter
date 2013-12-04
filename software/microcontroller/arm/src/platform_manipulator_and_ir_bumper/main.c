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

unsigned char irBumperCnt = 0;

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
    irBumperCnt++;
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

    // U5, U6 PIO (irBumperCh) configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);   //U5, U6 ENABLE
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);   //disable U5, U6


    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);    //link1DrivePWM                         PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);    //link2DrivePWM                         PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);    //cameraServoPWM                        PWM2
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

void stopAllManipulatorDrives()
{
    pwmDutySetPercent(0, 1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb

    pwmDutySetPercent(1, 1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //link2DriveINa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //gripperTilt ENA

    pwmDutySetPercent(3, 1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //gripperPosition IN3
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //gripperPosition ENB
}


int getLink1Position()
{
    return getValueChannel4();
}

int getLink2Position()
{
    return getValueChannel5();
}

int getLink3Position()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
    input1Channel = 0;
    return getValueChannel6();
}

int getGripperRotation()
{
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
    input1Channel = 1;
    return getValueChannel6();
}

int getGripperGraspPosition()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
    input1Channel = 2;
    return getValueChannel6();
}

unsigned int getDynamicDutyVal(unsigned int trend, int trendShift, unsigned char maxDuty, unsigned char minDuty)
{
    unsigned int duty = round((((double)(absv(trend - trendShift))) / (double) 100) * maxDuty);
    if (trend > absv(trendShift))
    {
        duty = round((double)duty * (double)((double)50 / (double)trend));
    }
    if (duty < minDuty)
    {
        return minDuty;
    }
    if (duty > maxDuty)
    {
        return maxDuty;
    }
    return duty;
}


unsigned int setLink1Position(unsigned int goalPosition)
{
    unsigned int curPosition = getLink1Position();
    unsigned int positionTrend = round(((double)absv(goalPosition - curPosition) / (double)link1Range) * 100);
    if (absv((signed int) (curPosition - goalPosition)) > thresholdSigma)
    {
        unsigned char positionVector = (goalPosition > curPosition) ? 1 : 0;
        unsigned int duty = getDynamicDutyVal(positionTrend, -50, 32, 25);
        pwmDutySetPercent(0, duty);
        AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
        if (positionVector == 0)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //link1DriveINb
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);    //link1DriveINa
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
        }
    }
    else
    {
        pwmDutySetPercent(0, 1);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
    }
    sprintf((char *)msg,"LINK1 TREND[%d] CUR[%d]->GOAL[%d]\n", positionTrend, curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

unsigned int setLink2Position(unsigned int goalPosition)
{
    unsigned int curPosition = getLink2Position();
    unsigned int positionTrend = round(((double)absv(goalPosition - curPosition) / (double)link2Range) * 100);
    if (absv((signed int) (curPosition - goalPosition)) > thresholdSigma)
    {
        unsigned char positionVector = (goalPosition > curPosition) ? 0 : 1;
        unsigned int duty = getDynamicDutyVal(positionTrend, -50, 32, 22);
        pwmDutySetPercent(1, duty);
        AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
        if (positionVector == 0)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //link2DriveINa
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //link2DriveINb
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //link2DriveINa
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
        }
    }
    else
    {
        pwmDutySetPercent(1, 1);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //link2DriveINa
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
    }
    sprintf((char *)msg,"LINK2 TREND[%d] CUR[%d]->GOAL[%d]\n", positionTrend, curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

unsigned int setLink3Position(unsigned int goalPosition)
{
    unsigned int curPosition = getLink3Position();
    if (absv((signed int) (curPosition - goalPosition)) > thresholdSigma)
    {
        unsigned char positionVector = (goalPosition > curPosition) ? 1 : 0;
        if (positionVector == 0)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //gripperTilt IN2
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //gripperTilt IN1
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
        }
    }
    else
    {
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //gripperTilt ENA
    }
    sprintf((char *)msg,"LINK3 (GRIPPER TILIT) CUR[%d]->GOAL[%d]\n", curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

unsigned int setGripperRotation(unsigned int goalPosition)
{
    unsigned int curPosition = getGripperRotation();
    unsigned int positionTrend = round(((double)absv(goalPosition - curPosition) / (double)gripperRotateRange) * 100);
    unsigned char positionVector = (goalPosition > curPosition) ? 0 : 1;
    if (absv((signed int) (curPosition - goalPosition)) > thresholdSigma)
    {
        unsigned int duty = getDynamicDutyVal(positionTrend, -50, 32, 22);
        pwmDutySetPercent(3, duty);
        AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);
        if (positionVector == 0)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //gripperRotate IN2
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);    //gripperRotate IN1
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
        }
    }
    else
    {
        pwmDutySetPercent(3, 1);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
    }
    sprintf((char *)msg,"GRIPPER ROTATION TREND[%d] CUR[%d]->GOAL[%d]\n", positionTrend, curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

unsigned int setGripperGrasp(unsigned int goalPosition)
{
    unsigned int curPosition = getGripperGraspPosition();
    if (absv((signed int) (curPosition - goalPosition)) > thresholdSigma)
    {
        unsigned char positionVector = (goalPosition > curPosition) ? 1 : 0;
        if (positionVector == 0)
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9); //gripperPosition IN3
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //gripperPosition IN4
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //gripperPosition IN3
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
        }
    }
    else
    {
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //gripperPosition IN3
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //gripperPosition ENB
    }
    sprintf((char *)msg,"GRIPPER GRASP CUR[%d]->GOAL[%d]\n", curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

unsigned char isObjectDetected()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);    //U4   C
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
    input1Channel = 6;
    unsigned int detectionCouter = 0;
    unsigned int objectDetection = 0;
    while (detectionCouter < 50)
    {
        objectDetection += getValueChannel6();
        delay_us(500);
        detectionCouter++;
    }
    objectDetection = round((double)objectDetection / (double)50);
    if (objectDetection < gripperObjectDetectedThreshold)
    {
        sprintf((char *)msg,"OBJECT DETECTED\n");
        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
        return 1;
    }
    else
    {
        sprintf((char *)msg,"NO OBJECT DETECTED\n");
        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
        return 0;
    }
}

unsigned char isObjectGrasped(unsigned int force)
{
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U4   A
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);    //U4   B
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U4   D
    input1Channel = 3;
    unsigned int senseCouter = 0;
    unsigned int objectSense = 0;
    while (senseCouter < 10)
    {
        objectSense += getValueChannel6();
        delay_us(250);
        senseCouter++;
    }
    objectSense = round((double)objectSense / (double)10);
    if (objectSense > force)
    {
        sprintf((char *)msg,"OBJECT GRASPED\n");
        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
        return 1;
    }
    else
    {
        sprintf((char *)msg,"NO OBJECT GRASPED\n");
        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
        return 0;
    }
}

void stopLink1Drive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
}

void stopLink2Drive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //link2DriveINa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
}

void stopLink3Drive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //gripperTilt ENA
}

void stopGripperRotationDrive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
}

void stopGripperGraspDrive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //gripperPosition IN3
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //gripperPosition ENB
}

unsigned int inspectGraspDriveCurrent()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U4   A
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U4   B
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U4   C
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);    //U4   D
    input1Channel = 8;

    graspCurrentSUM += getValueChannel6();
    graspCurrentMeasureStep++;

    if (graspCurrentMeasureStep > 10)
    {
        unsigned int inspectedValue = round((double)graspCurrentSUM / (double) 10);

        graspCurrentSUM = 0;
        graspCurrentMeasureStep = 0;

        if (inspectedValue > 24)
        {
            //overload
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
}

/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    struct cdcMessage cdcMessageObj;

    unsigned char input1Readings = 0;
    unsigned int input1Reading = 0;
    unsigned int cameraServoDuty = 1;
    unsigned int link1DriveDuty = 1;
    unsigned int link2DriveDuty = 1;
    unsigned int gripperRotateDriveDuty = 1;
    unsigned char serialDemo = 0;
    unsigned char link1Readings = 0;
    unsigned char link2Readings = 0;
    unsigned int link1Reading = 0;
    unsigned int link2Reading = 0;
    unsigned int link3Reading = 0;
    unsigned int gripperRotation = 0;

    unsigned int link1StaticVal = 0;
    unsigned int link2StaticVal = 0;
    unsigned int link3StaticVal = 0;
    unsigned int gripperGraspStaticVal = 0;
    unsigned int gripperRotationStaticVal = 0;

    unsigned char link1StaticMode = 0;
    unsigned char link2StaticMode = 0;
    unsigned char link3StaticMode = 0;
    unsigned char gripperGraspStaticMode = 0;
    unsigned char gripperRotationStaticMode = 0;

    unsigned char isObjectDetectedFlag = 0;
    unsigned char isObjectGraspedFlag = 0;

    unsigned int gripperGraspPosition = 0;

    unsigned int serialDemoVec = 0;

    unsigned int irBumperInitialized = 0;
    unsigned int irBumperDutyPercent = 1;
    unsigned int irBumperFreq = 38000;
    unsigned int irBumperChannel = 0;

    DeviceInit();

    while (1)
    {
        //Set links state
        if (link1StaticMode)
        {
            unsigned int curVal = setLink1Position(link1StaticVal);
            if (absv((signed int) (curVal - link1StaticVal)) <= thresholdSigma)
            {
                link1StaticMode = 0;
                stopLink1Drive();
            }
        }
        if (link2StaticMode)
        {
            unsigned int curVal = setLink2Position(link2StaticVal);
            if (absv((signed int) (curVal - link2StaticVal)) <= thresholdSigma)
            {
                link2StaticMode = 0;
                stopLink2Drive();
            }
        }
        if (link3StaticMode)
        {
            unsigned int curVal = setLink3Position(link3StaticVal);
            if (absv((signed int) (curVal - link3StaticVal)) <= thresholdSigma)
            {
                link3StaticMode = 0;
                stopLink3Drive();
            }
        }
        if (gripperRotationStaticMode)
        {
            setGripperRotation(gripperRotationStaticVal);
        }
        if (gripperGraspStaticMode)
        {
            unsigned int curVal = setGripperGrasp(gripperGraspStaticVal);
            if (absv((signed int) (curVal - gripperGraspStaticVal)) <= thresholdSigma)
            {
                gripperGraspStaticMode = 0;
                stopGripperGraspDrive();
            }
        }

        if (isObjectDetectedFlag)
        {
            isObjectDetected();
        }

        if (isObjectGraspedFlag)
        {
            isObjectGrasped(875);
        }

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"PLATFORM-MANIPULATOR-AND-IR-BUMPER\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
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
            if (strcmp((char*) cmdParts, "READLINK1START") == 0)
            {
                link1Readings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "READLINK1STOP") == 0)
            {
                link1Readings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "READLINK2START") == 0)
            {
                link2Readings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "READLINK2STOP") == 0)
            {
                link2Readings = 0;
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

                irBumperInitialized = 0;
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
            //SETLINK1POSITION#0
            //SETLINK1POSITION#100
            //SETLINK1POSITION#200
            //SETLINK1POSITION#300
            //SETLINK1POSITION#400
            //SETLINK1POSITION#500
            //SETLINK1POSITION#600
            //SETLINK1POSITION#700
            //SETLINK1POSITION#800
            //SETLINK1POSITION#900
            //SETLINK1POSITION#1023 INITIAL
            if (strcmp((char*) cmdParts, "SETLINK1POSITION") == 0)
            {
                link1StaticMode = 1;
                link1StaticVal = atoi(strtok( NULL, "#" ));
                continue;
            }
            //SETLINK1DRIVEDUTY#1
            //SETLINK1DRIVEDUTY#5
            //SETLINK1DRIVEDUTY#10
            //SETLINK1DRIVEDUTY#15
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
                link1StaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //link1DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //link1DriveINb
                continue;
            }
            //SETLINK2POSITION#135 INITIAL
            //SETLINK2POSITION#200
            //SETLINK2POSITION#300
            //SETLINK2POSITION#400
            //SETLINK2POSITION#500
            //SETLINK2POSITION#600
            //SETLINK2POSITION#700
            //SETLINK2POSITION#800
            //SETLINK2POSITION#930
            if (strcmp((char*) cmdParts, "SETLINK2POSITION") == 0)
            {
                link2StaticMode = 1;
                link2StaticVal = atoi(strtok( NULL, "#" ));
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
            if (strcmp((char*) cmdParts, "LINK2GETDOWN") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //link2DriveINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //link2DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK2GETUP") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //link2DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LINK2STOP") == 0)
            {
                link2StaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //link2DriveINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //link2DriveINb
                continue;
            }
            //SETGRIPPERROTATION#167
            //SETGRIPPERROTATION#200
            //SETGRIPPERROTATION#300
            //SETGRIPPERROTATION#400
            //SETGRIPPERROTATION#500
            //SETGRIPPERROTATION#600
            //SETGRIPPERROTATION#700
            //SETGRIPPERROTATION#875
            if (strcmp((char*) cmdParts, "SETGRIPPERROTATION") == 0)
            {
                gripperRotationStaticMode = 1;
                gripperRotationStaticVal = atoi(strtok( NULL, "#" ));
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
            if (strcmp((char*) cmdParts, "GRIPPERROTATECCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //gripperRotate IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERROTATECW") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);    //gripperRotate IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERROTATESTOP") == 0)
            {
                gripperRotationStaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //gripperRotate IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //gripperRotate IN2
                stopGripperRotationDrive();
                continue;
            }
            //SETLINK3POSITION#0
            //SETLINK3POSITION#100
            //SETLINK3POSITION#200
            //SETLINK3POSITION#300
            //SETLINK3POSITION#400
            //SETLINK3POSITION#500
            //SETLINK3POSITION#600
            //SETLINK3POSITION#700
            //SETLINK3POSITION#800
            //SETLINK3POSITION#940
            if (strcmp((char*) cmdParts, "SETLINK3POSITION") == 0)
            {
                link3StaticMode = 1;
                link3StaticVal = atoi(strtok( NULL, "#" ));
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTGETDOWN") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //gripperTilt IN2
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTGETUP") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //gripperTilt IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //gripperTilt ENA
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERTILTSTOP") == 0)
            {
                link3StaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //gripperTilt IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //gripperTilt IN2
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //gripperTilt ENA
                continue;
            }
            //SETGRIPPERGRASP#180
            //SETGRIPPERGRASP#200
            //SETGRIPPERGRASP#300
            //SETGRIPPERGRASP#400
            //SETGRIPPERGRASP#500
            //SETGRIPPERGRASP#600
            //SETGRIPPERGRASP#700
            //SETGRIPPERGRASP#795
            if (strcmp((char*) cmdParts, "SETGRIPPERGRASP") == 0)
            {
                gripperGraspStaticMode = 1;
                gripperGraspStaticVal = atoi(strtok( NULL, "#" ));
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONOPEN") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9); //gripperPosition IN3
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //gripperPosition IN4
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONCLOSE") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //gripperPosition IN3
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "GRIPPERPOSITIONSTOP") == 0)
            {
                gripperGraspStaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //gripperPosition IN3
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //gripperPosition IN4
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //gripperPosition ENB
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOSSTART") == 0)
            {
                serialDemo = 1;
                thresholdSigma = 10;
                sprintf((char *)msg,"SERIAL EXECUTION MANIPULATOR DEMO STARTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DEMOSSTOP") == 0)
            {
                serialDemo = 0;
                stopAllManipulatorDrives();
                thresholdSigma = 2;
                sprintf((char *)msg,"SERIAL EXECUTION MANIPULATOR DEMO STOPPED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "MANSTOP") == 0)
            {
                serialDemo = 0;
                link1StaticMode = 0;
                link2StaticMode = 0;
                link3StaticMode = 0;
                gripperGraspStaticMode = 0;
                gripperRotationStaticMode = 0;
                stopAllManipulatorDrives();
                sprintf((char *)msg,"ALL MANIPULATOR DRIVES HAVE BEEN STOPPED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //SETTHRESHOLDSIGMA#1
            //SETTHRESHOLDSIGMA#2
            //SETTHRESHOLDSIGMA#3
            //SETTHRESHOLDSIGMA#4
            //SETTHRESHOLDSIGMA#5
            //SETTHRESHOLDSIGMA#6
            //SETTHRESHOLDSIGMA#10
            //SETTHRESHOLDSIGMA#20
            if (strcmp((char*) cmdParts, "SETTHRESHOLDSIGMA") == 0)
            {
                thresholdSigma = atoi(strtok( NULL, "#" ));
                continue;
            }
            if (strcmp((char*) cmdParts, "ISOBJECTDETECTEDSTART") == 0)
            {
                isObjectDetectedFlag = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ISOBJECTDETECTEDSTOP") == 0)
            {
                isObjectDetectedFlag = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "ISOBJECTGRASPEDSTART") == 0)
            {
                isObjectGraspedFlag = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "ISOBJECTGRASPEDSTOP") == 0)
            {
                isObjectGraspedFlag = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);   //U5, U6 D
                irBumperChannel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);   //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERCHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);     //U5, U6 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);     //U5, U6 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA23);     //U5, U6 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);     //U5, U6 D
                irBumperChannel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERENABLE") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERDISABLE") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERINIT") == 0)
            {
                AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA13_PWM2, 0);
                AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
                pwmFreqSet(2, irBumperFreq);
                pwmDutySetPercent(2, 1);
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                sprintf((char *)msg,"IR BUMPER HAS BEEN INITIALIZED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                irBumperInitialized = 1;
                continue;
            }
            //IRBUMPERFREQ#37500
            //IRBUMPERFREQ#37600
            //IRBUMPERFREQ#37700
            //IRBUMPERFREQ#37800
            //IRBUMPERFREQ#37900
            //IRBUMPERFREQ#38000
            //IRBUMPERFREQ#38100
            //IRBUMPERFREQ#38200
            //IRBUMPERFREQ#38300
            //IRBUMPERFREQ#38400
            //IRBUMPERFREQ#38500
            if (strcmp((char*) cmdParts, "IRBUMPERFREQ") == 0)
            {
                if (irBumperInitialized)
                {
                    irBumperFreq = atoi(strtok( NULL, "#" ));
                    pwmFreqSet(2, irBumperFreq);
                }
                else
                {
                    sprintf((char *)msg,"IR BUMPER IN NOT INITIALIZED\n");
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                }
                continue;
            }
            //SETIRBUMPERTRDUTY#1
            //SETIRBUMPERTRDUTY#2
            //SETIRBUMPERTRDUTY#3
            //SETIRBUMPERTRDUTY#4
            //SETIRBUMPERTRDUTY#5
            //SETIRBUMPERTRDUTY#10
            //SETIRBUMPERTRDUTY#15
            //SETIRBUMPERTRDUTY#20
            //SETIRBUMPERTRDUTY#25
            //SETIRBUMPERTRDUTY#30
            //SETIRBUMPERTRDUTY#35
            //SETIRBUMPERTRDUTY#40
            //SETIRBUMPERTRDUTY#45
            //SETIRBUMPERTRDUTY#50
            //SETIRBUMPERTRDUTY#60
            //SETIRBUMPERTRDUTY#70
            //SETIRBUMPERTRDUTY#80
            //SETIRBUMPERTRDUTY#90
            if (strcmp((char*) cmdParts, "SETIRBUMPERTRDUTY") == 0)
            {
                if (irBumperInitialized)
                {
                    irBumperDutyPercent = atoi(strtok( NULL, "#" ));
                    pwmDutySetPercent(2, irBumperDutyPercent);
                }
                else
                {
                    sprintf((char *)msg,"IR BUMPER IN NOT INITIALIZED\n");
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                }
                continue;
            }
            if (strcmp((char*) cmdParts, "IRBUMPERREADING") == 0)
            {
                if (irBumperInitialized)
                {
                    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
                    irBumperCnt = 0;
                    for (unsigned char i = 0; i < 101; i++)
                    {
                        AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                        delay_us(600);
                        AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
                        delay_us(600);
                    }
                    AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
                    sprintf((char *)msg,"IR BUMPER [%u][%u]: %u\n", irBumperChannel, irBumperDutyPercent, irBumperCnt);
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                }
                else
                {
                    sprintf((char *)msg,"IR BUMPER IN NOT INITIALIZED\n");
                    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                }
                continue;
            }
        }

        //Inspecting grasp drive current
        if (!inspectGraspDriveCurrent())
        {
            stopGripperGraspDrive();
            gripperGraspStaticMode = 0;
            serialDemo = 0;
        }

        if (input1Readings)
        {
            input1Reading = getValueChannel6();
            sprintf((char *)msg,"INPUT1 CHANNEL [%u]: %u\n", input1Channel, input1Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (link1Readings)
        {
            link1Reading = getValueChannel4();
            sprintf((char *)msg,"LINK1: %u\n", link1Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (link2Readings)
        {
            link2Reading = getValueChannel5();
            sprintf((char *)msg,"LINK2: %u\n", link2Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }

        if (serialDemo)
        {
            link1Reading = getLink1Position();
            link2Reading = getLink2Position();
            link3Reading = getLink3Position();
            gripperRotation = getGripperRotation();
            gripperGraspPosition = getGripperGraspPosition();

            if (serialDemoVec == 0)
            {
                if (absv((signed int) (link2Reading - link2lowerThreshold)) > thresholdSigma)
                {
                    setLink2Position(link2lowerThreshold);
                }
                else
                {
                    if (absv((signed int) (link1Reading - link1upperThreshold)) > thresholdSigma)
                    {
                        setLink1Position(link1upperThreshold);
                    }
                    else
                    {
                        if (absv((signed int) (link3Reading - link3lowerThreshold)) > thresholdSigma)
                        {
                            setLink3Position(link3lowerThreshold);
                        }
                        else
                        {
                            if (absv((signed int) (gripperRotation - gripperRotateCWThreshold)) > thresholdSigma)
                            {
                                setGripperRotation(gripperRotateCWThreshold);
                            }
                            else
                            {
                                if (absv((signed int) (gripperGraspPosition - gripperOpenedThreshold)) > thresholdSigma)
                                {
                                    setGripperGrasp(gripperOpenedThreshold);
                                }
                                else
                                {
                                    stopGripperGraspDrive();
                                    serialDemoVec = 1;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                if (isObjectDetected())
                {
                    unsigned char objectIsGrasped = isObjectGrasped(875);
                    if (objectIsGrasped == 0)
                    {
                        setGripperGrasp(gripperClosedThreshold);
                    }
                    else
                    {
                        stopGripperGraspDrive();
                        if (absv((signed int) (gripperRotation - gripperRotateCWWThreshold)) > thresholdSigma)
                        {
                            setGripperRotation(gripperRotateCWWThreshold);
                        }
                        else
                        {
                            stopGripperRotationDrive();
                            if (absv((signed int) (link3Reading - link3Initial)) > thresholdSigma)
                            {
                                setLink3Position(link3Initial);
                            }
                            else
                            {
                                stopLink3Drive();
                                if (absv((signed int) (link1Reading - link1lowerThreshold)) > thresholdSigma)
                                {
                                    setLink1Position(link1lowerThreshold);
                                }
                                else
                                {
                                    stopLink1Drive();
                                    if (absv((signed int) (link2Reading - link2upperThreshold)) > thresholdSigma)
                                    {
                                        setLink2Position(link2upperThreshold);
                                    }
                                    else
                                    {
                                        stopAllManipulatorDrives();
                                        serialDemo = 0;
                                        serialDemoVec = 0;
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    stopGripperGraspDrive();
                }
            }
        }
    }

    return 0; /* never reached */
}

