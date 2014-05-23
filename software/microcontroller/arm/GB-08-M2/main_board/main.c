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

unsigned int leftWheelEncoderTicks = 0;
unsigned int rightWheelEncoderTicks = 0;

unsigned char timerLeft = 0;
unsigned char timerRight = 0;


unsigned int servoSignalPeriod = 0;
unsigned int servoSignalWidth = 0;
unsigned int radarServoRotation = 0;
unsigned char radarServoSet = 0;

//temp
unsigned tempPeriod = 10;

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
    if (timerLeft == 1)
    {
        leftWheelEncoderTicks++;
        if (leftWheelEncoderTicks > 32000)
            leftWheelEncoderTicks = 0;
        timerLeft = 0;
    }
}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{
    if (timerRight == 1)
    {
        rightWheelEncoderTicks++;
        if (rightWheelEncoderTicks > 32000)
            rightWheelEncoderTicks = 0;
        timerRight = 0;
    }
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
    AT91PS_TC TC_pt = AT91C_BASE_TC0;
    unsigned int dummy;
    //* Acknowledge interrupt status
    dummy = TC_pt->TC_SR;
    //* Suppress warning variable "dummy" was set but never used
    dummy = dummy;

    if (timerLeft == 0)
        timerLeft = 1;

    if (timerRight == 0)
        timerRight = 1;
}

__ramfunc void SoftIQRHandlerTC1(void)
{
    AT91PS_TC TC_pt = AT91C_BASE_TC1;
    unsigned int dummy;
    //* Acknowledge interrupt status
    dummy = TC_pt->TC_SR;
    //* Suppress warning variable "dummy" was set but never used
    dummy = dummy;

//    // Servo software signal generation
//    if (radarServoSet)
//    {
//        if (servoSignalPeriod > tempPeriod)
//        {
//            if (servoSignalWidth < radarServoRotation)
//            {
//                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
//                servoSignalWidth++;
//            }
//            else
//            {
//                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
//                servoSignalPeriod = 0;
//                servoSignalWidth = 0;
//            }
//        }
//        else
//        {
//            servoSignalPeriod++;
//        }
//    }

    if (radarServoSet)
    {
        radarServoSet = 0;
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    }
    else
    {
        radarServoSet = 1;
        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    }

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

    pwmFreqSet(0, 4000);
    pwmFreqSet(1, 4000);
    pwmFreqSet(2, 4000);
    pwmFreqSet(3, 4000);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 3, AT91C_PWMC_CHID3);

    // enable PWM channels 0, 1, 2
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID3);
}

static void InitPIO(void)
{
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);        //Left Wheels INa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);       //Left Wheels INb
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);       //Right Wheels INa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);       //Right Wheels INb
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);        //Lights
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);        //Servo Signal
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);    //PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);    //PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);    //PWM2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);    //PWM3
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

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
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ0Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ1Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    //AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) FIQHandler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);

    //Open timer0
    AT91F_TC_Open(AT91C_BASE_TC0, TC_CLKS_MCK8, AT91C_ID_TC0);
    //Open Timer 0 interrupt
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_TC0, TIMER0_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SoftIQRHandlerTC0);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;             // IRQ enable CPC
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_TC0);
    //Start timer0
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG ;

    //Open timer1
    AT91F_TC_Open(AT91C_BASE_TC1, TC_CLKS_MCK2, AT91C_ID_TC1);
    //Open Timer 1 interrupt
    AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_TC1, TIMER1_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SoftIQRHandlerTC1);
    AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;             // IRQ enable CPC
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_TC1);
    //Start timer1
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG ;
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

    unsigned int frontLeftWheelDuty = 1;
    unsigned int rearLeftWheelDuty = 1;
    unsigned int frontRightWheelDuty = 1;
    unsigned int rearRightWheelDuty = 1;

    unsigned int leftWheelsDirection = 1;
    unsigned int rightWheelsDirection = 1;

    unsigned int leftFrontCurReadings = 0;
    unsigned int leftFrontCurVal = 0;

    unsigned int rightFrontCurReadings = 0;
    unsigned int rightFrontCurVal = 0;

    unsigned int rightRearCurReadings = 0;
    unsigned int rightRearCurVal = 0;

    unsigned int leftRearCurReadings = 0;
    unsigned int leftRearCurVal = 0;

    unsigned int batteryVoltageReadings = 0;
    unsigned int batteryVoltageReading = 0;

    unsigned int distanceMeterReading = 0;

    unsigned int leftWheelEncoderReadings = 0;
    unsigned int rightWheelEncoderReadings = 0;

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
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            //PERIOD#100
            if (strcmp((char*) cmdParts, "PERIOD") == 0)
            {
                tempPeriod = atoi(strtok( NULL, "#" ));
                continue;
            }

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"GB-08-M2 MAIN BOARD\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //FRONTLEFTDUTY#1
            //FRONTLEFTDUTY#10
            //FRONTLEFTDUTY#20
            //FRONTLEFTDUTY#30
            //FRONTLEFTDUTY#40
            //FRONTLEFTDUTY#50
            //FRONTLEFTDUTY#60
            //FRONTLEFTDUTY#70
            //FRONTLEFTDUTY#80
            //FRONTLEFTDUTY#90
            //FRONTLEFTDUTY#100
            if (strcmp((char*) cmdParts, "FRONTLEFTDUTY") == 0)
            {
                frontLeftWheelDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(0, frontLeftWheelDuty);
                continue;
            }
            //FRONTRIGHTDUTY#1
            //FRONTRIGHTDUTY#5
            //FRONTRIGHTDUTY#10
            //FRONTRIGHTDUTY#20
            //FRONTRIGHTDUTY#30
            //FRONTRIGHTDUTY#40
            //FRONTRIGHTDUTY#50
            //FRONTRIGHTDUTY#60
            //FRONTRIGHTDUTY#70
            //FRONTRIGHTDUTY#80
            //FRONTRIGHTDUTY#90
            //FRONTRIGHTDUTY#100
            if (strcmp((char*) cmdParts, "FRONTRIGHTDUTY") == 0)
            {
                frontRightWheelDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, frontRightWheelDuty);
                continue;
            }
            //REARLEFTDUTY#1
            //REARLEFTDUTY#10
            //REARLEFTDUTY#20
            //REARLEFTDUTY#30
            //REARLEFTDUTY#40
            //REARLEFTDUTY#50
            //REARLEFTDUTY#60
            //REARLEFTDUTY#70
            //REARLEFTDUTY#80
            //REARLEFTDUTY#90
            //REARLEFTDUTY#100
            if (strcmp((char*) cmdParts, "REARLEFTDUTY") == 0)
            {
                rearLeftWheelDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(3, rearLeftWheelDuty);
                continue;
            }
            //REARRIGHTDUTY#1
            //REARRIGHTDUTY#10
            //REARRIGHTDUTY#20
            //REARRIGHTDUTY#30
            //REARRIGHTDUTY#40
            //REARRIGHTDUTY#50
            //REARRIGHTDUTY#60
            //REARRIGHTDUTY#70
            //REARRIGHTDUTY#80
            //REARRIGHTDUTY#90
            //REARRIGHTDUTY#100
            if (strcmp((char*) cmdParts, "REARRIGHTDUTY") == 0)
            {
                rearRightWheelDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(2, rearRightWheelDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTFORWARD") == 0)
            {
                leftWheelsDirection = 1;
                pwmDutySetPercent(0, 1);
                pwmDutySetPercent(3, 1);
                delay_ms(500);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);      //Left Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);       //Left Wheels INb
                pwmDutySetPercent(0, frontLeftWheelDuty);
                pwmDutySetPercent(3, rearLeftWheelDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTBACKWARD") == 0)
            {
                leftWheelsDirection = 2;
                pwmDutySetPercent(0, 1);
                pwmDutySetPercent(3, 1);
                delay_ms(500);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);        //Left Wheels INa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);     //Left Wheels INb
                pwmDutySetPercent(0, frontLeftWheelDuty);
                pwmDutySetPercent(3, rearLeftWheelDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTSTOP") == 0)
            {
                pwmDutySetPercent(0, 1);
                pwmDutySetPercent(3, 1);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);        //Left Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);       //Left Wheels INb
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTFORWARD") == 0)
            {
                rightWheelsDirection = 1;
                pwmDutySetPercent(1, 1);
                pwmDutySetPercent(2, 1);
                delay_ms(500);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);     //Right Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);       //Right Wheels INb
                pwmDutySetPercent(1, rearRightWheelDuty);
                pwmDutySetPercent(2, frontRightWheelDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTBACKWARD") == 0)
            {
                rightWheelsDirection = 2;
                pwmDutySetPercent(1, 1);
                pwmDutySetPercent(2, 1);
                delay_ms(500);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);       //Right Wheels INa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);     //Right Wheels INb
                pwmDutySetPercent(1, rearRightWheelDuty);
                pwmDutySetPercent(2, frontRightWheelDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTSTOP") == 0)
            {
                pwmDutySetPercent(1, 1);
                pwmDutySetPercent(2, 1);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);       //Right Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);       //Right Wheels INb
                continue;
            }
            if (strcmp((char*) cmdParts, "GETRIGHTFRONTCURRENT") == 0)
            {
                rightFrontCurVal = getValueChannel2();
                sprintf((char *)msg,"FRONT RIGHT MOTOR CURRENT: %u\n", rightFrontCurVal);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTRIGHTFRONTCURRENT") == 0)
            {
                rightFrontCurReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPRIGHTFRONTCURRENT") == 0)
            {
                rightFrontCurReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETRIGHTREARCURRENT") == 0)
            {
                rightRearCurVal = getValueChannel1();
                sprintf((char *)msg,"REAR RIGHT MOTOR CURRENT: %u\n", rightRearCurVal);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTRIGHTREARCURRENT") == 0)
            {
                rightRearCurReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPRIGHTREARCURRENT") == 0)
            {
                rightRearCurReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLEFTFRONTCURRENT") == 0)
            {
                leftFrontCurVal = getValueChannel0();
                sprintf((char *)msg,"FRONT LEFT MOTOR CURRENT: %u\n", leftFrontCurVal);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTLEFTFRONTCURRENT") == 0)
            {
                leftFrontCurReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPLEFTFRONTCURRENT") == 0)
            {
                leftFrontCurReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLEFTREARCURRENT") == 0)
            {
                leftRearCurVal = getValueChannel4();
                sprintf((char *)msg,"REAR LEFT MOTOR CURRENT: %u\n", leftRearCurVal);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTLEFTREARCURRENT") == 0)
            {
                leftRearCurReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPLEFTREARCURRENT") == 0)
            {
                leftRearCurReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETBATTERYVOLTAGE") == 0)
            {
                batteryVoltageReading = getValueChannel6();
                sprintf((char *)msg,"BATTERY VOLTAGE: %u\n", batteryVoltageReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTBATTERYVOLTAGEREADINGS") == 0)
            {
                batteryVoltageReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPBATTERYVOLTAGEREADINGS") == 0)
            {
                batteryVoltageReadings = 0;
                continue;
            }
            //RADARROTATIONSET#1
            //RADARROTATIONSET#2
            //RADARROTATIONSET#3
            //RADARROTATIONSET#4
            //RADARROTATIONSET#5
            //RADARROTATIONSET#6
            //RADARROTATIONSET#7
            //RADARROTATIONSET#8
            //RADARROTATIONSET#9
            //RADARROTATIONSET#10
            if (strcmp((char*) cmdParts, "RADARROTATIONSET") == 0)
            {
                radarServoRotation = atoi(strtok( NULL, "#" ));
                radarServoSet = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RADARROTATIONRESET") == 0)
            {
                radarServoSet = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
                servoSignalPeriod = 0;
                servoSignalWidth = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETDISTANCE") == 0)
            {
                distanceMeterReading = getValueChannel5();
                sprintf((char *)msg,"DISTANCE: %u\n", distanceMeterReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLEFTWHEELENCODER") == 0)
            {
                sprintf((char *)msg,"LEFT ENCODER: %u\n", leftWheelEncoderTicks);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTWHEELENCODERSTARTREADINGS") == 0)
            {
                AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                leftWheelEncoderReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTWHEELENCODERSTOPREADINGS") == 0)
            {
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                leftWheelEncoderReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETRIGHTWHEELENCODER") == 0)
            {
                sprintf((char *)msg,"RIGHT ENCODER: %u\n", rightWheelEncoderTicks);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(10);
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTWHEELENCODERSTARTREADINGS") == 0)
            {
                AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
                rightWheelEncoderReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTWHEELENCODERSTOPREADINGS") == 0)
            {
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
                rightWheelEncoderReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTWHEELENCODERRESET") == 0)
            {
                rightWheelEncoderTicks = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTWHEELENCODERRESET") == 0)
            {
                leftWheelEncoderTicks = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "LIGHTSON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                continue;
            }
            if (strcmp((char*) cmdParts, "LIGHTSOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                continue;
            }
        }


        if (leftFrontCurReadings)
        {
            leftFrontCurVal = getValueChannel0();
            sprintf((char *)msg,"FRONT LEFT MOTOR CURRENT: %u\n", leftFrontCurVal);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (rightFrontCurReadings)
        {
            rightFrontCurVal = getValueChannel2();
            sprintf((char *)msg,"FRONT RIGHT MOTOR CURRENT: %u\n", rightFrontCurVal);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (leftRearCurReadings)
        {
            leftRearCurVal = getValueChannel4();
            sprintf((char *)msg,"REAR LEFT MOTOR CURRENT: %u\n", leftRearCurVal);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (rightRearCurReadings)
        {
            rightRearCurVal = getValueChannel1();
            sprintf((char *)msg,"REAR RIGHT MOTOR CURRENT: %u\n", rightRearCurVal);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (batteryVoltageReadings)
        {
            batteryVoltageReading = getValueChannel6();
            sprintf((char *)msg,"BATTERY VOLTAGE: %u\n", batteryVoltageReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (leftWheelEncoderReadings)
        {
            sprintf((char *)msg,"LEFT ENCODER: %u\n", leftWheelEncoderTicks);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
        if (rightWheelEncoderReadings)
        {
            sprintf((char *)msg,"RIGHT ENCODER: %u\n", rightWheelEncoderTicks);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
        }
    }

    return 0; /* never reached */
}

