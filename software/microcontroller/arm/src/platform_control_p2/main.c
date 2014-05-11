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

unsigned int leftWheelEncoderTicks = 0;
unsigned int rightWheelEncoderTicks = 0;

unsigned char timerLeft = 0;
unsigned char timerRight = 0;

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

__ramfunc void SoftIQRHandler(void)
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
    //TODO check and set correctly
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

    pwmFreqSet(0, 60); // set by default to control Platform Front Sonar Drive
    pwmFreqSet(1, 2000);
    pwmFreqSet(2, 2000);

    pwmDutySet_u8(0, 1);
    pwmDutySet_u8(1, 1);
    pwmDutySet_u8(2, 1);

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
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);   //Alarm
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);   //CHARGER LEDs
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);   //Platform Front LEDs
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);   //Platform Rear LEDs
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);    //Platform Front Sonar Drive            PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);    //Charger Motor                         PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);    //PWM2
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

    // Platform wheels encoder interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);    // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);    // IRQ1

    //FIQ
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);    // FIQ

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

    //Open timer0
    AT91F_TC_Open(AT91C_BASE_TC0, TC_CLKS_MCK8, AT91C_ID_TC0);
    //Open Timer 0 interrupt
    AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC0, TIMER0_INTERRUPT_LEVEL,AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SoftIQRHandler);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;             // IRQ enable CPC
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_TC0);
    //Start timer0
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG ;
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

    unsigned int platformFrontSonarServoDuty = 1;
    unsigned int platformFrontSonarReadings = 0;
    unsigned int platformFrontSonarReading = 0;

    unsigned int chargerDriveDuty = 1;
    unsigned int chargerDriveDirection = 1; //1 - ?, 2 - ?

    unsigned int leftWheelEncoderReadings = 0;
    unsigned int rightWheelEncoderReadings = 0;

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
                sprintf((char *)msg,"PLATFORM-CONTROL-P2\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
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
            if (strcmp((char*) cmdParts, "LEFTWHEELENCODERRESET") == 0)
            {
                leftWheelEncoderTicks = 0;
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
            //SETSONARSERVODUTY#8
            //SETSONARSERVODUTY#9
            //SETSONARSERVODUTY#10
            //SETSONARSERVODUTY#11
            //SETSONARSERVODUTY#12
            //SETSONARSERVODUTY#13
            //SETSONARSERVODUTY#14
            //SETSONARSERVODUTY#15
            //SETSONARSERVODUTY#16
            //SETSONARSERVODUTY#17
            //SETSONARSERVODUTY#18 - center
            //SETSONARSERVODUTY#19
            //SETSONARSERVODUTY#20
            //SETSONARSERVODUTY#28
            if (strcmp((char*) cmdParts, "SETSONARSERVODUTY") == 0)
            {
                platformFrontSonarServoDuty = atoi(strtok( NULL, "#" ));
                AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PA0_PWM0, 0);
                AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
                pwmFreqSet(0, 60);
                pwmDutySet_u8(0, platformFrontSonarServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                sprintf((char *)msg,"PLATFORM FRONT SONAR SERVO ENABLED [%u]\n", platformFrontSonarServoDuty);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLESONARSERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
                sprintf((char *)msg,"PLATFORM FRONT SONAR SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTFRONTSONARREADINGS") == 0)
            {
                platformFrontSonarReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPFRONTSONARREADINGS") == 0)
            {
                platformFrontSonarReadings = 0;
                continue;
            }
            //SETCHARGERDRIVEDUTY#1
            //SETCHARGERDRIVEDUTY#5
            //SETCHARGERDRIVEDUTY#10
            //SETCHARGERDRIVEDUTY#15
            //SETCHARGERDRIVEDUTY#20
            //SETCHARGERDRIVEDUTY#30
            //SETCHARGERDRIVEDUTY#40
            //SETCHARGERDRIVEDUTY#50
            //SETCHARGERDRIVEDUTY#60
            //SETCHARGERDRIVEDUTY#70
            //SETCHARGERDRIVEDUTY#80
            //SETCHARGERDRIVEDUTY#90
            //SETCHARGERDRIVEDUTY#95
            //SETCHARGERDRIVEDUTY#100
            if (strcmp((char*) cmdParts, "SETCHARGERDRIVEDUTY") == 0)
            {
                chargerDriveDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, chargerDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            //CHARGERDRIVEDIRECTION#1
            //CHARGERDRIVEDIRECTION#2
            if (strcmp((char*) cmdParts, "CHARGERDRIVEDIRECTION") == 0)
            {
                chargerDriveDirection = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, 1);
                delay_ms(500);
                if (chargerDriveDirection == 1)
                {
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
                }
                if (chargerDriveDirection == 2)
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
                }
                pwmDutySetPercent(1, chargerDriveDuty);
                continue;
            }
            if (strcmp((char*) cmdParts, "CHARGERDRIVESTOP") == 0)
            {
                pwmDutySetPercent(1, 1);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
                continue;
            }
            //CHARGERDRIVEPUSH#1#95#250
            //CHARGERDRIVEPUSH#1#100#250
            //CHARGERDRIVEPUSH#2#95#250
            //CHARGERDRIVEPUSH#2#100#250
            if (strcmp((char*) cmdParts, "CHARGERDRIVEPUSH") == 0)
            {
                chargerDriveDirection = atoi(strtok( NULL, "#" ));
                chargerDriveDuty = atoi(strtok( NULL, "#" ));
                unsigned int chargerDrivePushDuration = atoi(strtok( NULL, "#" ));

                sprintf((char *)msg,"CHARGER DRIVE PUSH\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

                if (chargerDriveDirection == 1)
                {
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
                }
                if (chargerDriveDirection == 2)
                {
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //CHARGER MOTOR IN1
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //CHARGER MOTOR IN2
                }
                pwmDutySetPercent(1, chargerDriveDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                delay_ms(chargerDrivePushDuration);
                pwmDutySetPercent(1, 1);
                continue;
            }
            if (strcmp((char*) cmdParts, "ALARMON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"ALARM is ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "ALARMOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"ALARM is OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //ALARMBEEP#1
            //ALARMBEEP#2
            //ALARMBEEP#5
            //ALARMBEEP#10
            //ALARMBEEP#50
            //ALARMBEEP#100
            //ALARMBEEP#200
            if (strcmp((char*) cmdParts, "ALARMBEEP") == 0)
            {
                unsigned int alarmBeepDuration = atoi(strtok( NULL, "#" ));
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                delay_ms(alarmBeepDuration);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"ALARM BEEP\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "CHARGERLEDSON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
                sprintf((char *)msg,"CHARGER LEDs is ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "CHARGERLEDSOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
                sprintf((char *)msg,"CHARGER LEDs is OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "PLATFORMFRONTLEDSON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                sprintf((char *)msg,"PLATFORM FRONT LEDs is ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "PLATFORMFRONTLEDSOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                sprintf((char *)msg,"PLATFORM FRONT LEDs is OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "PLATFORMREARLEDSON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                sprintf((char *)msg,"PLATFORM REAR LEDs is ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "PLATFORMREARLEDSOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
                sprintf((char *)msg,"PLATFORM REAR LEDs is OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
        }

        if (leftWheelEncoderReadings)
        {
            sprintf((char *)msg,"LEFT WHEEL ENCODER: %u\n", leftWheelEncoderTicks);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(25);
        }
        if (rightWheelEncoderReadings)
        {
            sprintf((char *)msg,"RIGHT WHEEL ENCODER: %u\n", rightWheelEncoderTicks);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(25);
        }
        if (platformFrontSonarReadings)
        {
            platformFrontSonarReading = getValueChannel6();
            sprintf((char *)msg,"FRONT SONAR: %u\n", platformFrontSonarReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(25);
        }
    }

    return 0; /* never reached */
}

