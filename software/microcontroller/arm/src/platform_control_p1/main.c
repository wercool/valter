#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Board.h"
#include "cdc_enumerate.h"
#include "watchdog.h"
#include "adc.h"
#include "delay.h"
#include "pwm.h"
#include "util_math.h"

static volatile unsigned int leftMotorCounter = 0;
static volatile unsigned int rightMotorCounter = 0;
volatile unsigned int pwmFrequency = 8000;

unsigned int prevTurretReading = 0;
unsigned int turretReading = 0;
unsigned int turretPositionRange = 1000;
unsigned int turretRotationDirection = 0;
unsigned int turretPositioning = 0;

unsigned int dumperLeft = 0;
unsigned int dumperRight = 0;

//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ0Handler(void)
{
    if (dumperLeft == 100)
    {
        leftMotorCounter++;
        dumperLeft = 0;
    }
}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{
    if (dumperRight == 100)
    {
        rightMotorCounter++;
        dumperRight = 0;
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

    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA);

    // enable PWM clock in PMC
    AT91F_PWMC_CfgPMC();

    // disable PWM channels 0, 1, 2
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    //TODO: use pwm.h from platform_location_p1
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, 1 | AT91C_PWMC_CPOL, 2000, 1);
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 1, 1 | AT91C_PWMC_CPOL, 2000, 1);
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 2, 1 | AT91C_PWMC_CPOL, 2000, 1);

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);

    // enable PWM channels 0, 1, 2
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
}

static void InitPIO(void)
{
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15); //mainAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //inputChannel1 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //inputChannel1 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //inputChannel1 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //inputChannel1 D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);  //leftAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);  //rightAccumulatorRelay
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);  //dcdc5VEnable
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31); //chargerButton
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16); //leftAccumulatorEnable
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22); //rightAccumulatorEnable
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //inputChannel2 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //inputChannel2 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //inputChannel2 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //inputChannel2 D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);  //leftMotorPWM PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);  //rightMotorPWM PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);  //turretMotorPWM PWM2
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //leftMotorINa
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //leftMotorINb
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //rightMotorINa
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //rightMotorINb
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA17);  // ADC Channel 0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA18);  // ADC Channel 1
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA19);  // ADC Channel 2
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1


    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);

    // disable all pull-ups
    AT91C_BASE_PIOA->PIO_PPUDR = ~0;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ0Handler);
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ1Handler);
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
}

//*----------------------------------------------------------------------------
//* Function Name       : DeviceInit
//* Object              : Device peripherals initialization
///*----------------------------------------------------------------------------
static void DeviceInit(void)
{
    InitPIO();

    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);

    // Set-up the PIO
    // First, enable the clock of the PIO and set the LEDs in output
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );

    // Initialize USB device
    AT91FUSBOpen();
    // Wait for the end of enumeration
    while (!pCDC.IsConfigured(&pCDC))
    {
        watchdogReset();
    };

    InitPIO();
    InitADC();
    InitPWM();
    InitIRQ();
}

int getTurretPosition(unsigned char tracing)
{
    turretReading = getValueChannel5();
    if (turretReading == 1023)
    {
        if (turretRotationDirection == 1)
        {
            turretReading = prevTurretReading + 20;
        }
        else
        {
            turretReading = prevTurretReading - 20;
        }
    }
    else
    {
        if (turretReading > prevTurretReading)
        {
            turretRotationDirection = 1;
        }
        else
        {
            turretRotationDirection = 0;
        }
        prevTurretReading = turretReading;
    }
    if (tracing)
    {
        sprintf((char *)msg,"TURRET: %u\n", turretReading);
        pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    }
    return turretReading;
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

unsigned int setTurretPosition(unsigned int goalPosition)
{
    unsigned int curPosition = getTurretPosition(1);
    unsigned int positionTrend = round(((double)absv(goalPosition - curPosition) / (double)turretPositionRange) * 100);
    if (absv((signed int) (curPosition - goalPosition)) > 20)
    {
        unsigned char positionVector = (goalPosition > curPosition) ? 1 : 0;
        unsigned int duty = getDynamicDutyVal(positionTrend, -50, 30, 20);
        pwmDutySetPercent(2, duty);
        if (turretPositioning == 1)
        {
            if (positionVector == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //turretMotorINb
            }
            else
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //turretMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
            }
        }
    }
    else
    {
        turretPositioning = 0;
        pwmDutySetPercent(2, 1);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
    }
    sprintf((char *)msg,"TURRET TREND[%d] CUR[%d]->GOAL[%d]\n", positionTrend, curPosition, goalPosition);
    pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
    return curPosition;
}

void stopTurretDrive()
{
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
    turretPositioning = 0;
}


/*
 * Main Entry Point and Main Loop
 */
int main(void)
{
    watchdogReset();

    unsigned char WDINTENTIONALRESET = 0;

    struct cdcMessage cdcMessageObj;

    unsigned int input1Readings = 0;
    unsigned int input1Channel = 0;
    unsigned int input1Reading = 0;

    unsigned int input2Readings = 0;
    unsigned int input2Channel = 0;
    unsigned int input2Reading = 0;

    unsigned int turretReadings = 0;

    unsigned int leftMotorPWMDuty = 0;
    unsigned int rightMotorPWMDuty = 0;
    unsigned int turretMotorPWMDuty = 0;

    unsigned int leftMotorCurrentReadings = 0;
    unsigned int leftMotorCurrentReading = 0;
    unsigned int rightMotorCurrentReadings = 0;
    unsigned int rightMotorCurrentReading = 0;
    unsigned int turretMotorCurrentReadings = 0;
    unsigned int turretMotorCurrentReading = 0;

    unsigned char turretStaticMode = 0;
    unsigned int turretStaticVal = 0;

    unsigned int leftMotorCounterReadings = 0;
    unsigned int rightMotorCounterReadings = 0;

    DeviceInit();

    watchdogReset();

    while (1)
    {

        if (WDINTENTIONALRESET == 0)
        {
            watchdogReset();
        }

        if (dumperLeft < 100)
        {
            dumperLeft++;
        }
        if (dumperRight < 100)
        {
            dumperRight++;
        }

        if (turretStaticMode)
        {
            unsigned int curVal = setTurretPosition(turretStaticVal);
            if (absv((signed int) (curVal - turretStaticVal)) <= 20)
            {
                turretPositioning = 0;
                turretStaticMode = 0;
                stopTurretDrive();
            }
        }

        //IMPORTANT!!! Implement in a working firmware
        unsigned int turret_pos = getTurretPosition(0);
        if ((turret_pos <= 25 || turret_pos >= 999) && turret_pos != 1023)
        {
            sprintf((char *)msg,"TURRET EMERGENCY STOP\n");
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            turretStaticMode = 0;
            stopTurretDrive();
        }

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
/*
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
*/
            char *cmdParts;

            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "GETID") == 0)
            {
                sprintf((char *)msg,"PLATFORM-CONTROL-P1\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "WDRESET") == 0)
            {
                watchdogReset();
                sprintf((char *)msg,"WDRST\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "WDINTENTIONALRESETON") == 0)
            {
                WDINTENTIONALRESET = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "WDINTENTIONALRESETOFF") == 0)
            {
                WDINTENTIONALRESET = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETALLCURREADINGS") == 0)
            {
                input1Reading = getValueChannel0();
                input2Reading = getValueChannel6();
                turretReading = getValueChannel5();
                leftMotorCurrentReading = getValueChannel1();
                rightMotorCurrentReading = getValueChannel2();
                turretMotorCurrentReading = getValueChannel4();
                //ALLCURREADINGS:IN1 CH#,READING;IN2 CH#,READING;TURRET POSITION;LEFT MOTOR CURRENT;RIGHT MOTOR CURRENT;TURRET MOTOR CURRENT;LEFT MOTOR COUNTER;RIGHT MOTOR COUNTER
                sprintf((char *)msg,"ALLCURREADINGS:%u,%u;%u,%u;%u;%u;%u;%u;%u;%u\n", input1Channel, input1Reading, input2Channel, input2Reading, turretReading, leftMotorCurrentReading, rightMotorCurrentReading, turretMotorCurrentReading, leftMotorCounter, rightMotorCounter);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            // CAUTION!!! Use only when AC/DC 12V connected.
            if (strcmp((char*) cmdParts, "MAINACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
                sprintf((char *)msg,"MAIN ACCUMULATOR RELAY SET ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "MAINACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
                sprintf((char *)msg,"MAIN ACCUMULATOR RELAY SET OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "CHARGERBUTTONPRESS") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                delay_ms(500);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
                continue;
            }
            if (strcmp((char*) cmdParts, "DCDC5VENABLEON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                sprintf((char *)msg,"DC/DC 5V ENABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DCDC5VENABLEOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);
                sprintf((char *)msg,"DC/DC 5V DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"LEFT ACCUMULATOR RELAY SET ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                sprintf((char *)msg,"LEFT ACCUMULATOR RELAY SET OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATORRELAYON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"RIGHT ACCUMULATOR RELAY SET ON\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATORRELAYOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"RIGHT ACCUMULATOR RELAY SET OFF\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "GETINPUT1") == 0)
            {
                input1Reading = getValueChannel0();
                sprintf((char *)msg,"INPUT1 CHANNEL [%u]: %u\n", input1Channel, input1Reading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //main accumulator voltage
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 0;
                continue;
            }
            //left accumulator voltage
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 1;
                continue;
            }
            //right accumulator voltage
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 2;
                continue;
            }
            //main accumulator amperage total
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 3;
                continue;
            }
            //main accumulator amperage bottom
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 4;
                continue;
            }
            //main accumulator amperage top
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 5;
                continue;
            }
            //left accumulator amperage
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 6;
                continue;
            }
            //right accumulator amperage
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);  //D
                input1Channel = 7;
                continue;
            }
            //charger connected (charger voltage)
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 8;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 9;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 10;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 11;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 12;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 13;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 14;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //D
                input1Channel = 15;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATORCONNECT") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
                sprintf((char *)msg,"LEFT ACCUMULATOR CONNECTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTACCUMULATORDISCONNECT") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
                sprintf((char *)msg,"LEFT ACCUMULATOR DISCONNECTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATORCONNECT") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
                sprintf((char *)msg,"RIGHT ACCUMULATOR CONNECTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTACCUMULATORDISCONNECT") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA22);
                sprintf((char *)msg,"RIGHT ACCUMULATOR DISCONNECTED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "GETINPUT2") == 0)
            {
                input2Reading = getValueChannel6();
                sprintf((char *)msg,"INPUT2 CHANNEL [%u]: %u\n", input2Channel, input2Reading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //left motor DIAGa/ENa
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 0;
                continue;
            }
            //left motor DIAGb/ENb
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 1;
                continue;
            }
            //right motor DIAGa/ENa
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 2;
                continue;
            }
            //right motor DIAGb/ENb
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 3;
                continue;
            }
            //turret motor DIAGa/ENa
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 4;
                continue;
            }
            //turret motor DIAGb/ENb
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);  //D
                input2Channel = 5;
                continue;
            }
            //(from charger) CHARGE IN PROGRESS
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 6;
                continue;
            }
            //(from charger) CHARGE COMPLETE
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //D
                input2Channel = 7;
                continue;
            }
            //(from charger) CHARGER CONNECTED
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 8;
                continue;
            }
            //(from charger) 14.4 V / 0.8 A 1.2 - 35 Ah
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 9;
                continue;
            }
            //(from charger) 14.4 V / 3.6 A 35 - 120 Ah
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 10;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 11;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 12;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25); //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 13;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26); //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 14;
                continue;
            }
            //reserved
            if (strcmp((char*) cmdParts, "SETINPUT2CHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);   //A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);   //B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);   //C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);   //D
                input2Channel = 15;
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
            if (strcmp((char*) cmdParts, "STARTINPUT2READINGS") == 0)
            {
                input2Readings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPINPUT2READINGS") == 0)
            {
                input2Readings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETTURRETPOSITION") == 0)
            {
                turretReading = getValueChannel5();
                sprintf((char *)msg,"TURRET: %u\n", turretReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTTURRETREADINGS") == 0)
            {
                turretReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPTURRETREADINGS") == 0)
            {
                turretReadings = 0;
                continue;
            }
            //SETPWMFREQUENCY#8000
            if (strcmp((char*) cmdParts, "SETPWMFREQUENCY") == 0)
            {
                pwmFrequency = atoi(strtok( NULL, "#" ));
                pwmFreqSet(0, pwmFrequency);
                pwmFreqSet(1, pwmFrequency);
                pwmFreqSet(2, pwmFrequency);
                continue;
            }
            /*
            SETLEFTMOTORPWMDUTY#1
            SETLEFTMOTORPWMDUTY#5
            SETLEFTMOTORPWMDUTY#10
            SETLEFTMOTORPWMDUTY#15
            SETLEFTMOTORPWMDUTY#20
            SETLEFTMOTORPWMDUTY#25
            SETLEFTMOTORPWMDUTY#30
            SETLEFTMOTORPWMDUTY#35
            SETLEFTMOTORPWMDUTY#40
            SETLEFTMOTORPWMDUTY#45
            SETLEFTMOTORPWMDUTY#50
            SETLEFTMOTORPWMDUTY#55
            SETLEFTMOTORPWMDUTY#60
            SETLEFTMOTORPWMDUTY#65
            SETLEFTMOTORPWMDUTY#70
            SETLEFTMOTORPWMDUTY#75
            SETLEFTMOTORPWMDUTY#80
            SETLEFTMOTORPWMDUTY#85
            SETLEFTMOTORPWMDUTY#90
            SETLEFTMOTORPWMDUTY#100
            */
            if (strcmp((char*) cmdParts, "SETLEFTMOTORPWMDUTY") == 0)
            {
                leftMotorPWMDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(0, leftMotorPWMDuty);
                continue;
            }
            /*
            SETRIGHTMOTORPWMDUTY#1
            SETRIGHTMOTORPWMDUTY#5
            SETRIGHTMOTORPWMDUTY#10
            SETRIGHTMOTORPWMDUTY#15
            SETRIGHTMOTORPWMDUTY#20
            SETRIGHTMOTORPWMDUTY#25
            SETRIGHTMOTORPWMDUTY#30
            SETRIGHTMOTORPWMDUTY#35
            SETRIGHTMOTORPWMDUTY#40
            SETRIGHTMOTORPWMDUTY#45
            SETRIGHTMOTORPWMDUTY#50
            SETRIGHTMOTORPWMDUTY#55
            SETRIGHTMOTORPWMDUTY#60
            SETRIGHTMOTORPWMDUTY#65
            SETRIGHTMOTORPWMDUTY#70
            SETRIGHTMOTORPWMDUTY#75
            SETRIGHTMOTORPWMDUTY#80
            SETRIGHTMOTORPWMDUTY#85
            SETRIGHTMOTORPWMDUTY#90
            SETRIGHTMOTORPWMDUTY#100
            */
            if (strcmp((char*) cmdParts, "SETRIGHTMOTORPWMDUTY") == 0)
            {
                rightMotorPWMDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(1, rightMotorPWMDuty);
                continue;
            }
            /*
            SETTURRETMOTORPWMDUTY#1
            SETTURRETMOTORPWMDUTY#5
            SETTURRETMOTORPWMDUTY#10
            SETTURRETMOTORPWMDUTY#15
            SETTURRETMOTORPWMDUTY#20
            SETTURRETMOTORPWMDUTY#25
            SETTURRETMOTORPWMDUTY#30
            SETTURRETMOTORPWMDUTY#35
            SETTURRETMOTORPWMDUTY#40
            */
            if (strcmp((char*) cmdParts, "SETTURRETMOTORPWMDUTY") == 0)
            {
                turretMotorPWMDuty = atoi(strtok( NULL, "#" ));
                pwmDutySetPercent(2, turretMotorPWMDuty);
                continue;
            }
            //SETTURRETPOSITION#25
            //SETTURRETPOSITION#50
            //SETTURRETPOSITION#100
            //SETTURRETPOSITION#150
            //SETTURRETPOSITION#200
            //SETTURRETPOSITION#250
            //SETTURRETPOSITION#300
            //SETTURRETPOSITION#350
            //SETTURRETPOSITION#400
            //!!!
            //SETTURRETPOSITION#450
            //SETTURRETPOSITION#500
            //SETTURRETPOSITION#525 INTITIAL !!!
            //SETTURRETPOSITION#550
            //!!!
            //SETTURRETPOSITION#600
            //SETTURRETPOSITION#650
            //SETTURRETPOSITION#700
            //SETTURRETPOSITION#750
            //SETTURRETPOSITION#800
            //SETTURRETPOSITION#850
            //SETTURRETPOSITION#900
            //SETTURRETPOSITION#950
            //SETTURRETPOSITION#1000
            if (strcmp((char*) cmdParts, "SETTURRETPOSITION") == 0)
            {
                turretPositioning = 1;
                prevTurretReading = getTurretPosition(1);
                turretStaticMode = 1;
                turretStaticVal = atoi(strtok( NULL, "#" ));
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTMOTORCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //leftMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //leftMotorINb
                delay_ms(100);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);    //leftMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //leftMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTMOTORCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //leftMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //leftMotorINb
                delay_ms(100);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //leftMotorINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);    //leftMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTMOTORSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);  //leftMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);  //leftMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTMOTORCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //rightMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //rightMotorINb
                delay_ms(100);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);    //rightMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //rightMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTMOTORCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //rightMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //rightMotorINb
                delay_ms(100);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //rightMotorINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);   //rightMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTMOTORSTOP") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);  //rightMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27); //rightMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "TURRETMOTORCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
                delay_ms(100);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);   //turretMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "TURRETMOTORCCW") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
                delay_ms(100);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);   //turretMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "TURRETMOTORSTOP") == 0)
            {
                turretStaticMode = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28); //turretMotorINa
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29); //turretMotorINb
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLMOTORCUR") == 0)
            {
                leftMotorCurrentReading = getValueChannel1();
                sprintf((char *)msg,"LEFT MOTOR CURRENT: %u\n", leftMotorCurrentReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTLEFTMOTORCURRENTREADINGS") == 0)
            {
                leftMotorCurrentReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPLEFTMOTORCURRENTREADINGS") == 0)
            {
                leftMotorCurrentReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETRMOTORCUR") == 0)
            {
                rightMotorCurrentReading = getValueChannel2();
                sprintf((char *)msg,"RIGHT MOTOR CURRENT: %u\n", rightMotorCurrentReading);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTRIGHTMOTORCURRENTREADINGS") == 0)
            {
                rightMotorCurrentReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPRIGHTMOTORCURRENTREADINGS") == 0)
            {
                rightMotorCurrentReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTTURRETMOTORCURRENTREADINGS") == 0)
            {
                turretMotorCurrentReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPTURRETMOTORCURRENTREADINGS") == 0)
            {
                turretMotorCurrentReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETLEFTMOTORCOUNTER") == 0)
            {
                sprintf((char *)msg,"LEFT MOTOR COUNTER: %u\n", leftMotorCounter);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RESETLEFTMOTORCOUNTER") == 0)
            {
                leftMotorCounter = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "GETRIGHTMOTORCOUNTER") == 0)
            {
                sprintf((char *)msg,"RIGHT MOTOR COUNTER: %u\n", rightMotorCounter);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "RESETRIGHTMOTORCOUNTER") == 0)
            {
                rightMotorCounter = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTMOTORCOUNTERSTART") == 0)
            {
                leftMotorCounterReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "LEFTMOTORCOUNTERSTOP") == 0)
            {
                leftMotorCounterReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTMOTORCOUNTERSTART") == 0)
            {
                rightMotorCounterReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "RIGHTMOTORCOUNTERSTOP") == 0)
            {
                rightMotorCounterReadings = 0;
                continue;
            }
        }

        if (input1Readings)
        {
            input1Reading = getValueChannel0();
            sprintf((char *)msg,"INPUT1 CHANNEL [%u]: %u\n", input1Channel, input1Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (input2Readings)
        {
            input2Reading = getValueChannel6();
            sprintf((char *)msg,"INPUT2 CHANNEL [%u]: %u\n", input2Channel, input2Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (turretReadings)
        {
            turretReading = getValueChannel5();
            sprintf((char *)msg,"TURRET: %u\n", turretReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (leftMotorCurrentReadings)
        {
            leftMotorCurrentReading = getValueChannel1();
            sprintf((char *)msg,"LEFT MOTOR CURRENT: %u\n", leftMotorCurrentReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (rightMotorCurrentReadings)
        {
            rightMotorCurrentReading = getValueChannel2();
            sprintf((char *)msg,"RIGHT MOTOR CURRENT: %u\n", rightMotorCurrentReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (turretMotorCurrentReadings)
        {
            turretMotorCurrentReading = getValueChannel4();
            sprintf((char *)msg,"TURRET MOTOR CURRENT: %u\n", turretMotorCurrentReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (leftMotorCounterReadings)
        {
            sprintf((char *)msg,"LEFT MOTOR COUNTER: %u\n", leftMotorCounter);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (rightMotorCounterReadings)
        {
            sprintf((char *)msg,"RIGHT MOTOR COUNTER: %u\n", rightMotorCounter);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }

    }

    return 0; /* never reached */
}

