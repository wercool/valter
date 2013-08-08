#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Board.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "delay.h"

#define FIQ_INTERRUPT_LEVEL 0

const unsigned int usSensors40kHzPeriod = 20000;
const unsigned int usSensors40kHzDuty = 10000;
static volatile unsigned int usSensorsDelayCounter = 0;
static volatile unsigned int usSensorsPongTrigger = 0;

const unsigned int usRadarServoPeriod = 20000;
static volatile unsigned int leftUSRadarDelayCounter = 0;
static volatile unsigned int leftUSRadarPongTrigger = 0;
static volatile unsigned int rightUSRadarDelayCounter = 0;
static volatile unsigned int rightUSRadarPongTrigger = 0;

//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ0Handler(void)
{
    leftUSRadarPongTrigger = 1;
}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{
    rightUSRadarPongTrigger = 1;
}


//*----------------------------------------------------------------------------
//* Function Name       : FIQHandler
//* Object              : Interrupt Handler called by the FIQ interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void FIQHandler(void)
{
    usSensorsPongTrigger = 1;
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

    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, 1 | AT91C_PWMC_CPOL, usRadarServoPeriod, 1);                       //leftUSRadar  PWM0
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 1, 1 | AT91C_PWMC_CPOL, usRadarServoPeriod, 1);                       //rightUSRadar PWM1
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 2, 1 | AT91C_PWMC_CPOL, usSensors40kHzPeriod, usSensors40kHzDuty);    //usSensors    PWM2 - 40kHz

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, AT91C_PWMC_CHID0);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, AT91C_PWMC_CHID1);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 2, AT91C_PWMC_CHID2);

    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

    // enable PWM channels 0, 1, 2
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
    //AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
}

static void InitPIO(void)
{
    // U2, U3, U4, U8 PIO configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);   //U2, U3, U4, U8 EN
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7); //U2, U3, U4, U8 EN - HIGH to disable

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);

    // PWM configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);  //leftUSRadar  PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);  //rightUSRadar PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);  //usSensors    PWM2

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

    // US radars configuration
    // US radars interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5); //leftUSRadarTrigger
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6); //leftUSRadarTrigger

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

    //Input channel 1 configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U12 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);  //U12 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U12 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D

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
    AT91F_AIC_ConfigureIt(AT91C_BASE_PIOA, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) FIQHandler);
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
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);

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

    unsigned int sensorsChannel = 0;
    unsigned int sensorsIRReadings = 0;
    unsigned int sensorsUSReadings = 0;
    unsigned int sensorsIRReading = 0;
    unsigned int sensorsUSReading = 0;

    unsigned int leftUSRadarServoDuty = 1;
    unsigned int rightUSRadarServoDuty = 1;

    unsigned int leftUSRadarReadings = 0;
    unsigned int rightUSRadarReadings = 0;
    unsigned int leftUSRadarReading = 0;
    unsigned int rightUSRadarReading = 0;

    unsigned int input1Readings = 0;
    unsigned int input1Channel = 0;
    unsigned int input1Reading = 0;

    DeviceInit();

    while (1)
    {
        // perform US Sensor measurement
        if (sensorsUSReadings)
        {
            usSensorsDelayCounter = 0;
            usSensorsPongTrigger = 0;

            AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
            delay_us(1000);
            AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);

            while (usSensorsPongTrigger == 0)
            {
                usSensorsDelayCounter++;
                if (usSensorsDelayCounter > 50000)
                break;
            }

            AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
            sensorsUSReading = usSensorsDelayCounter;
        }

        // perform left US Radar measurement
        if (leftUSRadarReadings)
        {
            leftUSRadarDelayCounter = 0;
            leftUSRadarPongTrigger = 0;

            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
            delay_us(10);
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

            while (leftUSRadarPongTrigger == 0)
            {
                leftUSRadarDelayCounter++;
                if (leftUSRadarDelayCounter > 50000)
                break;
            }

            AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
            leftUSRadarReading = leftUSRadarDelayCounter;
        }

        // perform right US Radar measurement
        if (rightUSRadarReading)
        {
            rightUSRadarDelayCounter = 0;
            rightUSRadarPongTrigger = 0;

            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);
            delay_us(10);
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

            while (rightUSRadarPongTrigger == 0)
            {
                rightUSRadarDelayCounter++;
                if (rightUSRadarDelayCounter > 50000)
                break;
            }

            AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
            rightUSRadarReading = rightUSRadarDelayCounter;
        }

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;

            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "ENABLESENSORS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"SENSORS ENABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLESENSORS") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA7);
                sprintf((char *)msg,"SENSORS DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTIRSENSORSREADINGS") == 0)
            {
                sensorsIRReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPIRSENSORSREADINGS") == 0)
            {
                sensorsIRReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTUSSENSORSREADINGS") == 0)
            {
                sensorsUSReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPUSSENSORSREADINGS") == 0)
            {
                sensorsUSReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8   A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8   B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8   C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8   D
                sensorsChannel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U2, U3, U4, U8 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);  //U2, U3, U4, U8 D
                sensorsChannel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U2, U3, U4, U8 D
                sensorsChannel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U2, U3, U4, U8 D
                sensorsChannel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U2, U3, U4, U8 D
                sensorsChannel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);    //U2, U3, U4, U8 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);    //U2, U3, U4, U8 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);  //U2, U3, U4, U8 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U2, U3, U4, U8 D
                sensorsChannel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETSENSORSCHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA26);  //U2, U3, U4, U8 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);  //U2, U3, U4, U8 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);    //U2, U3, U4, U8 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);    //U2, U3, U4, U8 D
                sensorsChannel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLELEFTUSRADARSERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                sprintf((char *)msg,"LEFT US RADAR SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLERIGHTUSRADARSERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                sprintf((char *)msg,"RIGHT US RADAR SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "SETLEFTUSRADARSERVODUTY") == 0)
            {
                leftUSRadarServoDuty = atoi(strtok( NULL, "#" ));
                AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 0, leftUSRadarServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                continue;
            }
            if (strcmp((char*) cmdParts, "SETRIGHTUSRADARSERVODUTY") == 0)
            {
                rightUSRadarServoDuty = atoi(strtok( NULL, "#" ));
                AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, 1, rightUSRadarServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTLEFTUSRADARREADINGS") == 0)
            {
                leftUSRadarReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPLEFTUSRADARREADINGS") == 0)
            {
                leftUSRadarReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTRIGHTUSRADARREADINGS") == 0)
            {
                rightUSRadarReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPRIGHTUSRADARREADINGS") == 0)
            {
                rightUSRadarReadings = 0;
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
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL0") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL1") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL2") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 2;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL3") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 3;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL4") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 4;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL5") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 5;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL6") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 6;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL7") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 7;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL8") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 8;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL9") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 9;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL10") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 10;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL11") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 11;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL12") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 12;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL13") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 13;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL14") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 14;
                continue;
            }
            if (strcmp((char*) cmdParts, "SETINPUT1CHANNEL15") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14); //U12 A
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13); //U12 B
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24); //U12 C
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D
                input1Channel = 15;
                continue;
            }
        }
        if (sensorsIRReadings)
        {
            sensorsIRReading = getValueChannel4();
            sprintf((char *)msg,"SENSORS CHANNEL [%u] IR: %u\n", sensorsChannel, sensorsIRReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (sensorsUSReadings)
        {
            if (sensorsUSReading > 50000)
            {
                sprintf((char *)msg,"SENSORS CHANNEL [%u] US: FAILED\n", sensorsChannel);
            }
            else
            {
                sprintf((char *)msg,"SENSORS CHANNEL [%u] US: %u\n", sensorsChannel, sensorsUSReading);
            }
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (leftUSRadarReadings)
        {
            if (leftUSRadarReading > 50000)
            {
                sprintf((char *)msg,"LEFT US RADAR: FAILED\n");
            }
            else
            {
                sprintf((char *)msg,"LEFT US RADAR: %u\n", leftUSRadarReading);
            }
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (rightUSRadarReadings)
        {
            if (rightUSRadarReading > 50000)
            {
                sprintf((char *)msg,"RIGHT US RADAR: FAILED\n");
            }
            else
            {
                sprintf((char *)msg,"RIGHT US RADAR: %u\n", rightUSRadarReading);
            }
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (input1Readings)
        {
            input1Reading = getValueChannel5();
            sprintf((char *)msg,"INPUT1 CHANNEL [%u]: %u\n", input1Channel, input1Reading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
    }

    return 0; /* never reached */
}

