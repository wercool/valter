#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Board.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "pwm.h"
#include "aic.h"
#include "twi.h"
#include "async.h"
#include "twid.h"
#include "delay.h"

#define FIQ_INTERRUPT_LEVEL 0

const unsigned int usSensors40kHzFrequency = 40000;
const unsigned int usSensors40kHzDuty = 127;
volatile unsigned int usSensorsDelayCounter = 0;
volatile unsigned int usSensorsPongTrigger = 0;

const unsigned int usSonarServoFrequency = 60;
volatile unsigned int leftUSSonarDelayCounter = 0;
volatile unsigned int leftUSSonarPongTrigger = 0;
volatile unsigned int rightUSSonarDelayCounter = 0;
volatile unsigned int rightUSSonarPongTrigger = 0;

/// TWI clock frequency in Hz.
#define TWCK            400000

#define STLM75_ADDR_S1            0x4F
#define STLM75_ADDR_S2            0x4B
#define STLM75_REG_TEMP           0x00
#define STLM75_REG_CONF           0x01
#define STLM75_REG_TEMP_HYST      0x02
#define STLM75_REG_TEMP_OS        0x03

#define LSM303DLM_ACCELEROMETER_ADDR   0x18
#define LSM303DLM_CTRL_REG1_A          0x20
#define LSM303DLM_CTRL_REG4_A          0x23

#define LSM303DLM_MAGNETOMETER_ADDR    0x1E
#define LSM303DLM_CRA_REG_M            0x00
#define LSM303DLM_MR_REG_M             0x02

/// TWI peripheral redefinition if needed
#if !defined(AT91C_BASE_TWI) && defined(AT91C_BASE_TWI0)
    #define AT91C_BASE_TWI      AT91C_BASE_TWI0
    #define AT91C_ID_TWI        AT91C_ID_TWI0
    #define PINS_TWI            PINS_TWI0
#endif
/// TWI driver instance.
static Twid twid;


//------------------------------------------------------------------------------
/// TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
//------------------------------------------------------------------------------
void ISR_Twi(void)
{
    TWID_Handler(&twid);
}

//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ0 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ0Handler(void)
{
    leftUSSonarPongTrigger = 1;
}


//*----------------------------------------------------------------------------
//* Function Name       : IRQ0Handler
//* Object              : Interrupt Handler called by the IRQ1 interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void IRQ1Handler(void)
{
    rightUSSonarPongTrigger = 1;
}


//*----------------------------------------------------------------------------
//* Function Name       : FIQHandler
//* Object              : Interrupt Handler called by the FIQ interrupt with AT91
//*                       compatibility
//*----------------------------------------------------------------------------
__ramfunc void FIQHandler(void)
{
    if (usSensorsDelayCounter > 25)
    {
        usSensorsPongTrigger = 1;
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

    /*
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, 1 | AT91C_PWMC_CPOL, usSonarServoPeriod, 1);                       //leftUSSonar  PWM0
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 1, 1 | AT91C_PWMC_CPOL, usSonarServoPeriod, 1);                       //rightUSSonar PWM1
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 2, 1 | AT91C_PWMC_CPOL, usSensors40kHzFrequency, usSensors40kHzDuty);    //usSensors    PWM2 - 40kHz
    */
    pwmFreqSet(0, usSonarServoFrequency);
    pwmFreqSet(1, usSonarServoFrequency);

    pwmFreqSet(2, usSensors40kHzFrequency);
    pwmDutySet_u8(2, usSensors40kHzDuty);

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
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);  //leftUSSonar  PWM0
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);  //rightUSSonar PWM1
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);  //usSensors    PWM2

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA0);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA1);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA2);

    // US Sonars configuration
    // US Sonars interrupts
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA20);  // IRQ0
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, AT91C_PIO_PA30);  // IRQ1

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5); //leftUSSonarTrigger
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6); //rightUSSonarTrigger

    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

    //Input channel 1 configuration
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA14);  //U12 A
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA13);  //U12 B
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA24);  //U12 C
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA25);  //U12 D

    // U7 Digital Pot control
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);    //U7 INC
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);    //U7 U/D
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);   //U7 CS

    // U9, U10, U11, U13, U14 Shift registers control pins
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);   //U9, U10, U11, U13, U14 ST_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);   //U9, U10, U11 DS
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);   //U9, U10, U11 U7 SH_CP
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);   //U13, U14 DS
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);   //U13, U14 SH_CP

    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);

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

static void InitTWI()
{
    // Configure TWI
    // In IRQ mode: to avoid problems, the priority of the TWI IRQ must be max.
    // In polling mode: try to disable all IRQs if possible.
    // (in this example it does not matter, there is only the TWI IRQ active)
    /* Configure TWI PIOs */
    AT91F_TWI_CfgPIO();
    /* Configure PMC by enabling TWI clock */
    AT91F_TWI_CfgPMC();
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    TWI_ConfigureMaster(AT91C_BASE_TWI, TWCK, MCK);
    TWID_Initialize(&twid, AT91C_BASE_TWI);
    AIC_ConfigureIT(AT91C_ID_TWI, 0, ISR_Twi);
    AIC_EnableIT(AT91C_ID_TWI);
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
    InitTWI();
}

void setLeds(char * ledsState)
{
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    for (signed char i = 23; i >= 0; i--)
    {
        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        if (ledsState[i] == '1')
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);
        }
        delay_us(50);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA11);
        delay_us(50);
    }
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
}

void setShRegU13U14(char * regState)
{
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA12);
    for (signed char i = 23; i >= 0; i--)
    {
        AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
        if (regState[i] == '1')
        {
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
        }
        else
        {
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA16);
        }
        delay_us(50);
        AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA15);
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

    unsigned int sensorsChannel = 0;
    unsigned int sensorsIRReadings = 0;
    unsigned int sensorsUSReadings = 0;
    unsigned int sensorsIRReading = 0;
    unsigned int sensorsUSReading = 0;
    unsigned int sensorsUSDuty = usSensors40kHzDuty;
    unsigned int sensorsUSBurst = 250;
    unsigned int sensorsUSDelay = 2500;

    unsigned int leftUSSonarServoDuty = 1;
    unsigned int rightUSSonarServoDuty = 1;

    unsigned int leftUSSonarReadings = 0;
    unsigned int rightUSSonarReadings = 0;
    unsigned int leftUSSonarReading = 0;
    unsigned int rightUSSonarReading = 0;

    unsigned int input1Readings = 0;
    unsigned int input1Channel = 0;
    unsigned int input1Reading = 0;

    unsigned int accelerometerReadings = 0;
    int Ax, Ay, Az;

    unsigned int magnetometerReadings = 0;
    int Mx, My, Mz;

    DeviceInit();

    while (1)
    {
        // perform US Sensor measurement
        if (sensorsUSReadings)
        {
            usSensorsDelayCounter = 0;
            usSensorsPongTrigger = 0;

            pwmDutySet_u8(2, sensorsUSDuty);
            AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
            delay_us(sensorsUSBurst);
            AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);
            delay_us(sensorsUSDelay);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);

            while (usSensorsPongTrigger == 0)
            {
                usSensorsDelayCounter++;
                if (usSensorsDelayCounter > 7000)
                    break;
            }
            sensorsUSReading = usSensorsDelayCounter;
            delay_us(2500);
        }

        // perform left US Sonar measurement
        if (leftUSSonarReadings)
        {
            leftUSSonarDelayCounter = 0;
            leftUSSonarPongTrigger = 0;

            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
            delay_us(20);
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

            while (leftUSSonarPongTrigger == 0)
            {
                leftUSSonarDelayCounter++;
                if (leftUSSonarDelayCounter > 50000)
                    break;
            }
            leftUSSonarReading = leftUSSonarDelayCounter;
            delay_us(2500);
        }

        // perform right US Sonar measurement
        if (rightUSSonarReadings)
        {
            rightUSSonarDelayCounter = 0;
            rightUSSonarPongTrigger = 0;

            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);
            delay_us(20);
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA6);

            AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

            while (rightUSSonarPongTrigger == 0)
            {
                rightUSSonarDelayCounter++;
                if (rightUSSonarDelayCounter > 50000)
                    break;
            }
            rightUSSonarReading = rightUSSonarDelayCounter;
            delay_us(2500);
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
                sprintf((char *)msg,"PLATFORM-LOCATION-P1\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
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
            //USSENSORSVOLTAGEUP#1
            //USSENSORSVOLTAGEUP#10
            //USSENSORSVOLTAGEUP#25
            //USSENSORSVOLTAGEUP#50
            //USSENSORSVOLTAGEUP#100
            if (strcmp((char*) cmdParts, "USSENSORSVOLTAGEUP") == 0)
            {
                int steps = atoi(strtok( NULL, "#" ));
                /*
                AT91C_PIO_PA8    //U7 INC
                AT91C_PIO_PA9    //U7 U/D
                AT91C_PIO_PA10   //U7 CS
                */
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
                delay_ms(5);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
                for (int i = 0; i < steps; i++)
                {
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                    delay_ms(5);
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                    delay_ms(5);
                }
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                delay_ms(5);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
                delay_ms(5);
                continue;
            }
            //USSENSORSVOLTAGEDOWN#1
            //USSENSORSVOLTAGEDOWN#10
            //USSENSORSVOLTAGEDOWN#25
            //USSENSORSVOLTAGEDOWN#50
            //USSENSORSVOLTAGEDOWN#100
            if (strcmp((char*) cmdParts, "USSENSORSVOLTAGEDOWN") == 0)
            {
                int steps = atoi(strtok( NULL, "#" ));
                /*
                AT91C_PIO_PA8    //U7 INC
                AT91C_PIO_PA9    //U7 U/D
                AT91C_PIO_PA10   //U7 CS
                */
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA9);
                delay_ms(5);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
                for (int i = 0; i < steps; i++)
                {
                    AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                    delay_ms(5);
                    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                    delay_ms(5);
                }
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA8);
                delay_ms(5);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA10);
                delay_ms(5);
                continue;
            }
            //USSENSORSDUTY#1
            //USSENSORSDUTY#5
            //USSENSORSDUTY#10
            //USSENSORSDUTY#50
            //USSENSORSDUTY#90
            //USSENSORSDUTY#127
            if (strcmp((char*) cmdParts, "USSENSORSDUTY") == 0)
            {
                sensorsUSDuty = atoi(strtok( NULL, "#" ));
                continue;
            }
            //USSENSORSBURST#50
            //USSENSORSBURST#100
            //USSENSORSBURST#150
            //USSENSORSBURST#200
            //USSENSORSBURST#250
            //USSENSORSBURST#300
            //USSENSORSBURST#350
            //USSENSORSBURST#400
            //USSENSORSBURST#450
            //USSENSORSBURST#500
            if (strcmp((char*) cmdParts, "USSENSORSBURST") == 0)
            {
                sensorsUSBurst = atoi(strtok( NULL, "#" ));
                continue;
            }
            //USSENSORSDELAY#250
            //USSENSORSDELAY#500
            //USSENSORSDELAY#1000
            //USSENSORSDELAY#1500
            //USSENSORSDELAY#2000
            //USSENSORSDELAY#2500
            //USSENSORSDELAY#4000
            //USSENSORSDELAY#5000
            if (strcmp((char*) cmdParts, "USSENSORSDELAY") == 0)
            {
                sensorsUSDelay = atoi(strtok( NULL, "#" ));
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
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);
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
            if (strcmp((char*) cmdParts, "DISABLELEFTUSSONARSERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                sprintf((char *)msg,"LEFT US SONAR SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLERIGHTUSSONARSERVO") == 0)
            {
                AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                sprintf((char *)msg,"RIGHT US SONAR SERVO DISABLED\n");
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                continue;
            }
            //SETLEFTUSSONARSERVODUTY#5
            //SETLEFTUSSONARSERVODUTY#8
            //SETLEFTUSSONARSERVODUTY#20
            //SETLEFTUSSONARSERVODUTY#40
            //SETLEFTUSSONARSERVODUTY#14
            if (strcmp((char*) cmdParts, "SETLEFTUSSONARSERVODUTY") == 0)
            {
                leftUSSonarServoDuty = atoi(strtok( NULL, "#" ));
                pwmDutySet_u8(0, leftUSSonarServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
                continue;
            }
            //SETRIGHTUSSONARSERVODUTY#5
            //SETRIGHTUSSONARSERVODUTY#8
            //SETRIGHTUSSONARSERVODUTY#20
            //SETRIGHTUSSONARSERVODUTY#40
            //SETRIGHTUSSONARSERVODUTY#27
            if (strcmp((char*) cmdParts, "SETRIGHTUSSONARSERVODUTY") == 0)
            {
                rightUSSonarServoDuty = atoi(strtok( NULL, "#" ));
                pwmDutySet_u8(1, rightUSSonarServoDuty);
                AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTLEFTUSSONARREADINGS") == 0)
            {
                leftUSSonarReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPLEFTUSSONARREADINGS") == 0)
            {
                leftUSSonarReadings = 0;
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTRIGHTUSSONARREADINGS") == 0)
            {
                rightUSSonarReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPRIGHTUSSONARREADINGS") == 0)
            {
                rightUSSonarReadings = 0;
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
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
            //READSTLM75#1
            //READSTLM75#2
            if (strcmp((char*) cmdParts, "READSTLM75") == 0)
            {
                unsigned char sensorNum = atoi(strtok( NULL, "#" ));
                float TempReg;
                unsigned char pData[2];
                if (sensorNum == 1)
                {
                    TWID_Read(&twid, STLM75_ADDR_S1, STLM75_REG_TEMP, 2, pData, 2, 0);
                }
                if (sensorNum == 2)
                {
                    TWID_Read(&twid, STLM75_ADDR_S2, STLM75_REG_TEMP, 2, pData, 2, 0);
                }
                /* Shift the value and format it */
                TempReg = pData[0] << 1 ;
                TempReg += pData[1] >> 7;
                sprintf((char *)msg,"STLM75 [%d]: %.2f\n", sensorNum, TempReg);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                delay_ms(100);
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTACCELEROMETERREADINGS") == 0)
            {
                unsigned char LSM303DLM_CTRL_REG1_A_VALUE = 0x27;
                unsigned char LSM303DLM_CTRL_REG4_A_VALUE = 0x40;
                TWID_Write(&twid, LSM303DLM_ACCELEROMETER_ADDR, LSM303DLM_CTRL_REG1_A, 1, &LSM303DLM_CTRL_REG1_A_VALUE, 1, 0);
                TWID_Write(&twid, LSM303DLM_ACCELEROMETER_ADDR, LSM303DLM_CTRL_REG4_A, 1, &LSM303DLM_CTRL_REG4_A_VALUE, 1, 0);
                accelerometerReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPACCELEROMETERREADINGS") == 0)
            {
                accelerometerReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "STARTMAGNETOMETERREADINGS") == 0)
            {
                unsigned char LSM303DLM_CRA_REG_M_VALUE = 0x14;
                unsigned char LSM303DLM_MR_REG_M_VALUE  = 0x00;
                TWID_Write(&twid, LSM303DLM_MAGNETOMETER_ADDR, LSM303DLM_CRA_REG_M, 1, &LSM303DLM_CRA_REG_M_VALUE, 1, 0);
                TWID_Write(&twid, LSM303DLM_MAGNETOMETER_ADDR, LSM303DLM_MR_REG_M, 1, &LSM303DLM_MR_REG_M_VALUE, 1, 0);
                magnetometerReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPMAGNETOMETERREADINGS") == 0)
            {
                magnetometerReadings = 0;
                continue;
            }
            //SETLEDS#000000000000000000000000
            //SETLEDS#111111111111111111111111
            //SETLEDS#010101010101010101010101
            //SETLEDS#101010101010101010101010
            if (strcmp((char*) cmdParts, "SETLEDS") == 0)
            {
                char * ledsState = strtok( NULL, "#" );
                sprintf((char *)msg,"LEDS STATE: %s\n", ledsState);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                setLeds(ledsState);
                continue;
            }
            //SETSHREG#0000000000000000
            //SETSHREG#1111000000000000
            //SETSHREG#1010000000000000
            //SETSHREG#0101000000000000
            //SETSHREG#1100000000000000
            //SETSHREG#0011000000000000
            //SETSHREG#1111111111111111
            if (strcmp((char*) cmdParts, "SETSHREG") == 0)
            {
                char * regState = strtok( NULL, "#" );
                sprintf((char *)msg,"SHIFT REG STATE: %s\n", regState);
                pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
                setShRegU13U14(regState);
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
            if (sensorsUSReading > 7000)
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
        if (leftUSSonarReadings)
        {
            if (leftUSSonarReading > 50000)
            {
                sprintf((char *)msg,"LEFT US SONAR: FAILED\n");
            }
            else
            {
                sprintf((char *)msg,"LEFT US SONAR: %u\n", leftUSSonarReading);
            }
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (rightUSSonarReadings)
        {
            if (rightUSSonarReading > 50000)
            {
                sprintf((char *)msg,"RIGHT US SONAR: FAILED\n");
            }
            else
            {
                sprintf((char *)msg,"RIGHT US SONAR: %u\n", rightUSSonarReading);
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
        if (accelerometerReadings)
        {
            unsigned char ACC_Data[6];
            for(int r = 0; r < 6; r++)
            {
                ACC_Data[r] = '\0';
            }

            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x28, 1, &ACC_Data[0], 1, 0); //read OUT_X_L_A (MSB)
            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x29, 1, &ACC_Data[1], 1, 0); //read OUT_X_H_A (LSB)
            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x2A, 1, &ACC_Data[2], 1, 0); //read OUT_Y_L_A (MSB)
            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x2B, 1, &ACC_Data[3], 1, 0); //read OUT_Y_H_A (LSB)
            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x2C, 1, &ACC_Data[4], 1, 0); //read OUT_Z_L_A (MSB)
            TWID_Read(&twid, LSM303DLM_ACCELEROMETER_ADDR, 0x2D, 1, &ACC_Data[5], 1, 0); //read OUT_Z_H_A (LSB)

            Ax = (int) (ACC_Data[0] << 8) + ACC_Data[1];
            Ay = (int) (ACC_Data[2] << 8) + ACC_Data[3];
            Az = (int) (ACC_Data[4] << 8) + ACC_Data[5];

            sprintf((char *)msg,"ACCELEROMETER [X,Y,Z]: %d, %d, %d\n", Ax, Ay, Az);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);
        }
        if (magnetometerReadings)
        {
            unsigned char MR_REG_M = 0;
            unsigned char MR_Data[6];
            for(int r = 0; r < 6; r++)
            {
                MR_Data[r] = '\0';
            }
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x02, 1, &MR_REG_M, 1, 0);    //read MR_REG_M
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x03, 1, &MR_Data[0], 1, 0);  //read OUT_X_H_M (MSB)
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x04, 1, &MR_Data[1], 1, 0);  //read OUT_X_L_M (LSB)
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x05, 1, &MR_Data[2], 1, 0);  //read OUT_Y_H_M (MSB)
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x06, 1, &MR_Data[3], 1, 0);  //read OUT_Y_L_M (LSB)
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x07, 1, &MR_Data[4], 1, 0);  //read OUT_Z_H_M (MSB)
            TWID_Read(&twid, LSM303DLM_MAGNETOMETER_ADDR, 0x08, 1, &MR_Data[5], 1, 0);  //read OUT_Z_L_M (LSB)
            Mx = (int) (MR_Data[0] << 8) + MR_Data[1];
            My = (int) (MR_Data[2] << 8) + MR_Data[3];
            Mz = (int) (MR_Data[4] << 8) + MR_Data[5];

            sprintf((char *)msg,"MAGNETOMETER [X,Y,Z]: %d, %d, %d\n", Mx, My, Mz);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(100);

            delay_ms(100);
        }
    }

    return 0; /* never reached */
}

