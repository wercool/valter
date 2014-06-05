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
unsigned int leftWheelEncoderPositive = 1;
unsigned int rightWheelEncoderTicks = 0;
unsigned int rightWheelEncoderPositive = 1;

unsigned char timerLeft = 0;
unsigned char timerRight = 0;


unsigned int servoSignalPeriod = 0;
unsigned int servoSignalWidth = 0;
unsigned int radarServoRotation = 0;
unsigned char radarServoSet = 0;

volatile unsigned int resetCntStart = 0;
volatile unsigned int resetCnt = 0;


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
        AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
        if (leftWheelEncoderPositive)
        {
            AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ0Handler);
            leftWheelEncoderPositive = 0;
        }
        else
        {
            AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ0Handler);
            leftWheelEncoderPositive = 1;
        }
        AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
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
        AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
        if (rightWheelEncoderPositive)
        {
            AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) IRQ1Handler);
            rightWheelEncoderPositive = 0;
        }
        else
        {
            AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ1Handler);
            rightWheelEncoderPositive = 1;
        }
        AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
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
    volatile AT91PS_TC pTC = AT91C_BASE_TC0; /* pointer to timer channel 0 register structure */
    volatile unsigned int dummy; /* temporary */

    /* read TC0 Status Register to clear interrupt */
    dummy = pTC->TC_SR;
    //* Suppress warning variable "dummy" was set but never used
    dummy = dummy;

    if (radarServoSet)
    {
        if (servoSignalPeriod < 1000)
        {
            servoSignalPeriod++;
        }
        else
        {
            if (servoSignalWidth < radarServoRotation / 20)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
                servoSignalWidth++;
            }
            else
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
                servoSignalWidth = 0;
                servoSignalPeriod = 0;
            }
        }
    }
}

__ramfunc void SoftIQRHandlerTC1(void)
{
    AT91PS_TC TC_pt = AT91C_BASE_TC1;
    unsigned int dummy;
    //* Acknowledge interrupt status
    dummy = TC_pt->TC_SR;
    //* Suppress warning variable "dummy" was set but never used
    dummy = dummy;

    if (timerLeft == 0)
        timerLeft = 1;

    if (timerRight == 0)
        timerRight = 1;

    if (resetCntStart)
    {
        resetCnt++;
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

    pwmFreqSet(0, 12000);
    pwmFreqSet(1, 12000);
    pwmFreqSet(2, 12000);
    pwmFreqSet(3, 12000);

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

    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);        //Alarm
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);


    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);        //Encoders enable
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);

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

static void Timer0Setup()
{
    /* enable the Timer0 peripheral clock */
    volatile AT91PS_PMC pPMC = AT91C_BASE_PMC;
    pPMC->PMC_PCER = pPMC->PMC_PCSR | (1 << AT91C_ID_TC0);
    /* Set up the AIC registers for Timer 0 */
    volatile AT91PS_AIC pAIC = AT91C_BASE_AIC;
    /* Disable timer 0 interrupt */
    /* in AIC Interrupt Disable Command Register */
    pAIC->AIC_IDCR = (1 << AT91C_ID_TC0);
    /* Set the TC0 IRQ handler address in */
    /* AIC Source Vector Register[12] */
    pAIC->AIC_SVR[AT91C_ID_TC0] = (unsigned int) SoftIQRHandlerTC0;
    /* Set the interrupt source type and priority */
    /* in AIC Source Mode Register[12] */
    pAIC->AIC_SMR[AT91C_ID_TC0] = (AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 0x4);
    /* Clear the TC0 interrupt */
    /* in AIC Interrupt Clear Command Register */
    pAIC->AIC_ICCR = (1 << AT91C_ID_TC0);
    /* Remove disable timer 0 interrupt */
    /* in AIC Interrupt Disable Command Reg */
    pAIC->AIC_IDCR = (0 << AT91C_ID_TC0);
    /* Enable the TC0 interrupt */
    /* in AIC Interrupt Enable Command Register */
    pAIC->AIC_IECR = (1 << AT91C_ID_TC0);
    /* Setup timer0 to generate a 1 msec periodic interrupt */


    /* TC Block Control Register TC_BCR (read/write) */
    /* */
    /* |------------------------------------------------------------------|------| */
    /* | SYNC | */
    /* |------------------------------------------------------------------|------| */
    /* 31 1 0 */
    /* */
    /* SYNC = 0 (no effect) <===== take default */
    /* SYNC = 1 (generate software trigger for all 3 timer channels simultaneously) */
    /* */
    /* create a pointer to TC Global Register structure */
    AT91PS_TCB pTCB = AT91C_BASE_TCB;
    /* SYNC trigger not used */
    pTCB->TCB_BCR = 0;

    /* TC Block Mode Register TC_BMR (read/write) */
    /* */
    /* |-------------------------------------|-----------|-----------|-----------| */
    /* | TC2XC2S TCXC1S TC0XC0S | */
    /* |-------------------------------------|-----------|-----------|-----------| */
    /* 31 5 4 3 2 1 0 */
    /* */
    /* TC0XC0S Select = 00 TCLK0 (PA4) */
    /* = 01 none <===== we select this one */
    /* = 10 TIOA1 (PA15) */
    /* = 11 TIOA2 (PA26) */
    /* */
    /* TCXC1S Select = 00 TCLK1 (PA28) */
    /* = 01 none <===== we select this one */
    /* = 10 TIOA0 (PA15) */
    /* = 11 TIOA2 (PA26) */
    /* */
    /* TC2XC2S Select = 00 TCLK2 (PA29) */
    /* = 01 none <===== we select this one */
    /* = 10 TIOA0 (PA00) */
    /* = 11 TIOA1 (PA26) */
    /* */
    /* external clocks not used */
    pTCB->TCB_BMR = 0x15;


    /* TC Channel Control Register TC_CCR (read/write) */
    /* */
    /* |----------------------------------|--------------|------------|-----------| */
    /* | SWTRG CLKDIS CLKENS | */
    /* |----------------------------------|--------------|------------|-----------| */
    /* 31 2 1 0 */
    /* */
    /* CLKEN = 0 no effect */
    /* CLKEN = 1 enables the clock <===== we select this one */
    /* */
    /* CLKDIS = 0 no effect <===== take default */
    /* CLKDIS = 1 disables the clock */
    /* */
    /* SWTRG = 0 no effect */
    /* SWTRG = 1 software trigger aserted counter reset and clock starts <===== we select this one */
    /* */
    /* create a pointer to channel 0 Register structure */
    AT91PS_TC pTC = AT91C_BASE_TC0;
    /* enable the clock and start it */
    pTC->TC_CCR = 0x5;

    /* TC Channel Mode Register TC_CMR (read/write) */
    /* */
    /* |-----------------------------------|------------|---------------| */
    /* | LDRB LDRA | */
    /* |-----------------------------------|------------|---------------| */
    /* 31 19 18 17 16 */
    /* */
    /* |----------|---------|--------------|------------|---------------| */
    /* |WAVE = 0 CPCTRG ABETRG ETRGEDG | */
    /* |----------|---------|--------------|------------|---------------| */
    /* 15 14 13 11 10 9 8 */
    /* */
    /* |----------|---------|--------------|------------|---------------| */
    /* | LDBDIS LDBSTOP BURST CLKI TCCLKS | */
    /* |----------|---------|--------------|------------|---------------| */
    /* 7 6 5 4 3 2 0 */
    /* */
    /* CLOCK SELECTION */
    /* TCCLKS = 000 TIMER_CLOCK1 (MCK/2 = 24027420 hz) */
    /* 001 TIMER_CLOCK2 (MCK/8 = 6006855 hz) */
    /* 010 TIMER_CLOCK3 (MCK/32 = 1501713 hz) */
    /* 011 TIMER_CLOCK4 (MCK/128 = 375428 hz) */
    /* 100 TIMER_CLOCK5 (MCK/1024 = 46928 hz) <===== we select this one */
    /* 101 XC0 */
    /* 101 XC1 */
    /* 101 XC2 */
    /* */
    /* CLOCK INVERT */
    /* CLKI = 0 counter incremented on rising clock edge <===== we select this one */
    /* CLKI = 1 counter incremented on falling clock edge */
    /* */
    /* BURST SIGNAL SELECTION */
    /* BURST = 00 clock is not gated by any external system <===== take default */
    /* 01 XC0 is anded with the clock */
    /* 10 XC1 is anded with the clock */
    /* 11 XC2 is anded with the clock */
    /* */
    /* COUNTER CLOCK STOPPED WITH RB LOADING */
    /* LDBSTOP = 0 counter clock is not stopped when RB loading occurs <===== take default */
    /* = 1 counter clock is stopped when RB loading occur */
    /* */
    /* COUNTER CLOCK DISABLE WITH RB LOADING */
    /* LDBDIS = 0 counter clock is not disabled when RB loading occurs <===== take default */
    /* = 1 counter clock is disabled when RB loading occurs */
    /* */
    /* EXTERNAL TRIGGER EDGE SELECTION */
    /* ETRGEDG = 00 (none) <===== take default */
    /* 01 (rising edge) */
    /* 10 (falling edge) */
    /* 11 (each edge) */
    /* */
    /* TIOA OR TIOB EXTERNAL TRIGGER SELECTION */
    /* ABETRG = 0 (TIOA is used) <===== take default */
    /* 1 (TIOB is used) */
    /* */
    /* RC COMPARE TRIGGER ENABLE */
    /* CPCTRG = 0 (RC Compare has no effect on the counter and its clock) */
    /* 1 (RC Compare resets the counter and starts the clock) <===== we select this one */
    /* */
    /* WAVE */
    /* WAVE = 0 Capture Mode is enabled <===== we select this one */
    /* 1 Waveform Mode is enabled */
    /* */
    /* RA LOADING SELECTION */
    /* LDRA = 00 none) <===== take default */
    /* 01 (rising edge of TIOA) */
    /* 10 (falling edge of TIOA) */
    /* 11 (each edge of TIOA) */
    /* */
    /* RB LOADING SELECTION */
    /* LDRB = 00 (none) <===== take default */
    /* 01 (rising edge of TIOA) */
    /* 10 (falling edge of TIOA) */
    /* 11 (each edge of TIOA) */
    /* */
    /* TCCLKS = 1 (TIMER_CLOCK5) */
    /* CPCTRG = 1 (RC Compare resets the counter and restarts the clock) */
    /* WAVE = 0 (Capture mode enabled) */
    pTC->TC_CMR = 0x4004;

    /* TC Register C TC_RC (read/write) Compare Register 16-bits */
    /* */
    /* |----------------------------------|----------------------------------------| */
    /* | not used RC | */
    /* |----------------------------------|----------------------------------------| */
    /* 31 16 15 0 */
    /* */
    /* Timer Calculation: What count gives 1 msec time-out? */
    /* */
    /* TIMER_CLOCK5 = MCK / 1024 = 48054841 / 1024 = 46928 hz */
    /* */
    /* TIMER_CLOCK5 Period = 1 / 46928 = 21.309239686 microseconds */
    /* */
    /* A little algebra: .001 sec = count * 21.3092396896*10**-6 */
    /* count = .001 / 21.3092396896*10**-6 */
    /* count = 46.928 */
    /* */
    /* STK: Even Simpler, let the compiler do the work: */
    /* */
    /* TIMER_CLOCK5 = (MCK / 1024) / 1000 */
    /* = 48054841 / 1024 / 1000 = 46.928 */


    //MASKA
    /* Timer Calculation: What count gives 100 msec time-out? */
    /* */
    /* TIMER_CLOCK5 = MCK / 128 = 48054841 / 1024 = 375428 hz */
    /* */
    /* TIMER_CLOCK5 Period = 1 / 375428 = 2.663626581 microseconds */
    /* */
    /* A little algebra: .000001 sec (1 msec) = count * 2.663626581*10**-6 */
    /* count = .000001 / 2.663626581*10**-6 */
    /* count = 1 */
    /* */

    pTC->TC_RC = 1;

    /* TC Interrupt Enable Register TC_IER (write-only) */
    /* */
    /* */
    /* |------------|-------|-------|-------|-------|--------|--------|--------|--------| */
    /* | ETRGS LDRBS LDRAS CPCS CPBS CPAS LOVRS COVFS | */
    /* |------------|-------|-------|-------|-------|--------|--------|--------|--------| */
    /* 31 8 7 6 5 4 3 2 1 0 */
    /* */
    /* COVFS = 0 no effect <===== take default */
    /* 1 enable counter overflow interrupt */
    /* */
    /* LOVRS = 0 no effect <===== take default */
    /* 1 enable load overrun interrupt */
    /* */
    /* CPAS = 0 no effect <===== take default */
    /* 1 enable RA compare interrupt */
    /* */
    /* CPBS = 0 no effect <===== take default */
    /* 1 enable RB compare interrupt */
    /* */
    /* CPCS = 0 no effect */
    /* 1 enable RC compare interrupt <===== we select this one */
    /* */
    /* LDRAS = 0 no effect <===== take default */
    /* 1 enable RA load interrupt */
    /* */
    /* LDRBS = 0 no effect <===== take default */
    /* 1 enable RB load interrupt */
    /* */
    /* ETRGS = 0 no effect <===== take default */
    /* 1 enable External Trigger interrupt */
    /* */
    /* enable RC compare interrupt */
    pTC->TC_IER = 0x10;

    /* TC Interrupt Disable Register TC_IDR (write-only) */
    /* */
    /* */
    /* |------------|-------|-------|-------|-------|--------|--------|--------|--------| */
    /* | ETRGS LDRBS LDRAS CPCS CPBS CPAS LOVRS COVFS | */
    /* |------------|-------|-------|-------|-------|--------|--------|--------|--------| */
    /* 31 8 7 6 5 4 3 2 1 0 */
    /* */
    /* COVFS = 0 no effect */
    /* 1 disable counter overflow interrupt <===== we select this one */
    /* */
    /* LOVRS = 0 no effect */
    /* 1 disable load overrun interrupt <===== we select this one */
    /* */
    /* CPAS = 0 no effect */
    /* 1 disable RA compare interrupt <===== we select this one */
    /* */
    /* CPBS = 0 no effect */
    /* 1 disable RB compare interrupt <===== we select this one */
    /* */
    /* CPCS = 0 no effect <===== take default */
    /* 1 disable RC compare interrupt */
    /* */
    /* LDRAS = 0 no effect */
    /* 1 disable RA load interrupt <===== we select this one */
    /* */
    /* LDRBS = 0 no effect */
    /* 1 disable RB load interrupt <===== we select this one */
    /* */
    /* ETRGS = 0 no effect */
    /* 1 disable External Trigger interrupt <===== we select this one */
    /* */
    /* disable all except RC compare interrupt */
    pTC->TC_IDR = 0xEF;
}

static void InitIRQ()
{
    // IRQ0 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA20, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ0, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ0Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);

    // IRQ1 initialization
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA30, 0);
    AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_IRQ1, AT91C_AIC_PRIOR_HIGHEST, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void(*)(void)) IRQ1Handler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

    // FIQ initialization
    //AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, AT91C_PIO_PA19, 0);
    //AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, (void(*)(void)) FIQHandler);
    //AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_FIQ);

    Timer0Setup();

////    Open timer1
//    AT91F_TC_Open(AT91C_BASE_TC1, TC_CLKS_MCK8, AT91C_ID_TC1);
//    //Open Timer 1 interrupt
//    AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_TC1, TIMER1_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SoftIQRHandlerTC1);
//    AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;             // IRQ enable CPC
//    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_TC1);
//    //Start timer1
//    AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG;





    // Init timer1 (USE THIS!!!)
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC1;         // Enable peripheral clock
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;             // Disable TC clock
    AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;                  // Disable interrupts
    unsigned int dummy = AT91C_BASE_TC1->TC_SR;               // Clear status register
    dummy = dummy;
    AT91C_BASE_TC1->TC_CMR = TC_CLKS_MCK32 | AT91C_TC_CPCTRG;    // Set mode

    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN ;             // Enable the Clock counter
    AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;

    AT91C_BASE_AIC->AIC_IDCR = 0x1 << AT91C_ID_TC1;
    AT91C_BASE_AIC->AIC_SVR[AT91C_ID_TC1] = (unsigned int) SoftIQRHandlerTC1;
    AT91C_BASE_AIC->AIC_SMR[AT91C_ID_TC1] = AT91C_AIC_SRCTYPE_EXT_LOW_LEVEL | 4;
    AT91C_BASE_AIC->AIC_ICCR = 0x1 << AT91C_ID_TC1;
    AT91C_BASE_AIC->AIC_IECR = 0x1 << AT91C_ID_TC1;

    AT91C_BASE_TC1->TC_RC = 100000;
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG;

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

    unsigned int distanceMeterReadings = 0;
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

        if (resetCnt > 150)
        {
            resetCntStart = 0;

            resetCnt = 0;

            //beep
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
            delay_ms(5);
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);

            //LEFTSTOP
            pwmDutySetPercent(0, 1);
            pwmDutySetPercent(3, 1);
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);        //Left Wheels INa
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);       //Left Wheels INb

            //RIGHTSTOP
            pwmDutySetPercent(1, 1);
            pwmDutySetPercent(2, 1);
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);       //Right Wheels INa
            AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);       //Right Wheels INb

            //RADARROTATIONRESET
            radarServoSet = 0;
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
            servoSignalPeriod = 0;
            servoSignalWidth = 0;

            //DISABLEENCODERS
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
            AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
            AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

            //LIGHTSOFF
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);

            //ALARMOFF
            AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);

            //info readings
            batteryVoltageReadings = 0;
            rightFrontCurReadings = 0;
            rightRearCurReadings = 0;
            leftFrontCurReadings = 0;
            leftRearCurReadings = 0;
            leftWheelEncoderReadings = 0;
            rightWheelEncoderReadings = 0;
        }

        cdcMessageObj = getCDCMEssage();
        if (cdcMessageObj.length > 0)
        {
            //sprintf((char *)msg,"MSG:%s\n", cdcMessageObj.data);
            //pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));

            char *cmdParts;
            cmdParts = strtok((char*) cdcMessageObj.data, "#" );

            if (strcmp((char*) cmdParts, "PING") == 0)
            {
                resetCnt = 0;
                resetCntStart = 1;
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
            //RADARROTATIONSET#700
            //RADARROTATIONSET#750
            //RADARROTATIONSET#800
            //RADARROTATIONSET#850
            //RADARROTATIONSET#900
            //RADARROTATIONSET#950
            //RADARROTATIONSET#1000
            //RADARROTATIONSET#1050
            //RADARROTATIONSET#1100
            //RADARROTATIONSET#1150
            //RADARROTATIONSET#1200
            //RADARROTATIONSET#1250
            //RADARROTATIONSET#1300
            //RADARROTATIONSET#1350
            //RADARROTATIONSET#1400
            //RADARROTATIONSET#1450 //CENTER
            //RADARROTATIONSET#1500
            //RADARROTATIONSET#1550
            //RADARROTATIONSET#1600
            //RADARROTATIONSET#1650
            //RADARROTATIONSET#1700
            //RADARROTATIONSET#1750
            //RADARROTATIONSET#1800
            //RADARROTATIONSET#1850
            //RADARROTATIONSET#1900
            //RADARROTATIONSET#1950
            //RADARROTATIONSET#2000
            //RADARROTATIONSET#2050
            //RADARROTATIONSET#2100
            //RADARROTATIONSET#2150
            //RADARROTATIONSET#2200

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
            if (strcmp((char*) cmdParts, "STARTDISTANCEREADINGS") == 0)
            {
                distanceMeterReadings = 1;
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPDISTANCEREADINGS") == 0)
            {
                distanceMeterReadings = 0;
                continue;
            }
            if (strcmp((char*) cmdParts, "ENABLEENCODERS") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
                AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
                continue;
            }
            if (strcmp((char*) cmdParts, "DISABLEENCODERS") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
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
            if (strcmp((char*) cmdParts, "ALARMON") == 0)
            {
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
            if (strcmp((char*) cmdParts, "ALARMOFF") == 0)
            {
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
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
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                delay_ms(alarmBeepDuration);
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);
                continue;
            }
            if (strcmp((char*) cmdParts, "STOPALL") == 0)
            {
                //LEFTSTOP
                pwmDutySetPercent(0, 1);
                pwmDutySetPercent(3, 1);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA3);        //Left Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA31);       //Left Wheels INb

                //RIGHTSTOP
                pwmDutySetPercent(1, 1);
                pwmDutySetPercent(2, 1);
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA29);       //Right Wheels INa
                AT91F_PIO_SetOutput(AT91C_BASE_PIOA, AT91C_PIO_PA28);       //Right Wheels INb

                //RADARROTATIONSET
                radarServoRotation = 1450;
                radarServoSet = 1;

                delay_ms(500);

                //RADARROTATIONRESET
                radarServoSet = 0;
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA27);
                servoSignalPeriod = 0;
                servoSignalWidth = 0;

                //DISABLEENCODERS
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA5);
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);

                //LIGHTSOFF
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA4);

                //ALARMOFF
                AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, AT91C_PIO_PA21);

                //info readings
                batteryVoltageReadings = 0;
                rightFrontCurReadings = 0;
                rightRearCurReadings = 0;
                leftFrontCurReadings = 0;
                leftRearCurReadings = 0;
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ0);
                leftWheelEncoderReadings = 0;
                AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_IRQ1);
                rightWheelEncoderReadings = 0;
                main();
                break;
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
        if (distanceMeterReadings)
        {
            distanceMeterReading = getValueChannel5();
            sprintf((char *)msg,"DISTANCE: %u\n", distanceMeterReading);
            pCDC.Write(&pCDC, (char *)msg, strlen((char *)msg));
            delay_ms(10);
            continue;
        }
    }

    return 0; /* never reached */
}

