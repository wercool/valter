//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : main.c
//* Object              : main application written in C
//* Creation            : JPP   16/Jun/2004
//*----------------------------------------------------------------------------

// Include Standard files
#include "stdio.h"
#include "string.h"

#include "fastfunc.h"
#include "Board.h"
#include "cdc_enumerate.h"
#include "adc.h"
#include "system.h"
#include "mmc.h"

/* Global variables */
#define SPEED       (MCKKHz/10)
unsigned int LedSpeed = SPEED *50;
#define MSG_SIZE        1000
extern void Usart_init ( void );
extern void AT91F_US_Put( char *buffer); // \arg pointer to a string ending by \0
extern void Trace_Toggel_LED( unsigned int led);
unsigned char res[MSG_SIZE];
unsigned char pos[MSG_SIZE];
unsigned char cmd[MSG_SIZE];
unsigned char msg[MSG_SIZE];

unsigned char settings[128];

int maxPos = 900;
int minPos = 300;
int curPos = 600;

int maxForce = 100000;
int minForce = 66000;
int curForce = 66000;

int pressure;
int position;
int direction;
unsigned char reading = 0;
unsigned char dataReading = 0;


//AAT Settings
unsigned char aatDiag = 0;
unsigned char aatDiagStep = 0;
int minDiagForce = 90000;
int lockDiagForce = 100000;
int lockDiagPressure = 0;
int diagForce = 80000;

int averager = 0;
int averager_force = 0;

int test_read = 0;
int set_zero = 0;

int green_led_off = 0;
int green_led_on = 0;

char CDC = 0;

long int cnt = 0;

// external buffer which is use to read/write in MMC card
extern unsigned char mmc_buffer[512];
int mmc_buffer_index = 0;
int mmc_block_index = 0;
unsigned char mmc_buffer_out[512];

unsigned char storingOnMMC = 0;


// We will store our data in the last page of Flash.
#define OUR_FLASH_ADDR (AT91C_IFLASH + AT91C_IFLASH_SIZE - AT91C_IFLASH_PAGE_SIZE)
#define AT91C_MC_WRITE_KEY  ((unsigned)0x5A << 24) // Value to enable flash commands.

// ***************************************************************
// ** flashInit - Sets up the flash writing peripheral. Call this
// **             before you do any flash writes.
// **


extern void init_extint (void);


void flashInit()
{
// Get base location of memory controller peripheral.
    volatile AT91PS_MC mc = AT91C_BASE_MC;

// Enable the auto-erase feature and set up flash write timing.
    mc->MC_FMR = AT91C_MC_FWS_1FWS | (1 + (((MCK * 15) / 10000000)) << 16);
}


struct _AT91S_CDC   pCDC;

void Init_PWM(void)
{
  AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
  AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );
  AT91F_PWMC_CH0_CfgPIO();
  AT91F_PWMC_CfgPMC();    
  AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
  AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, 1 | AT91C_PWMC_CPOL, 100000, 66000);
  AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0, 0);
  AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
  AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, 0, AT91C_PA23_PWM0);
  AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_Open
//* \brief This function Open the USB device
//*----------------------------------------------------------------------------
void AT91F_USB_Open(void)
{
    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;

    // Specific Chip USB Initialisation
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);

    // Enable UDP PullUp (USB_DP_PUP) : enable & Clear of the corresponding PIO
    // Set in PIO mode and Configure in Output
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA,AT91C_PIO_PA16);
    // Clear for set the Pul up resistor
    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA,AT91C_PIO_PA16);

    // CDC Open by structure initialization
    AT91F_CDC_Open(&pCDC, AT91C_BASE_UDP);
}

//*--------------------------------------------------------------------------------------
//* Function Name       : change_speed
//* Object              : Adjust "LedSpeed" value depending on SW1 and SW2 are pressed or not
//* Input Parameters    : none
//* Output Parameters   : Update of LedSpeed value.
//*--------------------------------------------------------------------------------------
static void change_speed ( void )
{//* Begin
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & SW1_MASK) == 0 )
    {
        if ( LedSpeed > SPEED ) LedSpeed -=SPEED ;
    }
    if ( (AT91F_PIO_GetInput(AT91C_BASE_PIOA) & SW2_MASK) == 0 )
    {
        if ( LedSpeed < MCK ) LedSpeed +=SPEED ;
    }
}//* End

void clearLEDs()
{
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
}

void blinkingGreen(int delay)
{
    /*LED3 - GREEN*/
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3);
    Delay(delay);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
    Delay(delay);
}

void startBlinking(int delay)
{
    /* START BLINKING LED */
    /*LED1 - RED*/
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1);
    Delay(delay);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
    Delay(delay);
    /*LED2 - YELLOW*/
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2);
    Delay(delay);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
    Delay(delay);
    /*LED3 - GREEN*/
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3);
    Delay(delay);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
    Delay(delay);
}

void Delay(delayVal)
{
  volatile unsigned int waiting_time ;
  for(waiting_time = 0; waiting_time < delayVal; waiting_time++) ;
}

void setForce(int force)
{
    AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, 1 | AT91C_PWMC_CPOL, 100000, force);
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0, 0);
    AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);
}

void liftUp(int force)
{
    setForce(force);
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, DIRC);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, DIRD);
    direction = -1;
}

void lowerDown(int force)
{
    setForce(force);
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, DIRD);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, DIRC);
    direction = 1;
}

void releasePosition()
{
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, DIRD);
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, DIRC);
}

void setPosition(int force)
{
    setForce(force);
    if (abs(curPos - position) > 5)
    {
        if (curPos > position)
        {
            lowerDown(force);
        }
        if (curPos < position)
        {
            liftUp(force);
        }
    }
    else
    {
        releasePosition();
    }
}

void printTrace(char * trace)
{
    if (dataReading == 0)
    {
        if (CDC == 1)
        {
            pCDC.Write(&pCDC, trace, strlen(trace));
        }
    }
}

void clearMMCCard()
{
    if (initMMC() == MMC_SUCCESS) // card found
    {
      //card_state |= 1;
      memset(&mmc_buffer,0,512);
      mmcReadRegister (10, 16);
      mmc_buffer[7]=0;

      /*Clear first 1000 blocks*/
      int blockClearCnt;
      for (blockClearCnt = 0; blockClearCnt < 2048; blockClearCnt++)
      {
          memset(&mmc_buffer,'0',512);
          mmcWriteBlock(512 * blockClearCnt);
      }

      /*
      // Fill first Block (0) with 'A'
      memset(&mmc_buffer,'0',512);    //set breakpoint and trace mmc_buffer contents
      mmcWriteBlock(0);
      // Fill second Block (1)-AbsAddr 512 with 'B'
      memset(&mmc_buffer,'1',512);
      mmcWriteBlock(512);

      // Read first Block back to buffer
      memset(&mmc_buffer,0x00,512);
      mmcReadBlock(0,512);

      // Read first Block back to buffer
      memset(&mmc_buffer,0x00,512);
      mmcReadBlock(512,512);
      */
    }
}

void parseSettings(char data[MSG_SIZE])
{
    int i;
    int  settings_size;

    char cmdDelim[] = "#";
    char *cmdParts = NULL;

    cmdParts = strtok( data, cmdDelim );

    if (strcmp(cmdParts, "TEST") == 0)
    {
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "TEST") == 0)
        {
            //INITIAL TEST
            printTrace("INITIAL TEST\r\n");
            for (i = 0; i < 15; i++)
            {
                startBlinking(50000);
            }
        }
        if (strcmp(cmdParts, "START") == 0)
        {
            printTrace("TEST READING ACTIVATED\r\n");
            test_read = 1;
        }
        if (strcmp(cmdParts, "STOP") == 0)
        {
            printTrace("TEST READING STOPPED\r\n");
            test_read = 0;
        }
    }

    if (strcmp(cmdParts, "ZERO") == 0)
    {
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "SET") == 0)
        {
            set_zero = pressure;
        }
        if (strcmp(cmdParts, "UNSET") == 0)
        {
            set_zero = 0;
        }
    }
    /*
    //AAT process
    if (strcmp(cmdParts, "DIAG") == 0)
    {
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "START") == 0)
        {
            aatDiag = 1;
            aatDiagStep = 0;
        }
        if (strcmp(cmdParts, "STOP") == 0)
        {
            aatDiag = 0;
        }
        if (strcmp(cmdParts, "STEP4") == 0)
        {
            printTrace("DIAG#3#DONE\r\n");
            AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3);
            aatDiagStep = 4;
            diagForce = maxForce;
        }
    }
    if (strcmp(cmdParts, "POSITION") == 0)
    {
        // POSITION SETTING
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "MAX") == 0)
        {
            //MAX POSITION
            maxPos = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"MAX POSITION IS SET TO: %d\r\n", maxPos);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "MIN") == 0)
        {
            //MIN POSITION
            minPos = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"MIN POSITION IS SET TO: %d\r\n", minPos);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "SET") == 0)
        {
            //CUR POSITION
            curPos = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"CURRENT POSITION IS SET TO: %d\r\n", curPos);
            printTrace(msg);
        }
    }
    if (strcmp(cmdParts, "FORCE") == 0)
    {
        // FORCE SETTING
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "MAX") == 0)
        {
            //MAX FORCE
            maxForce = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"MAX FORCE IS SET TO: %d\r\n", maxForce);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "MIN") == 0)
        {
            //MIN FORCE
            minForce = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"MIN FORCE IS SET TO: %d\r\n", minForce);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "SET") == 0)
        {
            //CUR FORCE
            curForce = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"CURRENT FORCE IS SET TO: %d\r\n", curForce);
            printTrace(msg);
        }
        //AAT process
        if (strcmp(cmdParts, "DIAGMIN") == 0)
        {
            //CUR FORCE
            minDiagForce = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"MINIMUM DIAG FORCE IS SET TO: %d\r\n", minDiagForce);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "DIAGLOCK") == 0)
        {
            //CUR FORCE
            lockDiagForce = atoi(strtok( NULL, cmdDelim ));
            sprintf((char *)msg,"LOCK DIAG FORCE IS SET TO: %d\r\n", lockDiagForce);
            printTrace(msg);
        }
    }
    //AAT setting
    if (strcmp(cmdParts, "PRESSURE") == 0)
    {
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "SETLOCK") == 0)
        {
            //LOCK PRESSURE
            lockDiagPressure = pressure;
            sprintf((char *)msg,"LOCK PRESSURE IS SET TO: %d\r\n", lockDiagPressure);
            printTrace(msg);
        }
    }
    if (strcmp(cmdParts, "SETTINGS") == 0)
    {
        //SETTINGS
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "GET") == 0)
        {
            sprintf((char *)msg,"SETTINGS: %-80s\r\n", OUR_FLASH_ADDR);
            printTrace(msg);
        }
        if (strcmp(cmdParts, "SET") == 0)
        {
            for(int r = 0; r < 128; r++)
            {
                settings[r] = '\0';
            }
            sprintf((char *)settings,"POSITION#MIN#%d~POSITION#MAX#%d~POSITION#SET#%d~FORCE#MIN#%d~FORCE#MAX#%d~FORCE#SET#%d", minPos, maxPos, curPos, minForce, maxForce, curForce);
            settings_size = strlen(settings) + 1;
            flashWrite(OUR_FLASH_ADDR, settings, settings_size);
            printTrace(settings);
            sprintf((char *)msg,"\r\nWrote %d bytes to flash at address 0x%08X.\r\n", (3 + settings_size) & ~3, OUR_FLASH_ADDR);
            printTrace(msg);
        }
    }
    if (strcmp(cmdParts, "READINGS") == 0)
    {
        // READINGS
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "START") == 0)
        {
            //START
            cmdParts = strtok( NULL, cmdDelim );
            if (strcmp(cmdParts, "TRACE") == 0)
            {
                sprintf((char *)msg,"READINGS STARTED\r\n", maxPos);
                printTrace(msg);
                dataReading = 0;
            }
            if (strcmp(cmdParts, "DATA") == 0)
            {
                dataReading = 1;
            }
            reading = 1;
        }
        if (strcmp(cmdParts, "STOP") == 0)
        {
            //STOP
            sprintf((char *)msg,"READINGS STOPPED\r\n", minPos);
            printTrace(msg);
            reading = 0;
        }
    }
    if (strcmp(cmdParts, "VALUES") == 0)
    {
        // GET CURRENT VALUES
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "GET") == 0)
        {
            sprintf((char *)msg,"POSITION#MIN#%d~POSITION#MAX#%d~POSITION#SET#%d~FORCE#MIN#%d~FORCE#MAX#%d~FORCE#SET#%d", minPos, maxPos, curPos, minForce, maxForce, curForce);
            printTrace(msg);
        }
    }
    if (strcmp(cmdParts, "LED") == 0)
    {
        // LEDs control
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "RED") == 0)
        {
            //GREEN LED
            cmdParts = strtok( NULL, cmdDelim );
            if (strcmp(cmdParts, "ON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1);
            }
            if (strcmp(cmdParts, "OFF") == 0)
            {
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
            }
            if (strcmp(cmdParts, "SET") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED1);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
            }
        }
        if (strcmp(cmdParts, "YELLOW") == 0)
        {ne MMC_SUCCESS           0x00
            //YELLOW LED
            cmdParts = strtok( NULL, cmdDelim );
            if (strcmp(cmdParts, "ON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2);
            }
            if (strcmp(cmdParts, "OFF") == 0)
            {
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
            }
            if (strcmp(cmdParts, "SET") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED2);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
            }
        }
        if (strcmp(cmdParts, "GREEN") == 0)
        {
            //RED LED
            cmdParts = strtok( NULL, cmdDelim );
            if (strcmp(cmdParts, "ON") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3);
            }
            if (strcmp(cmdParts, "OFF") == 0)
            {
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
            }
            if (strcmp(cmdParts, "SET") == 0)
            {
                AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED3);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
                AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
            }
        }
        if (strcmp(cmdParts, "ALLOFF") == 0)
        {
            AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED1);
            AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED2);
            AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, LED3);
        }
    }
    */
    if (strcmp(cmdParts, "MMC") == 0)
    {
        cmdParts = strtok( NULL, cmdDelim );
        if (strcmp(cmdParts, "READ") == 0)
        {
            //INITIAL TEST
            printTrace("STORED MMC VALUES:\r\n");
            int blockIndex;
            for (blockIndex = 0; blockIndex < 2048; blockIndex++)
            {
                memset(&mmc_buffer, 0x00, 512);
                mmcReadBlock(blockIndex * 512, 512);
                pCDC.Write(&pCDC, mmc_buffer, 512);
            }
            printTrace("\r\n");
            Delay(2500000);
        }
        if (strcmp(cmdParts, "CLEAR") == 0)
        {
            printTrace("CLEARING SD CARD...\r\n");
            clearMMCCard();
            printTrace("CLEARING DONE\r\n");
        }
        if (strcmp(cmdParts, "START") == 0)
        {
            storingOnMMC = 1;
        }
        if (strcmp(cmdParts, "STOP") == 0)
        {
            storingOnMMC = 0;
        }
    }
}

void putCharToMMCBuffer(char c)
{
    if (mmc_buffer_index < 512)
    {
        mmc_buffer[mmc_buffer_index] = c;
        mmc_buffer_index++;
    }
    else
    {
        mmc_buffer_index = 0;
        mmcWriteBlock(mmc_block_index);
        mmc_block_index += 512;

        memset(&mmc_buffer, 0x00, 512);
        mmcReadBlock(mmc_block_index - 512, 512);
        //pCDC.Write(&pCDC, mmc_buffer, 512);
    }
}



//*--------------------------------------------------------------------------------------
//* Function Name       : Main
//* Object              : Software entry point
//* Input Parameters    : none.
//* Output Parameters   : none.
//*--------------------------------------------------------------------------------------
int main(void)
{
    char data[MSG_SIZE];
    unsigned int length;
    int stepCnt = 0;
    unsigned char str[10];

    /**** System init ****/
    //InitFrec();
    Init_CP_WP();
    //chek for CP and WP
    //CP - card present
    while(((AT91C_BASE_PIOA->PIO_PDSR) & BIT15)) { /*put your card present event here*/  }
    //WP - write protect
    //while(((AT91C_BASE_PIOA->PIO_PDSR) & BIT16)) { /*put your write protect event here*/ }

    if (initMMC() == MMC_SUCCESS)
    {
        //card_state |= 1;
        memset(&mmc_buffer,0,512);
        mmcReadRegister (10, 16);
        mmc_buffer[7]=0;
    }


    flashInit();

    Init_PWM();

    // Enable User Reset and set its minimal assertion to 960 us
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);
    // Led init
    // First, enable the clock of the PIOB
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA ) ;
    //* to be outputs. No need to set these pins to be driven by the PIO because it is GPIO pins only.
    AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, OUTPUT_MASK );
    //* Clear the LED's.
    /*
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, OUTPUT_MASK );
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, OUTPUT_MASK );
    */


    // Init USB device
    AT91F_USB_Open();
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, OUTPUT_MASK );
    // Init USB device
    // Wait for the end of enumeration
    setForce(40000);
    int pCDCEnablingCounter = 0;
    while (!pCDC.IsConfigured(&pCDC) && pCDCEnablingCounter < 2500000){ pCDCEnablingCounter++; };

    if (pCDCEnablingCounter < 2500000)
    {
        CDC = 1;
    }

    setForce(0);

    // Set Usart in interrupt
    //Usart_init();

    //Read and set settings
    memcpy(settings, OUR_FLASH_ADDR, 128);
    int i;memset(&mmc_buffer, 0x00, 512);
    int j;
    char *settingsBlocks[50];
    char settingsDelim[] = "~";
    char *settingsParts = strtok( settings, settingsDelim );
    i = 0;
    while( settingsParts != NULL )
    {
      settingsBlocks[i++] = settingsParts;
      settingsParts = strtok( NULL, settingsDelim );
    }
    for (j = 0; j < i; j++)
    {
       parseSettings(settingsBlocks[j]);
    }

    InitADC();

    Init_PWM();

    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, SW1_MASK);
    AT91F_PIO_CfgInput(AT91C_BASE_PIOA, SW2_MASK);

    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED_GREEN);
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, LED_YELLOW);
    setForce(0);

    //startBlinking(250000);

    /**** MMC CARD ****/

    init_extint();


    while (1)
    {
        cnt++;
        if (cnt > 50000)
        {
            cnt = 0;
            printTrace("COUNTER RESET\n");
        }
    }
}
