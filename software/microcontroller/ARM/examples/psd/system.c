#include "system.h"
#include "AT91SAM7S64.h"

AT91PS_PMC pPMC   = AT91C_BASE_PMC;

void Delay_s (unsigned long a) { while (--a!=0); }

//MAIN POINTER
AT91PS_PIO    p_pPio   = AT91C_BASE_PIOA;
AT91PS_PMC    p_pPMC   = AT91C_BASE_PMC;
AT91PS_USART  p_pUSART = AT91C_BASE_US0;
AT91PS_PDC    p_pPDC   = AT91C_BASE_PDC_US0;
AT91PS_MC     p_pMC    = AT91C_BASE_MC;
AT91PS_AIC    p_pAic   = AT91C_BASE_AIC;

void InitFrec(void)
{
  //Watchdog Disable
  AT91C_BASE_WDTC->WDTC_WDMR= AT91C_WDTC_WDDIS;

  //Enabling the Main Oscillator:
  //SCK = 1/32768 = 30.51 uSecond
  //Start up time = 8 * 6 / SCK = 56 * 30.51 = 1,46484375 ms
  pPMC->PMC_MOR = (( AT91C_CKGR_OSCOUNT & (0x06 <<8) | AT91C_CKGR_MOSCEN ));
  //Wait the startup time
  while(!(pPMC->PMC_SR & AT91C_PMC_MOSCS));


  //Setting PLL and divider:
  //- div by 5 Fin = 3,6864 =(18,432 / 5)
  //- Mul 25+1: Fout =	95,8464 =(3,6864 *26)
  //for 96 MHz the erroe is 0.16%
  //Field out NOT USED = 0
  //PLLCOUNT pll startup time estimate at : 0.844 ms
  //PLLCOUNT 28 = 0.000844 /(1/32768)
  pPMC->PMC_PLLR = ((AT91C_CKGR_DIV & 3) | (AT91C_CKGR_PLLCOUNT & (28<<8)) | (AT91C_CKGR_MUL & (24<<16)));

  // Wait the startup time
  while(!(pPMC->PMC_SR & AT91C_PMC_LOCK));
  while(!(pPMC->PMC_SR & AT91C_PMC_MCKRDY));

  //Selection of Master Clock and Processor Clock
  //select the PLL clock divided by 2
  pPMC->PMC_MCKR = AT91C_PMC_CSS_PLL_CLK | AT91C_PMC_PRES_CLK_2 ;
  while(!(pPMC->PMC_SR & AT91C_PMC_MCKRDY));

}


void InitPeriphery(void) {


}

