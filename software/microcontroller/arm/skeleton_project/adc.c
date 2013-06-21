#include "adc.h"

AT91PS_ADC a_pADC = AT91C_BASE_ADC;
AT91PS_PMC a_pPMC = AT91C_BASE_PMC;

#define ADC_CHN_0  0x01
#define ADC_CHN_1  0x02
#define ADC_CHN_2  0x04
#define ADC_CHN_3  0x08
#define ADC_CHN_4  0x10
#define ADC_CHN_5  0x20
#define ADC_CHN_6  0x40
#define ADC_CHN_7  0x80

void InitADC(void)
{
    // enable clock frequency for ADC
    a_pPMC->PMC_PCER = 1 << AT91C_ID_ADC;

    // reset ADC
    a_pADC->ADC_CR = 0x1;
    a_pADC->ADC_CR = 0x0;

    // max for startup time и hold time
    a_pADC->ADC_MR = 0x0F1F0F00;

    // enable all needed channels
    a_pADC->ADC_CHER = AT91C_ADC_CH0;
}

unsigned int getValueChannel0()
{
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR & ADC_CHN_0)); //wait until conversion complete
  return a_pADC->ADC_CDR0;
}

unsigned int getValueChannel5()
{
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR & ADC_CHN_5)); //wait until conversion complete
  return a_pADC->ADC_CDR5;
}

unsigned int getValueChannel6()
{
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR & ADC_CHN_6)); //wait until conversion complete
  return a_pADC->ADC_CDR6;
}
