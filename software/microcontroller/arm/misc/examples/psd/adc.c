//adc.c
#include "adc.h"

AT91PS_ADC a_pADC = AT91C_BASE_ADC;
AT91PS_PMC a_pPMC = AT91C_BASE_PMC;


void InitADC(void) {

  //a_pPMC->PMC_PCER = 1 << AT91C_ID_ADC;
  a_pADC->ADC_CR = 0x2;   // set Start Bit
  a_pADC->ADC_MR = 0x3f00;   // set mode register, 10bit chanel, MCK/128
  a_pADC->ADC_CHER = (1 << 0); //(1 << 4) | (1 << 5) | (1 << 6); // enable chanel 4 and 5 and 6 and 7
}

unsigned int GetValue_chanel0() {
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR&0x10)); //wait until conversion complete
  return a_pADC->ADC_CDR0;
}

unsigned int GetValue_chanel4() {
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR&0x10)); //wait until conversion complete
  return a_pADC->ADC_CDR4;
}

unsigned int GetValue_chanel5() {
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR&0x10)); //wait until conversion complete
  return a_pADC->ADC_CDR5;
}

unsigned int GetValue_chanel6() {
  a_pADC->ADC_CR = 0x2; // set Start Bit
  while(!(a_pADC->ADC_SR&0x10)); //wait until conversion complete
  return a_pADC->ADC_CDR6;
}

