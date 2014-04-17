#include "delay.h"

void delay_us(int delay)
{
  while(delay--)
  {
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
      __asm ("NOP");
  }
}

void delay_ms(int delay)
{
  char i;
  while(delay--)
  {
    for(i = 0; i < 4; i++)
    {
      delay_us(250);
    }
  }
}
