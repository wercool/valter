#define F_CPU 7372800UL

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

char msg[32];

void uart_init( void )
{
  //настройка скорости обмена
  UBRRH = 0;
  UBRRL = 3;
  //8 бит данных, 1 стоп бит, без контроля четности
  UCSRC = ( 1 << URSEL ) | ( 1 << UCSZ1 ) | ( 1 << UCSZ0 );
  //разрешить прием и передачу данных
  UCSRB = ( 1 << TXEN ) | ( 1 <<RXEN );
}

unsigned char uart_getc( void )
{
   //ждем приема байта
   while( ( UCSRA & ( 1 << RXC ) ) == 0  );
   //считываем принятый байт
   return UDR;
}
 
void uart_putc( char c )
{
  //ждем окончания передачи предыдущего байта
  while( ( UCSRA & ( 1 << UDRE ) ) == 0 );
  UDR = c;
}

void uart_puts( char *str )
{
  unsigned char c;
  while( ( c = *str++ ) != 0 ) {
    uart_putc( c );
  }
}

int read_adc(uint8_t channel)
{ 
    int ADCval;

    ADMUX = channel;          // use #1 ADC
    ADMUX |= (1 << REFS0);    // use AVcc as the reference
    ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution
    
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
    ADCSRA |= (1 << ADEN);    // Enable the ADC

    ADCSRA |= (1 << ADSC);    // Start the ADC conversion

    while(ADCSRA & (1 << ADSC));      // Thanks T, this line waits for the ADC to finish 


    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;    // ADCH is read so ADC can be updated again

    return ADCval;
} 

int main (void)
{
  DDRD |= _BV(DDD3); 

  DDRC = 0x00; 

  int adc_x;
  int adc_y;
  int adc_z;
  int adc_xyz;
  int i;

  uart_init();

  while(1) 
  {
    adc_x = read_adc(0);
    adc_y = read_adc(1);
    adc_z = read_adc(2);

    for (i = 0; i < 50; i++)
    {
        adc_x += read_adc(0);
        adc_y += read_adc(1);
        adc_z += read_adc(2);
    }

    adc_x = (int)(adc_x / 50);
    adc_y = (int)(adc_y / 50);
    adc_z = (int)(adc_z / 50);

    adc_xyz = (int)((adc_x + adc_y + adc_z) / 3);

    sprintf(msg, "%d, %d, %d, %d\n", adc_x, adc_y, adc_z, adc_xyz);
    uart_puts(msg);

    if (abs(adc_xyz - 500) < 50)
    {
      PORTD |= _BV(PD3);
    }
    else
    {
      PORTD &= ~_BV(PD3);
    }

    _delay_ms(10);
  }

  return 0;
}
