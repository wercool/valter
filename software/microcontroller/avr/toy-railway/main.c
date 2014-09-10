#define F_CPU 8000000UL

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/iom8.h>

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


char msg[32];


ISR(TIMER0_OVF_vect)
{
}


int main( void )
{
  DDRD |= _BV(DDD3);
  DDRC = 0x00;
/*
    8000000 / 1024 = 7812    Hz;   1 / 7812  * 256 =  0.03277 s
    8000000 / 256  = 31250   Hz;
    8000000 / 64   = 125000  Hz;
    8000000 / 8    = 1000000 Hz;
*/
  TCCR0 = 0x03;
  TIMSK = 0x01;
  sei();

  uart_init();

  while (1)
  {
      sprintf(msg, "%d\n", command);
      //uart_puts(msg);
  }
}
