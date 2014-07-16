#define F_CPU 7372800UL

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

char msg[32];

void uart_init( void )
{
  //настройка скорости обмена
  UBRRH = 0;
  UBRRL = 5;
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

int main (void)
{
  DDRD |= _BV(DDD3); 

  uart_init();

  int c = 0;
  while(1) 
  {
    PORTD ^= _BV(PD3);
    c++;
    sprintf(msg, "%d", c);
    uart_puts("Hello uart\r\n" );
    _delay_ms(500);
  }

  return 0;
}
