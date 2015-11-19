#define F_CPU 8000000UL
#define BAUD 38400UL
#define BAUDRATE ((F_CPU)/(BAUD * 16UL)-1)

#define BIT(x)              (1 << (x))         //Set a particular bit mask
#define CHECKBIT(x,b)       (x & BIT(b))       //Checks bit status

#define SETBIT(x,b)         x |= BIT(b);       //Sets the particular bit
#define CLEARBIT(x,b)       x &= ~BIT(b);      //Sets the particular bit
#define TOGGLEBIT(x,b)      x^=BIT(b);         //Toggles the particular bit

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/iom8.h>

//UART buf

char buf[32];

void uart_init( void )
{
    UBRRH = (BAUDRATE>>8);                      // shift the register right by 8 bits
    UBRRL = BAUDRATE;                           // set baud rate
    UCSRB|= (1<<TXEN)|(1<<RXEN);                // enable receiver and transmitter
    UCSRC|= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format
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
    while( ( c = *str++ ) != 0 )
    {
        uart_putc( c );
    }
}

int main( void )
{
    DDRB |= (_BV(DDB0) | _BV(DDB1));
    DDRC |= (_BV(DDC0) | _BV(DDC1));

    DDRD &= ~(1 << PD2);

    sei();
    uart_init();

    sprintf(buf, "START\n");
    uart_puts(buf);

    while (1)
    {
    }
}
