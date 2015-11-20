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

char uart_buf[32];

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

unsigned char columns[24];
unsigned char rows[8];
/*
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
 */

unsigned char frames[32][24] = {
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

};



int main( void )
{
    signed char r;
    signed char c;
    signed char rCnt = 0;
    signed char frameCnt = 0;

    //DDRB |= (_BV(DDB0) | _BV(DDB1));
    DDRC |= (_BV(DDC0) | _BV(DDC1) | _BV(DDC2) | _BV(DDC3) | _BV(DDC4));

    sei();
    uart_init();

    sprintf(uart_buf, "START\n");
    uart_puts(uart_buf);

    PORTC |= _BV(PC3);
    PORTC &= ~ _BV(PC4);
    PORTC |= _BV(PC2);
    PORTC |= _BV(PC1);

    while (1)
    {
        if (rCnt == -1)
        {
            rCnt = 7;
        }
        for (r = 0; r < 8; r++)
        {
            rows[r] = 0;
        }
        rows[rCnt] = 1;
        rCnt--;


        for (c = 0; c < 23; c++)
        {
            columns[c] = frames[frameCnt][c];
        }
        frameCnt++;
        if (frameCnt == 8)
        {
            frameCnt = 0;
        }

        PORTC &= ~ _BV(PC2);

        for (c = 23; c > 0; c--)
        {
            if (columns[c] == 1)
            {
                PORTC &= ~ _BV(PC0);
            }
            else
            {
                PORTC |= _BV(PC0);
            }
            PORTC &= ~ _BV(PC1);
            PORTC |= _BV(PC1);
        }
        for (r = 0; r < 8; r++)
        {
            if (rows[r] == 1)
            {
                PORTC |= _BV(PC0);
            }
            else
            {
                PORTC &= ~ _BV(PC0);
            }
            PORTC &= ~ _BV(PC1);
            PORTC |= _BV(PC1);
        }

        PORTC |= _BV(PC2);

        //temporary
        _delay_ms(100);
    }
}
