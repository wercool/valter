#define F_CPU 7372800UL

#include <avr/io.h>
#include <util/delay.h>

int
main (void)
{
    DDRD |= _BV(DDD3); 
    
    while(1) 
    {
        PORTD ^= _BV(PD3);
        _delay_ms(500);
    }
}
