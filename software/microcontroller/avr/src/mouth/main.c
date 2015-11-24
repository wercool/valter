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

//0
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

//1
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

//2
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

//3
        {0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},

//4
        {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
        {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
        {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
 */

unsigned char frame[8][24];

unsigned char frame_1_r_2_5[24]   = {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0};
unsigned char frame_1_r_3_4[24]   = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1};

unsigned char frame_2_r_1_6[24]   = {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
unsigned char frame_2_r_2_5[24]   = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0};
unsigned char frame_2_r_3_4[24]   = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1};

unsigned char frame_3_r_0_7[24]   = {0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0};
unsigned char frame_3_r_1_6[24]   = {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0};
unsigned char frame_3_r_2_5[24]   = {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0};
unsigned char frame_3_r_3_4[24]   = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1};

unsigned char frame_4_r_0_7[24]   = {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0};
unsigned char frame_4_r_1_6[24]   = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0};
unsigned char frame_4_r_2_5[24]   = {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0};
unsigned char frame_4_r_3_4[24]   = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1};


int main( void )
{
    unsigned char row;
    signed char r;
    signed char c;
    unsigned int adcSUM = 0;
    unsigned int adcCNT = 0;
    signed char frameNum = 0;
    unsigned char animDirection = 1;

    unsigned int signal;

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

    for (c = 0; c < 24; c++)
    {
        frame[0][c] = 0;
        frame[1][c] = 0;
        frame[2][c] = 0;
        frame[3][c] = 1;
        frame[4][c] = 1;
        frame[5][c] = 0;
        frame[6][c] = 0;
        frame[7][c] = 0;
    }

    while (1)
    {
        signal = read_adc(6);
        if (signal > 23)
        {
            adcSUM += signal;
        }
        adcCNT++;

        if (adcCNT > 10)
        {

            signal = round((double)adcSUM / (double)adcCNT);
            adcCNT = 0;
            adcSUM = 0;

            //sprintf(uart_buf, "SIGNAL:%u\r\n", signal);
            //uart_puts(uart_buf);

            if (signal > 23)
            {
                animDirection = rand() % 2;

                for (c = 0; c < 24; c++)
                {
                    switch (frameNum)
                    {
                        case 0:
                            frame[0][c] = 0;
                            frame[1][c] = 0;
                            frame[2][c] = 0;
                            frame[3][c] = 1;
                            frame[4][c] = 1;
                            frame[5][c] = 0;
                            frame[6][c] = 0;
                            frame[7][c] = 0;
                            break;
                        case 1:
                            frame[0][c] = 0;
                            frame[1][c] = 0;
                            frame[2][c] = frame_1_r_2_5[c];
                            frame[3][c] = frame_1_r_3_4[c];
                            frame[4][c] = frame_1_r_3_4[c];
                            frame[5][c] = frame_1_r_2_5[c];
                            frame[6][c] = 0;
                            frame[7][c] = 0;
                            break;
                        case 2:
                            frame[0][c] = 0;
                            frame[1][c] = frame_2_r_1_6[c];
                            frame[2][c] = frame_2_r_2_5[c];
                            frame[3][c] = frame_2_r_3_4[c];
                            frame[4][c] = frame_2_r_3_4[c];
                            frame[5][c] = frame_2_r_2_5[c];
                            frame[6][c] = frame_2_r_1_6[c];
                            frame[7][c] = 0;
                            break;
                        case 3:
                            frame[0][c] = frame_3_r_0_7[c];
                            frame[1][c] = frame_3_r_1_6[c];
                            frame[2][c] = frame_3_r_2_5[c];
                            frame[3][c] = frame_3_r_3_4[c];
                            frame[4][c] = frame_3_r_3_4[c];
                            frame[5][c] = frame_3_r_2_5[c];
                            frame[6][c] = frame_3_r_1_6[c];
                            frame[7][c] = frame_3_r_0_7[c];
                            break;
                        case 4:
                            frame[0][c] = frame_4_r_0_7[c];
                            frame[1][c] = frame_4_r_1_6[c];
                            frame[2][c] = frame_4_r_2_5[c];
                            frame[3][c] = frame_4_r_3_4[c];
                            frame[4][c] = frame_4_r_3_4[c];
                            frame[5][c] = frame_4_r_2_5[c];
                            frame[6][c] = frame_4_r_1_6[c];
                            frame[7][c] = frame_4_r_0_7[c];
                            break;
                    }
                }
                if (animDirection == 1)
                {
                    frameNum++;
                    if (frameNum == 5)
                    {
                        frameNum = 4;
                        animDirection = 0;
                    }
                }
                else
                {
                    frameNum--;
                    if (frameNum == -1)
                    {
                        frameNum = 0;
                        animDirection = 1;
                    }
                }
            }
            else
            {
                for (c = 0; c < 24; c++)
                {
                    frame[0][c] = 0;
                    frame[1][c] = 0;
                    frame[2][c] = 0;
                    frame[3][c] = 1;
                    frame[4][c] = 1;
                    frame[5][c] = 0;
                    frame[6][c] = 0;
                    frame[7][c] = 0;
                }
            }
        }


        //OUTPUT FRAME ONTO THE LED MATRIX
        for (row = 0; row < 8; row++)
        {
            for (r = 0; r < 8; r++)
            {
                rows[r] = 0;
            }
            rows[row] = 1;

            for (c = 0; c < 24; c++)
            {
                columns[c] = frame[row][c];
            }

            PORTC &= ~ _BV(PC2);

            for (c = 23; c > -1; c--)
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

            for (r = 7; r > -1; r--)
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
        }
        _delay_ms(1);
    }
}
