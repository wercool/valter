#define F_CPU 7372800UL

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/iom8.h>

//#include <compat/twi.h>

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


char msg[32];
static unsigned int readings_num = 256;
unsigned int readings[256];
unsigned int readings_idx = 0;
char accelerating = 0;
char accelerating_cnt = 0;

unsigned int ejectTime = 0;
unsigned int ejectTimeCnt = 0;
unsigned int velocity = 0;

unsigned int adc_x = 0;
unsigned int adc_y = 0;
unsigned int adc_z = 0;
//  unsigned int adc_xyz;
char axay_cnt = 0;

unsigned int acceleration_adc_sum = 0;
unsigned int acceleration_g_sum = 0;
double velocity_m_per_s = 0;
unsigned int velocity_mm_per_s = 0;
unsigned int eject_time_ms = 0;

unsigned int timer_cnt = 0;
ISR(TIMER0_OVF_vect)
{
    timer_cnt++;
    if (timer_cnt > 2)
    {
        if (accelerating == 1)
        {
            velocity_m_per_s += (((double) adc_z / (double) 60) * 9.81) / (double) 1000;
        }
        timer_cnt = 0;
    }
}



int main( void )
{
  DDRD |= _BV(DDD3);
  DDRC = 0x00;
/*
 7372800 / 1024 = 7200 Hz;   1 / 7200   = 0.000138889 s; 0.000138889 * 256 = 0.035555584 s
7372800 / 256  = 28800 Hz;  1 / 28800  = 0.000034722 s; 0.000034722 * 256 = 0.008888832 s;
7372800 / 64   = 115200 Hz; 1 / 115200 = 0.000008681 s; 0.000008681 * 256 = 0.002222336 s;
7372800 / 8    = 921600 Hz; 1 / 921600 = 0.000001085 s; 0.000001085 * 256 = 0.00027776  s;
 */
  TCCR0 = 0x03;
  TIMSK = 0x01;
  sei();

  uart_init();

  for (readings_idx = 0; readings_idx < readings_num; readings_idx++)
  {
    readings[readings_idx] = 0;
  }
  readings_idx = 0;

  while (1)
  {
    adc_x = abs(read_adc(0) - 500);
    adc_y = abs(read_adc(1) - 500);
    if (ejectTime == 0)
    {
        if (adc_x > 250 || adc_y > 250)
        {
            axay_cnt++;
            if (axay_cnt > 5)
            {
                char n = 0;
                for (n = 0; n < 4; n++)
                {
                  PORTD |= _BV(PD3);
                  _delay_ms(20);
                  PORTD &= ~_BV(PD3);
                  _delay_ms(40);
                }
                axay_cnt = 0;
            }
            sprintf(msg, "Vo=%u mm/s, t=%ums\n", velocity_mm_per_s, eject_time_ms);
            uart_puts(msg);
        }
        else
        {
            axay_cnt = 0;
        }
    }

    unsigned int adc_z_sum = 0;
    char j = 0;
    for (j = 0; j < 10; j++)
    {
        adc_z_sum += abs(read_adc(2) - 500);
    }
    adc_z = round((double) adc_z_sum / (double) 10);

    //adc_xyz = (unsigned int)round((double)(adc_x + adc_y + adc_z) / 3);


//  sprintf(msg, "%u\n", (int) ((double) adc_z / (double) 60));
//  sprintf(msg, "%u\n", velocity_mm_per_s);
//  uart_puts(msg);

    if (ejectTime > 0)
    {
        ejectTimeCnt--;
        if (ejectTimeCnt == 0)
        {
            PORTD |= _BV(PD3);
            _delay_ms(100);
            PORTD &= ~_BV(PD3);

            accelerating = 0;
            accelerating_cnt = 0;
            ejectTime = 0;
            ejectTimeCnt = 0;
            velocity = 0;
            for (readings_idx = 0; readings_idx < readings_num; readings_idx++)
            {
              readings[readings_idx] = 0;
            }
            readings_idx = 0;

            eject_time_ms = (int) round((double) velocity_m_per_s / 9.81 * 1000);
            velocity_mm_per_s = (int) round(velocity_m_per_s * 1000);
            velocity_m_per_s = 0;
        }
    }
    else
    {
          if (adc_z > 100)
          {
            accelerating_cnt++;
            if (accelerating_cnt > 5)
            {
                accelerating = 1;
            }
          }
          else
          {
              if (accelerating == 1 && readings_idx > 50)
              {
                  readings_idx = readings_idx - 1;
                  unsigned int i;
                  for (i = 0; i < readings_idx; i++)
                  {
                      velocity += readings[i];
                  }
                  velocity = (int) round((double) velocity / (double) 60);
                  ejectTime = (int) round((double) velocity / (double)(2.5));
                  ejectTimeCnt = ejectTime;
              }

              if (accelerating_cnt <= 0)
              {
                  accelerating_cnt = 0;
                  accelerating = 0;
                  readings_idx = 0;
              }
              else
              {
                  accelerating_cnt--;
              }
          }

          if (accelerating == 1)
          {
            if (readings_idx < readings_num)
            {
              readings[readings_idx] = adc_z;
              readings_idx++;
            }
          }
    }
  }

  while (1)
  {
      char n = 0;
      for (n = 0; n < 4; n++)
      {
        PORTD |= _BV(PD3);
        _delay_ms(100);
        PORTD &= ~_BV(PD3);
        _delay_ms(100);
      }
    sprintf(msg, "Vo=%u,t=%u\n", velocity, ejectTime);
//    uart_puts(msg);
    for (readings_idx = 0; readings_idx < readings_num; readings_idx++)
    {
      if (readings[readings_idx] != 0)
      {
        sprintf(msg, "%u,\n", readings[readings_idx]);
      }
//      uart_puts(msg);
    }
    _delay_ms(4000);
  }
}
