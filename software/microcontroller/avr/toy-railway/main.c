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

char buf[10];

// пороговое значение для сравнения длинн импульсов и пауз
static const unsigned int IrPulseThershold = 9;// 1024/8000 * 9 = 1.152 msec
// определяет таймаут приема посылки
// и ограничивает максимальную длину импульса и паузы
static const uint8_t TimerReloadValue = 150;
static const uint8_t TimerClock = (1 << CS02) | (1 << CS00);// 8 MHz / 1024
uint32_t prev_code = 0x0;

unsigned int selected = 0;
unsigned int forward_dir = 0;
unsigned int backward_dir = 0;


volatile struct ir_t
{
    // флаг начала приема полылки
    uint8_t rx_started;
    // принятый код
    uint32_t code,
    // буфер приёма
    rx_buffer;
} ir;

static void ir_start_timer()
{
    TIMSK = (1 << TOIE0);
    TCNT0 = 0;
    TCCR0 = TimerClock;
}

// когда таймер переполнится, считаем, что посылка принята
// копируем принятый код из буфера
// сбрасываем флаги и останавливаем таймер
ISR(TIMER0_OVF_vect)
{
    ir.code = ir.rx_buffer;
    ir.rx_buffer = 0;
    ir.rx_started = 0;
    if(ir.code == 0)
        TCCR0 = 0;
    TCNT0 = TimerReloadValue;
}

// внешнее прерывание по фронту и спаду
ISR(INT0_vect)
{
    uint8_t delta;
    if(ir.rx_started)
    {
        // если длительность импульса/паузы больше пороговой
        // сдвигаем в буфер единицу иначе ноль.
        delta = TCNT0 - TimerReloadValue;
        ir.rx_buffer <<= 1;
        if(delta > IrPulseThershold) ir.rx_buffer |= 1;
    }
    else
    {
        ir.rx_started = 1;
        ir_start_timer();
    }
    TCNT0 = TimerReloadValue;
}

static inline void ir_init()
{
    GIMSK |= _BV(INT0);
    MCUCR |= (1 << ISC00) | (0 <<ISC01);
    ir_start_timer();
}

static inline void pwm_init()
{
    TCCR1A = 0;     // disable all PWM on Timer1 whilst we set it up

    //TOP = (Clock_Speed / (Prescaler * Output_PWM_Frequency)) - 1
    //5kHz
    //TOP = (8000000 / (1 * 5000)) - 1
    ICR1 = 500;   // frequency

    // Configure timer 1 for Fast PWM mode via ICR1, with no prescaling
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1<<WGM12) | (1 << CS10);

    DDRB |= _BV(DDB1);

    TCCR1A |= 2 <<  6;  // enable PWM on port B1 in non-inverted compare mode 2

    OCR1A = 200; // 50% duty on PB1
}


void forward(void)
{
    if (backward_dir)
    {
        if (OCR1A > 200)
        {
            OCR1A -= 100;
        }
        else
        {
            backward_dir = 0;
            forward_dir = 1;
        }
    }
    else
    {
        forward_dir = 1;
        if (OCR1A < 1500)
        {
            OCR1A += 100;
            PORTC |= _BV(PC0);
            PORTC &= ~_BV(PC1);
        }
    }
}

void backward(void)
{
    if (forward_dir)
    {
        if (OCR1A > 200)
        {
            OCR1A -= 100;
        }
        else
        {
            forward_dir = 0;
            backward_dir = 1;
        }
    }
    else
    {
        backward_dir = 1;
        if (OCR1A < 1500)
        {
            OCR1A += 100;
            PORTC &= ~_BV(PC0);
            PORTC |= _BV(PC1);
        }
    }
}

void stop(void)
{
    forward_dir = 0;
    backward_dir = 0;
    PORTC &= ~_BV(PC0);
    PORTC &= ~_BV(PC1);
    OCR1A = 200;
}

int main( void )
{
    DDRB |= (_BV(DDB0) | _BV(DDB1));
    DDRC |= (_BV(DDC0) | _BV(DDC1));

    DDRD &= ~(1 << PD2);

    ir_init();
    sei();
    uart_init();
    pwm_init();

    OCR1A = 200;

    while (1)
    {
        // если ir.code не ноль, значит мы приняли новую комманду
        // ir.code будет сохранять свое значение до следующего переполнения таймера
        if(ir.code)
        {
            // конвертируем код в строку и выводим на дисплей
            if (ir.code != prev_code)
            {
                //ultoa(ir.code, buf, 16);
                //sprintf(buf, "%s\n", buf);
                //uart_puts(buf);
                switch (ir.code)
                {
                    case 0xa2a0080a:
                        //#1 selected
                        sprintf(buf, "#1\n");
                        uart_puts(buf);
                        PORTB |= _BV(PB0);
                        selected = 1;
                    break;
                    case 0xaa0a00a:
                        //#2 selected
                        sprintf(buf, "#2\n");
                        uart_puts(buf);
                        PORTB &= ~_BV(PB0);
                        selected = 2;
                    break;
                    case 0xa80802a2:
                        sprintf(buf, "↑\n");
                        uart_puts(buf);
                        if (selected == 1)
                        {
                            forward();
                        }
                        break;
                    case 0x208a8a2:
                        sprintf(buf, "↓\n");
                        uart_puts(buf);
                        if (selected == 1)
                        {
                            backward();
                        }
                        break;
                    case 0x288a822:
                        sprintf(buf, "S\n");
                        uart_puts(buf);
                        if (selected == 1)
                        {
                            stop();
                        }
                        break;
                }
            }
            prev_code = ir.code;
        }
    }
}
