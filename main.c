#include<stdint.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#define SET_BIT(PORT,BIT) PORT|= (1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&= ~(1<<BIT)

struct {
  volatile unsigned int FLAG_ISR1:1;
}FLAG_BIT;

int main()
{
    SET_BIT(DDRD,PD6);
    CLR_BIT(DDRD,PD3);
    SET_BIT(PORTD,PD3);
    EICRA&=~(1<<ISC11);
    EICRA|=(1<<ISC10);
    EIMSK|=(1<<INT1);
    sei();

    TCCR0A|= ((1<<WGM01) | (1<<WGM00));
    TCCR0A |= (1<< COM0A1);
    TCCR0A |= (1<<COM0A0);

    TCNT0=0x00;
    OCR0A=128;
    TCCR0B|=((1<<CS00)|(1<<CS02));
    TCCR0B&=~(1<<CS01);


    uint16_t DISTANCE;
    ADMUX&=0x00;
    ADMUX|= (1<<REFS0);
    ADCSRA|= (1<< ADEN);
    SET_BIT(DDRB,PB1);

    while(1)
    {
        if(FLAG_BIT.FLAG_ISR1==1)
            OCR0A+=13;
        {
            FLAG_BIT.FLAG_ISR1=0;
        }
        ADCSRA|=(1<<ADSC);
        while(ADCSRA & (1<<ADSC));
        DISTANCE=ADC;
        if(DISTANCE<409){
            if(OCR0A>128 && OCR0A<142)
            {
                SET_BIT(PORTB,PB1);
                _delay_ms(1000);

            }
            else
            {
                CLR_BIT(PORTB,PB1);
            }
        }
    }
    return 0;
    }

ISR(INT1_vect)
            {
            cli();
            FLAG_BIT.FLAG_ISR1=1;
            sei();
            }



