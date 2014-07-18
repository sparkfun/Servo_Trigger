/*
 * GccApplication1.c
 *
 * Created: 7/3/2014 3:43:10 PM
 *  Author: byron.jacquot
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

//volatile uint16_t vals[3];

typedef enum state
{
	eIDLE = 0,
	eATOB,
	eATTOP,
	eBTOA	
}state;

typedef struct status
{
	int16_t a;
	int16_t b;
	int16_t t;
	
	int16_t last;
	
	bool sw;
	
	state st;
	
} status;

status current_status;

int16_t calcIncrement()
{
	int16_t val;
	
	val = current_status.t >> 2;		
	
	if(val == 0)
	{
		val = 1;
	}
	if(current_status.a > current_status.b)
	{
		val = -val;
	}
	return val;
}

int16_t evalState()
{
	int16_t delta;
	
	delta = calcIncrement();
	
	switch(current_status.st)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(current_status.sw == true)
			{
				current_status.st = eATOB;
				
				return current_status.last;
			}
			
			// stay at current A position
			return current_status.a;
		}
		break;
		case eATOB:
		{
			if(current_status.sw == false)
			{
				current_status.st = eBTOA;
				
				return current_status.last;
			}
			
			if(current_status.a < current_status.b)
			{
				// climbing up to b
				if((current_status.last + delta) >= current_status.b)
				{
					current_status.st = eATTOP;
				
					return current_status.b;
				}
			}
			else
			{
				// dropping down to b
				if((current_status.last + delta) <= current_status.b)
				{
					current_status.st = eATTOP;
					
					return current_status.b;
				}
			}
			
			return current_status.last + delta;
		}
		break;
		case eATTOP:
		{
			if(current_status.sw == false)
			{
				current_status.st = eBTOA;
				
				return current_status.last;
			}
			
			return current_status.b;
		}
		break;
		case eBTOA:
		{
			if(current_status.sw == true)
			{
				current_status.st = eATOB;
				
				return current_status.last;
			}
			
			if(current_status.a < current_status.b)
			{
				// dropping down to A
				if((current_status.last - delta) <= current_status.a)
				{
					current_status.st = eIDLE;
					
					return current_status.a;
				}
			}
			else
			{
				// climbing up to A
				if((current_status.last - delta) >= current_status.a)
				{
					current_status.st = eIDLE;
					
					return current_status.a;
				}
			}
			
			return current_status.last - delta;
					
		}
		break;
	}
	
	// TODO: better fix?
	// catch for invalid states
	while(1);
}

ISR(TIM1_CAPT_vect)
{
	//uint16_t new;
	
	// flag is cleared on execution of this vector
	
	// set the pulse width based on switch and pot positions.
	current_status.last = evalState();
	
	OCR1A = 1000 + current_status.last;
	
}



void setPWM(void)
{
	// holdoff timer prescalar counting
	GTCCR = 1 << TSM;

	// Enable timer output compare A bit as output
	DDRA |= 0x40;

	// set control reg A
	//TCCR1A = 0x40;// toggle OC1a on match
	TCCR1A = 0x82;// clear OC1a on match, WGM = fast pwm, top is ICR1
	
	// set control reg B
	//TCCR1B = 0x02;
	//TCCR1B = 0x05; // div by 1024
	//TCCR1B = 0x0d; // div by 1024, clear at OCR1A
	TCCR1B = 0x1a; // // div by 8, fast PWM, top is ICR1
	
	
	// set control reg C
	TCCR1C = 0;

	// no interrupts
	TIMSK1 = 0;

	//TCNT1 = 0x5555;
	ICR1 = 20000-1;
	TCNT1 = 0x0;
	OCR1A = 1000;

	// enable interrupt on capture (period)
	TIMSK1 = 0x20;

	// reset & restart prescalar counter
	GTCCR = 1 << PSR10;
}

int readADC(uint8_t chan)
{
	int value;
	
	// only allow the pins we've selected to read...
	if(!( (chan == 0) || (chan == 3) || (chan == 7)))
	{
		return 0;
	}
	
	// turn on adc
	// TODO: perhaps turn on and leave on?
	ADCSRA = 0x86; // power bit, prescale of 64
	
	// set digital input disable
	DIDR0 = 0x89;
	
	// input mux, vref selection
	ADMUX = chan;

	// pause a moment...
	for(volatile uint8_t i = 0xff; i != 0; i--);
	
	// bit 6 starts conversion
	// write bit 4 to clear it
	ADCSRA |= 0x50;
	//ADCSRA = 0x40;
	
	// start bit clears when complete
	while(ADCSRA & 0x40);

	value = ADCW;
	
	return value;
}

int main(void)
{
	//DDRA |= 0x02;
	//PORTA |= 0x00;
	
	// Configure clock scaling.
	// 2-step procedure to keep stray pointers from mangling the value.
	// TODO: evaluate speeds...we're trading speed for power consumption.
	CLKPR = 0x80;
	CLKPR = 0x01; // clk div 2 = 8 MHz.
	
	setPWM();
	
	// set up button pin
	DDRA &= ~0x02; // PA1 as input
	PORTA |= 0x02; // pulled up
	
	// configure internal data
	current_status.sw = false;
	current_status.st = eIDLE;
		
	// then enable interrupts
	sei();
	
    while(1)
    {
		// read pots
		current_status.a = readADC(0); 
		current_status.b = readADC(3);
		current_status.t = readADC(7);
		
		// read switch - active low
		current_status.sw = !(PINA & 0x02);
		
    }
}