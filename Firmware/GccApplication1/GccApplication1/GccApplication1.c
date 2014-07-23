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
	int32_t a;
	int32_t b;
	int32_t t;
	
	int32_t last;
	
	bool sw;
	
	state st;
	
} status;


// declare data
status current_status;

// define some constants for calculations
//
// # of ticks on longest traverse (end stop for T)
// 0x00ff = 255, 255 * 20 ms = 5.1 Sec
// This might be more useful is variable.
static const int32_t MAX_TICKS = 0x00ff;
//static const int32_t MAX_ADC = 0x03ff;
static const int32_t MAX_ADC = 0xffc0;



// Calculate SIGNED value for current update.
int32_t calcIncrement()
{

	int32_t travel_time;
	int32_t deltaY;
	int32_t increment;
	
	travel_time = (MAX_TICKS * current_status.t )/MAX_ADC;

	// Travel time can't be 0, or division underflows...
	if(travel_time == 0)
	{
		travel_time = 1;
	}
	
	deltaY = current_status.b - current_status.a;
	
	increment = deltaY/travel_time;
	
	//increment /= 64;
	
	return increment;
}

int32_t evalState()
{
	int32_t delta;
	
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
			delta = calcIncrement();

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
			delta = calcIncrement();

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
	
	OCR1A = 1000 + (current_status.last /64);
	
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

uint32_t readADC(uint8_t chan)
{
	uint32_t value;
	
	// only allow the pins we've selected to read...
	if(!( (chan == 0) || (chan == 3) || (chan == 7)))
	{
		return 0;
	}
	
	// turn on adc
	// TODO: perhaps turn on and leave on?
	ADCSRA = 0x86; // power bit, prescale of 64
	
	ADCSRB |= 0x10; // left justify
	
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