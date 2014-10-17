/*
 * GccApplication1.c
 *
 * Created: 7/3/2014 3:43:10 PM
 *  Author: byron.jacquot
 *
 *
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

typedef struct phasetrack
{
	
	int32_t phasor; // counts 0 to 0xffff - needs to be singed to catch overflow
	bool    rising;
	
	int32_t us_val; // how many microseconds to add to the PWM ?
	
}phasetrack;

// declare data
status current_status;

phasetrack pt;

// define some constants for calculations
//
// # of ticks on longest traverse (end stop for T)
// 0x00ff = 255, 255 * 20 ms = 5.1 Sec
// This might be more useful is variable.
static const int32_t MAX_TICKS = 0x00ff;
static const int32_t MAX_ADC = 0xffc0;

#if 0

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

#endif

// Table of timing constants.
// These are looked up and added to the phasor on each pulse ISR.
// They are indexed & interpolated using the T pot.
//
// Each value was calc'd using the spreadsheet...
//
// the formula per entry is
// increment entry = max phasor/pwm freq in Hz/desired time
// ie: increment = 0xffff/50/travel time
//
// Table is 17 entries long so we can do linear interpolation between 
// the N and N+1th entries
static int16_t timelut[17] =
{
	131, 164, 218,  262,  328,  437,  524,  655,
	749, 874, 1049, 1311, 1748, 2621, 5243, 13107,
	26214
};

// this tells us how far to step in each increment.
// It's based on the reading of the T pot.
int16_t calcDelta()
{
	// time reading is 12 bits, formatted into the top of 
	// a 16-bit value.
	// We'll use the 4 MSBs to look up a time value,
	// and the next 4 to do linear interpolation between it and the next value.

	uint8_t idx = current_status.t >> 12;
	int16_t val = timelut[idx];
	
	// Calc window to next value
	volatile int16_t window = timelut[idx+1] - timelut[idx];

	// split that window into 16 chunks
	window >>= 4;  
	
	// and take the number of chunks as determined by the next 4 bits
	window *= ((current_status.t & 0x00000f00) >> 8);

	return val + window;

}




// This calculates the value of the phasor.
// The phasor is basically constant:
//	- it always goes from 0 to 0xffff
//  - it always increases on the A->B traverse
//  - it always decreases on the B->A traverse
//  - it sits at the endpoints (0 or 0xffff) when in the static positions
//
// The trick is that it will be scaled based on the pot settings before being applied.
//
// Returns true when it reaches the end of the segment, to advance FSM
bool calcNextPhasor(int16_t increment)
{
	if(pt.rising)
	{
		pt.phasor += increment;
	}
	else
	{
		pt.phasor -= increment;
	}
	
	//check for overflow indicating end of segment reached.
	// truncate & return...
	if(pt.phasor > 0xffff)
	{
		pt.phasor = 0xffff;
		return true;		
	}
	else if(pt.phasor < 0)
	{
		pt.phasor = 0;
		return true;		
	}
	
	return false;
}




// This routine scales the phasor into the proper range.
// The phasor always goes from A to B by covering the 0 to 0xffff range
// But A and B may by nearer to each other than that
// and B might even be less than A, reversing things overall.
// This routine centralizes that translation.
int16_t scalePhasor()
{
	int32_t range;
	int32_t offset;
	
	int32_t   result;

	// a, b are 16-bit unsigned values	
	// range and offset are 32-bit signed.
	// When a > b, range is negative
	// thus the result is subtracted from the offset.
	range = current_status.b - current_status.a;
	offset = current_status.a;
	
	//Scale range and offset into 0 to 1000 uSec
	range = (range * 1000)/0x0000ffff;
	offset = (offset * 1000)/0x0000ffff;
		
	// scale phasor into range
	result = (pt.phasor * range)/0x0000ffff;
		
	//add on offset
	result += offset;

	return result;
}



void evalState2()
{
	int16_t delta;
	
	delta = calcDelta();
	
	switch(current_status.st)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(current_status.sw == true)
			{
				current_status.st = eATOB;
				pt.rising         = true;
			}
		}
		break;
		case eATOB:
		{
			if(current_status.sw == false)
			{
				current_status.st = eBTOA;
				pt.rising         = false;				
			}
			
			// climbing up to b
			if( calcNextPhasor(delta) )
			{
				current_status.st = eATTOP;
			}
		}
		break;
		case eATTOP:
		{
			if(current_status.sw == false)
			{
				current_status.st = eBTOA;
				pt.rising         = false;
			}
		}
		break;
		case eBTOA:
		{
			if(current_status.sw == true)
			{
				current_status.st = eATOB;
				pt.rising         = true;
			}
			
			// dropping down to A
			if( calcNextPhasor(delta) )
			{
				current_status.st = eIDLE;
			}
		}
		break;
		default:
		{
			// TODO: better fix?
			// debugger catch for invalid states
			while(1);
		}
	}
	
	pt.us_val =  scalePhasor();
	
}

ISR(TIM1_CAPT_vect)
{
	// no hardware soure to reset - flag is cleared on execution of this vector
	
	// set the pulse width based on switch and pot positions.
	evalState2();
	
	OCR1A = 1000 + pt.us_val;
	
}



void setupPWM(void)
{
	// holdoff timer prescalar counting while we configure
	// This disables the peripheral
	GTCCR = 1 << TSM;

	// Enable timer output compare A bit as output
	DDRA |= 0x40;

	// Note: WGM is 4 bits split between Ctrl registers A and B.
	// We want mode FAST PWM, top set by ICR1
	// Selected by 0b1110.

	// set control reg A
	// Bits 7,6 - Output compare mode chan A = 0b10 = clear output on match, set output at bottom
	// Bits 5,4 - Output compare mode chan B = 0b00 = disconnected
	// Bits 3,2 - RFU
	// Bits 1,0 - Waveform gen mode LSBs 11,10 : 0b10 
	// WGM = fast pwm, top is ICR1
	TCCR1A = 0x82;
	
	// set control reg B
	// Bit 7 - input noice calcel = 0b0 = off
	// bit 6 - input capture edge sel = off
	// bit 5 - RFU
	// bits 4:3 - WGM MSBs = 0b11
	// bits 2:0 - clk scale: 0b010 = IO clk div by 8 = 1 mHz.
	TCCR1B = 0x1a; 
	
	// set control reg C
	// Force output compare - inactive in PWM modes.
	TCCR1C = 0;

	// Start with no interrupts
	TIMSK1 = 0;

	// ICR1 is 16-bit reg.  In Fast PWM, it sets the TOP value for the counter.
	// 50 Hz = 20000 1 mHz pulses.
	ICR1 = 20000-1;
	
	// initialize the counter at 0.
	TCNT1 = 0x0;
	
	// Initialize the output compare for 1000 clock pulses (1 millisecond)
	OCR1A = 1000;

	// enable interrupt on capture (period)
	TIMSK1 = 0x20;

	// reset & restart prescalar counter
	// Writing 0 to TSM (bit 7) causes counter to start counting,
	// Writing 1 to PSR10 (bit 0) causes prescalar to reset, starting prescale from 0
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
	// Using 16 mHz resonator, we'll divide by 2 for 8 MHz.
	CLKPR = 0x80;
	CLKPR = 0x01; // clk div 2 = 8 MHz.
	
	setupPWM();
	
	// set up button pin
	DDRA &= ~0x02; // PA1 as input
	PORTA |= 0x02; // pulled up
	
	// configure global data
	current_status.sw = false;
	current_status.st = eIDLE;
		
	current_status.st = eIDLE;
		
	pt.phasor = 0;
	pt.rising = true;
		
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