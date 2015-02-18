/*
 ServoTrigger.c
 
 Created: 7/3/2014 3:43:10 PM
 Author: byron.jacquot
 
 The Servo Trigger is a small board you can use to implement simple control 
 over hobby RC servos.  When an external switch or logic signal changes 
 state, the servo trigger causes the servo motor to move from  position A 
 to position B.  You also specify the time that the servo takes to make 
 the transition. 
 
 The code is designed to function primarily in the interrupt service routine
 for PWM1.  PWM hardware is configured to call the ISR at the start of the cycle
 when the output gets set high.  It then calculates and sets the pulse 
 width, and refreshes the ADC inputs.
 
 The main loop of the code is rather simple.  It sets up in I/O and
 PWM, then allows the interrupt to run, and puts the processor to 
 sleep in between.
 
 BUILDING & CONFIGURATION:
 There are more possible state machines than can fit in memory all at once.
 2 FSMs can be present at a time - selected using the MODE solder jumper on the back
 of the PCB.  They are known as FSMA (jumper cleared - default) and FSMB (jumper soldered).
 
 The FSMs are selected by defining compile-time macros in the build scripts. 
 To change functions, find the -D directives, and set them to the names of the
 desired functions. 
 In AVRStudio, this is in the project tab->toolchain->AVR/GNU C compiler->symbols dialog. 
 In the command-line AVR-GCC, it's in the *.c->*.o rule in the makefile.
 
 You can select the FSMs for the two slots by changing the definition of the "FSMA" and
 "FSMB" macros.  These macros should be set to the name of the FSM function for the desired slot.
 There are several candidate FSM names:
  - bistableFSM
  - oneshotFSM
  - ctpFSM
  - togglingFSM
  - astableFSM
 
 See the product documentation for more details about each FSM.
 
 All of these FSMs are implemented in the source file - rather than building a bunch of 
 convoluted macros to include the right functions, we're taking advantage of the 
 linker-elision feature of GCC - functions that are never called will be omitted from the 
 output - you'll see the names in the "discarded sections" of the *.MAP file.  It's very
 dependent of compiling and linking with the right flags set.
 See this link for more info: https://gcc.gnu.org/ml/gcc-help/2003-08/msg00128.html
 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdbool.h>

/* constant declarations */

// constants for the hardware interfaces
// Pots for positions A, B and time T.
// Numbers are ADC channels, not pins
static const uint8_t POTA_CHAN = 0;
static const uint8_t POTB_CHAN = 3;
static const uint8_t POTT_CHAN = 7;

// Pins for configuration solder jumpers
// "_A_" means in port A, "_B_" means in port B.
static const uint8_t POLSEL_A_MASK = 0x04;
static const uint8_t MODESEL_B_MASK = 0x04;

// Input pin for switch or trigger
// "_A_" means in port A
static const uint8_t TRIG_PIN_A_MASK = 0x02;

// Timing & ramp gen constants
// Pulse width = PWM_MIN_USEC + PWM_RANGE_USEC
// some servos have a wider range than others - 1 to 2 msec is save for all tested so far.
// .7 to 2.3 msec will drive some farther, but will make others very unhappy, possibly grinding gears.

#ifdef SAFERANGE
// Some servos are very unhappy if pulses leave the 1 to 2 mSec range.  
// SAFERANGE constrains pulses to that range.
static const int32_t PWM_MIN_USEC   = 1000; // narrowest pulse is 1000 usec
static const int32_t PWM_RANGE_USEC = 1000; // diff between min and max PWM pulses.
#else
// some servos are less fragile/critical - if you want shorter/wider pulses, 
// you can enable these and adjust the values
static const int32_t PWM_MIN_USEC   = 700; // narrowest pulse is 700 usec
static const int32_t PWM_RANGE_USEC = 1600; // diff between min and max PWM pulses.
#endif


static const int32_t ADC_MAX        = 0x0000ffff; // ADC values are left-justified into 16-bits
static const int32_t PHASOR_MAX     = 0x0000ffff; // Phasor counts from 0 to 0xffff

// declare some constants for calculations
//
// Table of timing constants.
// These are looked up and added to the phasor on each pulse ISR.
// They are indexed & interpolated using the T pot.
//
// Each value was calc'd using the spreadsheet: translation.ods
//
// the formula per entry is
// value = PHASOR_MAX/pwm freq in Hz/desired time
// ie: increment = 0xffff/50/travel time
//
// Table is 17 entries long so we can do linear interpolation between
// the N and N+1th entries, indexed using 4 MSBs of T value, interpolated
// using the next 4 bits.
static const int16_t timelut[17] =
{
    437,  583,   749,   1049,
    1311, 1748,  2185,  2621,
    3277, 4369,  5243,  6554,
    8738, 10923, 13107, 16384,
    26214
};


/* Structure definitions */

// All the FSMs use the same group of states - some may not use all states.
typedef enum state
{
	eIDLE = 0,
	eATOB,
	eATTOP,
	eBTOA,
	eWAIT_TO_RESET	
}state;

// status encapsulates what would otherwise be global variables.
typedef struct status 
{
	int32_t a; // latest ADC value for A pot
	int32_t b; // latest ADC value for B pot
	int32_t t; // latest ADC value for T pot 
	
	bool  input; // is input asserted?  (accounting for input polarity setting)
	
    
	bool  mode;  // stores mode jumper config
	bool  input_polarity; // stores input polarity jumper config
	
	state st;  // currently active state of FSM
	
	int32_t phasor; // counts 0 to 0xffff - needs to be singed to catch overflow
	bool    rising; // are we rising or falling?
	
	int32_t us_val; // how many microseconds to add to the PWM pulse?
	
}status;

/* Structure declarations*/

status     current_status;


/*
	calcDelta()

	Calculate how far to step in each increment, and return the step size.
	It's based on the reading of the T pot.
	MS nybble indexes LUT value.
	Also reads the next adjacent LUT value, then uses next nybble to 
	fo linear interoplation between them.
*/
int16_t calcDelta()
{
	// time reading is 12 bits, formatted into the top of 
	// a 16-bit value.
	// We'll use the 4 MSBs to look up a time value,
	// and the next 4 to do linear interpolation between it and the next value.

	uint8_t idx = current_status.t >> 12;
	int16_t val = timelut[idx];
	
	// Calc window between this and next value
	volatile int16_t window = timelut[idx+1] - timelut[idx];

	// split that window into 16 chunks
	window >>= 4;  // IE: divide by 16
	
	// and take the number of chunks as determined by the next 4 bits
	window *= ((current_status.t & 0x00000f00) >> 8);

	return val + window;

}


/*
	bool calcNextPhasor(int16_t increment)

    This calculates the value of the phasor.
    The phasor is range constrained:
      - it always goes from 0 to 0xffff (PHASOR_MAX)
      - it always increases on the A->B traverse
      - it always decreases on the B->A traverse
      - it sits at the endpoints (0 or 0xffff) when in the static positions

    This leaves us some resolution, so really slow transitions atill slowly crawl 
    along, and really quick ones

    The trick is that it will be scaled based on the pot settings before being applied.

    Returns true when segment overflows, to advance FSM.
*/
bool calcNextPhasor(int16_t increment)
{
	if(current_status.rising)
	{
		current_status.phasor += increment;
	}
	else
	{
		current_status.phasor -= increment;
	}
	
	// check for over/underflow indicating end of segment reached.
	// If so, truncate & return...
	if(current_status.phasor > PHASOR_MAX)
	{
		current_status.phasor = PHASOR_MAX;
		return true;		
	}
	else if(current_status.phasor < 0)
	{
		current_status.phasor = 0;
		return true;		
	}
	
	return false;
}

/*
    int16_t scalePhasor()

    This routine scales the phasor into the proper range.
    The phasor always goes from A to B by covering the 0 to 0xffff range
    But A and B may by nearer to each other than that
    and B might even be less than A, reversing things overall.
    This routine centralizes that translation.
*/
int16_t scalePhasor()
{
	int32_t range;
	int32_t offset;
	
	int32_t   result;

	// a, b are 16-bit unsigned values	
	// range and offset are 32-bit signed.
    // When a > b, range is negative
	// thus the ultimate result will be subtracted from the offset.
	range = current_status.b - current_status.a;
	offset = current_status.a;
	
	//Scale range and offset into uSec values.
	range = (range * PWM_RANGE_USEC)/ADC_MAX;
	offset = (offset * PWM_RANGE_USEC)/ADC_MAX;
		
	// scale phasor into range
	result = (current_status.phasor * range)/PHASOR_MAX;
		
	//add on offset
    // (or subtract, if a > b)
	result += offset;

	return result;
}

/* void bistableFSM()

    This FSM sits in bottom state (A) waiting for switch to close.
	When it closes, it transits to B, taking time T.
	It stays in B until the switch opens.
	It then transits back to A, again taking time T.

    On every invocation, it will update the microsecond value in current_status.
*/
void bistableFSM()
{
	int16_t delta;
	
	delta = calcDelta();
	
	switch(current_status.st)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(current_status.input == true)
			{
				current_status.st = eATOB;
				current_status.rising         = true;
			}
		}
		break;
		case eATOB:
		{
            // Switch released early
			if(current_status.input == false)
			{
				current_status.st = eBTOA;
				current_status.rising         = false;				
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
            // waiting for switch release
			if(current_status.input == false)
			{
				current_status.st = eBTOA;
				current_status.rising         = false;
			}
		}
		break;
		case eBTOA:
		{
            // Switch re-closed before reaching A
			if(current_status.input == true)
			{
				current_status.st = eATOB;
				current_status.rising         = true;
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
	
	current_status.us_val =  scalePhasor();
	
}

/* void oneshotFSM()

    This FSM sits in idle state (A) waiting for switch to close.
    When it closes, it does a complete cycle A-to-B-to-A, then waits for switch release.

    On every invocation, it will update the microsecond value in current_status.
*/
void oneshotFSM()
{
	int16_t delta;
	
	delta = calcDelta();
	
	switch(current_status.st)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(current_status.input == true)
			{
				current_status.st = eATOB;
				current_status.rising         = true;
			}
		}
		break;
		case eATOB:
		{
			// climbing up to b
			// only quits when it gets there - ignores switch
			if( calcNextPhasor(delta) )
			{
				current_status.st = eBTOA;
				current_status.rising         = false;
			}
		}
		break;
		case eATTOP:
		{
			// we shouldn't actually land here,
			// provide proper action in case we somehow do.
			current_status.st = eBTOA;
			current_status.rising         = false;
		}
		break;
		case eBTOA:
		{
			// dropping down to A
			// only quits when it gets there - ignores switch
			if( calcNextPhasor(delta) )
			{
				current_status.st = eWAIT_TO_RESET;
			}
		}
		break;
		case eWAIT_TO_RESET:
		{
			// If the switch is still held
			// wait for it's release here
			if(!current_status.input)
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
	
	current_status.us_val =  scalePhasor();
	
}

/* bool edgeDetect()

    A simple edge detector for the ctpFSM.
    Other FSMs have been level sensitive

*/
bool edgeDetect()
{
    static uint8_t deb_count = 0;

    if(current_status.input == 0)
    {
        deb_count = 0;
    }
    else if(current_status.input == 1)
    {
        
        // increment counter
        deb_count++;
                
        if(deb_count == 2)
        {
            // we call 2 in a row a rising edge
            return true;            
        }
        else if(deb_count >= 3)
        {
            // make it stick at 3 so it can't overflow
            deb_count = 3;
        }
    }

    return false;
}


/* void ctpFSM()

    Custom FSM for CTP's cuckoo bellows project.

    This FSM sits in idle state (A) waiting for switch to close.
    This is similar to the oneshotFSM, but differs a bit in the retriggering behavior.
    
    When switch closes, it does a complete cycle A-to-B.
    It then begins a B-to-A, but watches for additional switch closures.  If found,
    it will jump back to the A-to-B state.  This makes it more responsive to repeated 
    switch actuations, because it doesn't need to complete the B-to-A cycle.

    On every invocation, it will update the microsecond value in current_status.
*/
void ctpFSM()
{
	int16_t delta;
	
	delta = calcDelta();
	
    bool edge = edgeDetect();
    
	switch(current_status.st)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(edge)
			{
				current_status.st = eATOB;
				current_status.rising         = true;
			}
		}
		break;
		case eATOB:
		{
			// climbing up to b
			// only quits when it gets there - ignores switch
			if( calcNextPhasor(delta) )
			{
                // It didn't do so well when it jumped straight to BTOA...
				current_status.st = eATTOP;
				current_status.rising         = false;
			}
		}
		break;
		case eATTOP:
		{
			// we shouldn't actually land here,
			// provide proper action in case we somehow do.
			current_status.st = eBTOA;
			current_status.rising         = false;
		}
		break;
		case eBTOA:
		{
			// dropping down to A,
            // but if we see new edge, return to B.
			if(edge)
			{
    			current_status.st     = eATOB;
    			current_status.rising = true;
			}
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
	
	current_status.us_val =  scalePhasor();
	
}

/* void togglingFSM()

    ThisFSM will toggle between A and B each time the input changes.
    
    Unlike other FSMs, it takes 2 actuations to make a complete cycle.
    
    It might be useful with continuous rotation servos, which could drive 
    back and forth, changing direction with each input.  
    Automated camera slider, maybe?
    */
void togglingFSM()
{
    int16_t delta;
    
    delta = calcDelta();
    
    bool edge = edgeDetect();
    
    switch(current_status.st)
    {
        case eIDLE: 
        {
            // Advance when we see the switch actuate
            if(edge)
            {
                current_status.st = eATOB;
                current_status.rising         = true;
            }
        }
        break;
        case eATOB:
        {
            // rising to B,
            // but if we see new edge, return to A.
            if(edge)
            {
                current_status.st     = eBTOA;
                current_status.rising = false;
            }
            if( calcNextPhasor(delta) )
            {
                current_status.st = eATTOP;
            }
        }
        break;
        case eATTOP:
        {
            // Advance when we see the switch actuate
            if(edge)
            {
                current_status.st = eBTOA;
                current_status.rising         = false;
            }
        }
        break;
        case eBTOA:
        {
            // dropping down to A,
            // but if we see new edge, return to B.
            if(edge)
            {
                current_status.st     = eATOB;
                current_status.rising = true;
            }
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
    
    current_status.us_val =  scalePhasor();
    
}

/* void astableFSM()
  astable - runs back & forth while switch is held
*/
void astableFSM()
{
	int16_t delta;
	
	delta = calcDelta();
	
    bool edge = edgeDetect();
    
	switch(current_status.st)
	{
    	case eIDLE:
    	{
        	// Advance when we see the switch actuate
        	if(current_status.input == true)
        	{
            	current_status.st = eATOB;
            	current_status.rising         = true;
        	}
    	}
    	break;
    	case eATOB:
    	{
            // input goes away?  Stop where we are.
        	if(current_status.input == false)
        	{
            	current_status.st = eIDLE;
            	current_status.rising         = false;
        	}
        	else if( calcNextPhasor(delta) )
        	{
            	current_status.st = eBTOA;
            	current_status.rising         = false;
        	}
    	}
    	break;
    	case eATTOP:
    	{
        	// we shouldn't actually land here,
        	// provide proper action in case we somehow do.
        	if(current_status.input == false)
        	{
            	current_status.st = eIDLE;
            	current_status.rising         = false;
        	}
            
            // Just fall through
        	current_status.st = eBTOA;
        	current_status.rising         = false;
    	}
    	break;
    	case eBTOA:
    	{
        	// dropping down to A
        	// only quits when it gets there - ignores switch
        	if(current_status.input == false)
	        	{
    	        	current_status.st = eIDLE;
    	        	current_status.rising         = false;
	        	}
            else if( calcNextPhasor(delta) )
        	{
            	current_status.st = eATOB;
  	        	current_status.rising         = true;
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
    
    

	current_status.us_val =  scalePhasor();
	
}


// Other FSM candidates?
// monostable - like one-shot, but doesn't have to complete cycle is released before it reaches B?


// Read a given channel of ADC.
uint32_t readADC(uint8_t chan)
{
	uint32_t value;
	
	// only allow the pins we've selected to read...
	if(!( (chan == POTA_CHAN) || (chan == POTB_CHAN) || (chan == POTT_CHAN)))
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
	
    // Pots seem to act backwards, so reverse these readings
	return (0xffff - value);
}


/* void readInputs()

    Read the 3 pots and switch input.

*/
void readInputs()
{
	// read pots
	current_status.a = readADC(POTA_CHAN);
	current_status.b = readADC(POTB_CHAN);
	current_status.t = readADC(POTT_CHAN);
			
    // Input can be jumpered for active low or active high.
    // We'll correct for that here
	if(current_status.input_polarity)
	{
		// default = active low or switch closure
		// read switch - active low, so invert level
		current_status.input = !(PINA & TRIG_PIN_A_MASK);
	}
	else
	{
		// active high
		current_status.input = (PINA & TRIG_PIN_A_MASK);
	}
}

/* ISR(TIM1_CAPT_vect)

    Using some magic of AVRGCC, this gets set up as the ISR for Timer1 (aka PWM1).
 
    This is where all of the runtime work is done.

    Called when the PWM timer is restarted.
    The PWM output is set when counter is reset.
    We need to calculate when that output will be cleared.
*/
ISR(TIM1_CAPT_vect)
{
	// no hardware source to reset - flag is cleared on execution of this vector
	
	// set the pulse width based on switch and pot positions.
    //
    // run the FSM - it'll calculate the next value for the uSec timer
	if(current_status.mode)
	{
		FSMA();
	}
	else
	{
        FSMB();
	}
	
    // Apply the uSec timer to the PWM hardware.
	OCR1A = PWM_MIN_USEC + current_status.us_val;

	// Then read and prepare for next invocation	
	readInputs();

}

/* void setupPWM(void)
    
   Initialize the PWM hardware to generate pulses suitable
   for driving a hobby servo.
   
   50 mSec interval, pulses range between 1 and 2 mSec wide - servo translates
   width to position.
*/
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
	
	// Set up and read the solder jumpers
	// Both solder jumpers are normally-open, pulled up externally,
	// therefore grounded when closed.
	// Mode selects which FSM is used - 
	//     pulled up = bistable
	//     grounded  = monostable.
	// Polarity indicates polarity of the switch/ pulse input. 
	//     pulled up = active low or switch closure
	//     grounded  = active high, no pull.
	current_status.mode = (PINB & MODESEL_B_MASK);
	current_status.input_polarity = ( PINA & POLSEL_A_MASK);
	
	// set up input button pin
	if(current_status.input_polarity)
	{
		// Pulled up, active low.
		DDRA &= ~0x02; // PA1 as input
		PORTA |= 0x02; // pulled up
		
	}
	else
	{
		// Active high, no pull.
		DDRA &= ~0x02; // PA1 as input
	}

	// initialize global data
	current_status.input  = false;
	current_status.st     = eIDLE;
	current_status.phasor = 0;
	current_status.rising = true;

	// Read all of the inputs one before we enable the FSM,
	// so it doesn't start with invalid data.
	readInputs();

	// Turn off some peripherals that we're not using.
	PRR |= 0x06;//turn off USI and TIM0.

	// then enable interrupts
	sei();
	
    while(1)
    {
        // The real action all takes place in the PWM ISR.
        
		// Not sure how much difference this was making...maybe a couple mA?
		// We have to leave the timer running, so we can only idle...
		sleep_enable();
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
    }
}