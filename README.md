Servo Trigger - 
========================================

[![Part Name](URL for picture of part)
*Part Name (SKU)*](URL for product on Sparkfun.com)

Small board the simplifies the control of RC servos.  Accepts input from switch closures, logic pulses, etc, and actuates a servo in response.  The start and end points, as well as thae transit time between them are all set with potentiometers.

The Servo Trigger is a small board you can use to implement simple control over hobby RC servos.  When an external switch or logic signal changes state, the servo trigger causes the servo motor to move from  position A to position B.  You also specify the time that the servo takes to make the transition. 

The Servo Trigger was designed as a collaboration with the kinetic sculptor CTP. ... bio...other details...

### Hobby Servo Control

...background on PWM for servos...

### Connections

The servo trigger has three main connections.

* In the upper left corner is the most important connection - the power input.  Tie this to a stable, regulated source of 5VDC, and it's corresponding ground.  
* Across from that, at the upper left corner, is the 3-pin connection for a standard hobby servo.  You'll want to doublecheck the color code  for your specific servo, but the common standards are as follows: 
	* White, Red, Black - Align the black wire with the `GND` pin.
	* Orange, Red, Brown - Align the brown wire with the `GND` pin.
* The third important connection is the trigger input.  In ordinary applications, you'll wire a normally open single pole switch between these contacts. 
* There is also a fourth connection - the 2x3 header for an ISP programmer.  The board should be useful for most applications without reprogramming, but in cases where it needs to be changed, you can use this header with an AVR programmer to update the firmware.

### Configuration

There are two different sets of configuration on the servo trigger.  The first configures it for a specific purpose, while the second allows you to tune the performance while you work with the Servo Trigger.

#### Initialization Config

There are two solder jumpers on the back of the board that change the behavior of the board.  These jumpers are only checked as the board initializes - being solder jumpers, good practice would suggest that you power the board off before soldering to the pads.

* The first jumper (SJ1) configures how the board responds to switch closures, by altering the Finite State Machine (FSM) used to drive the servo. 
	* SJ1 open (default) selects the "bistable" FSM.  When it first powers up, the board will drive the servo to position A.  When the input is actuated, the board will move the servo to position B.  When it reaches B, it waits for the input to clear, and once cleared, it returns to position B.  If the input is shorter than the transition time, the servo will make an incomplete cycle.
	* SJ1 closed selects the "monostable" or "one-shot" FSM.  When the input is actuated, the servo will make a complete cycle from A to B, then back to A.  It will not pause at B.  In this case, the cycle always completes, regardless of how long the input is sustained.    
	
* The second jumper (SJ2) configured the polarity of the input.  
	* SJ2 open (default) configures the input as active-low, internally pulled up.  The is intended to be used with normally-open switches (such at tact switches, microswitches or similar N.O. switches), but can also be used when triggering with external active-low logic signals.
	* SJ2 closed configures the input as active high, with no pullup.  This is intended for interfacing with external active-high logic signals. 

Because the input can be configured for active-low or active-high control, this document will use the terms "active" or "asserted" to indicate the input being driven, rather than "high" and "low."  

#### Runtime Config

There are also three potentiometers on the board, that fine tune the servo behavior.  These inputs are scanned continuously, and influence the behavior of the board immediately.

The trimpots are arranged in a row, labeled 'A', 'B' and 'T';

* 'A' selects the default position of the servo, where it rests when the input is inactive.
* 'B' selects the far endpoint position of the servo.  The exact behavior of B depends on the FSM selected by SJ1.
	* With SJ1 open, the servo will traverse to B when the input is held, and it will stay there until the input is released.
	* With SJ1 close, the servo will reach B and immediately return to A. 
* 'T' controls the time that a traversal takes, between 50 mSec and 10 seconds.

It's possible for position A to be above B - in which case the rotation will be "backwards."

### Sample Installation

The initial concept behind the Servo Trigger was to build a musical organ out of bellows & whistle assemblies removed from cuckoo clocks.  Organ keys would close switches, which would cause hobby servos to pump the bellows, making the whistle blow.  So let's consider how we would use the Servo Trigger in this application.

First, lets assume that we have a set of keys from a parlor organ which have each been fitted with N.O. microswitches.

Second, the whistle only sounds while the bellows are moving in the forward direction, and it has a limited range of travel.  Mechanically, it couldn't produce sound indefinitely if a key were held, and we want to reset the bellows quickly after the whictle has sounded.  Therefore, the monostable mode makes more sense.

Therefore, we'll configure the board with SJ2 cleared, but with a solder blob closing SJ1.

The mechanical tuning and assembly is a bit of an interactive process - we wouldn't want to have the motor over-exert the bellows, possibly breaking it.

So to start, consifure the solder blobs.  Then, attach the servo motor and switch to the Servo Trigger board, but leave the motor horn physically disconnected from the bellows.

Apply power - the servo will jump to position A when the board wakes up.  Now you can turn the pots and actuate the switch, getting the basic motion into a range that won't over-extend the bellows.

Once you're in the ballpatk, attach the bellows to the horn of the servo and fine-tune the trimpots. 
 
Finally, consider adding a dab of silicone or RTV to each of the trimpots, to hold the settings in place.

#### Troubleshooting

If there's no motion when you actuate the input, check that A and B are not the same, otherwise there's no position change!

If you're feeding the input with a logic signal from an external device, be sure to drive the signal for more than 50 milliseconds - the PWM signal is updated every 50 Sec, and events shorter than that may missed. 

It's also possible that T can be shorter than the time it take the servo motor to physically rotate.  In this case, the motor may not physically reach B before returning to A.  Try turning T up to see if a longer transition time allows the motor to turn.

Power supply woes...


### Going Further

The firmware on the Servo Trigger was programmed using Atmel Studio 6.6, using AVRGCC 3.4.4.1229 and a JTAGICE3 debug module.

If the provided FSMs don't meet your needs, you can implement custom ones - follow the pattern used by the functions bistableFSM() and oneshotFSM().

Similarly, if the range of times provided by default isn't quite right, the timing is controlled by a lookup table, which is generated by the _translation.ods_ spreadsheet.  Type the desired times into the green cells, then cut and paste the yellow cells into the 'timelut' array.  The firmware performs linear interpolation between the entries. 

---


[The datasheet can be found here.](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/PS-MPU-9150A.pdf)

More information can be found on the Wiki for this repo. 

This part was created in Eagle vXXX, this firmware was created in Arduino vBlahBlahBlah, etc. 




Repository Contents
-------------------

* **/Firmware** - Any firmware that the part ships with, 
* **/Fritzing** - Fritzing Example wiring images
* **/Hardware** - All Eagle design files (.brd, .sch, .STL)
* **/Production** - Test bed files and production panel files
* **[Wiki](URL for GitHub Wiki) - Wiki with examples or helpful information for product

Product Versions
----------------
* [Part SKU](part URL)- Basic part and short description here
* [Retail part SKU] (retail URL)- Retail packaging of standard description here
* [Any other parts this repo covers](any other URLs) - Description of said parts

Version History
---------------
* [vExxFxxZxxHxxLxxSxx](URL for tag specific to this version) description 
* [vEyyFyyZyyHyyLyySyy](URL for tag specific to this version) description

License Information
-------------------
The hardware is released under [Creative Commons ShareAlike 4.0 International](https://creativecommons.org/licenses/by-sa/4.0/).
The code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.

->Any collaboration credit should appear here.<-

