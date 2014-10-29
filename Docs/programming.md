# Some Notes About TINY84 Tools.

*For notes regarding getting in and out of the device using a JTAGICE3, see the /firmware/tool-notes.txt.  This file explains some of the finer points of using a Tiny Programmer (PGM-11801) and AVRDude for field reprogramming, most likely of files that were created using the IDE.*

## Prepping the Target

If you've been developing with SWD mode, you need to clear that fuse before the ISP will be active again.

Do this by loading and running an application in AVRStudio, then select "debug->quit and reenable ISP" to exit the debug session.  Simply hitting the stop button leaves it in SWD mode, which is incompatible with 

## Getting the Tools

I was using a SparkFun Tiny Programmer (PGM-11801).  Win64 Drivers were downloaded from the product page.

I soldered a female 2x3 header to the bottom of the programmer, and a male 2x3 to the top of the Servo Trigger.  When they plug in together, the servo trigger board will point away from the USB plug, so as not to interfere mechanically.

I got AVRDude in the WinAVR bundle from [sourceforge](http://sourceforge.net/projects/winavr/files/WinAVR/).  The latest release should be linked near the top of the page.  There are some notes there that the install clobbers your path - this appears to have been fixed, and my path only got longer.

The AVRDude docs are [hosted at nongnu.org](http://www.nongnu.org/avrdude/user-manual).

### Testing

You can invoke AVRDude in interactive mode to demonstrate that everything is working.

	>avrdude -t -c usbtiny -p t84

From there, the `sig` commands should return the chip device ID, or `read flash 0 10` should read 10 bytes of flash.

### Programming

The moment you've been waiting for:

	>avrdude -c usbtiny -p t84 -e -U flash:w:GccApplication1.hex

This also does a verification readback of the chip.

GccApplication1.hex is simply the hex file produced in the /debug/ or /release/ folders of the AVRStudio project.

### Verifying

Use interactive mode to `read flash 0 1000`.  It'll dump a bunch of hex.  If it's all 0xff's something went wrong.  You can alternately verify flash using the programming command line, but replacing the `:w:` with a `:v:`.

### Fuses 

You can get to the fuses using the follwing names for the memories (via the read or dump commands):

* lfuse - lower fuse byte.
* hfuse - higher fuse byte.
* efuse - extended fuse byte.

The device I was expreimenting with during the composition of this document has fuses of:

* lower: 0x7f
* upper: 0xdf
* extended: 0xff

The definition of the fuse bits is hidden in section 19.2 of the datasheet.  In short, these are mostly the defaults - notable enabled features are: SPI programming enabled, and external resonator enabled.
 