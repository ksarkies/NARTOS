A/D converter Library

Copyright 2007 Ken W. Sarkies

Target:    AVR
Author:    Ken Sarkies
Date:      19 June 2007
Licence:   GPL 2
avr-gcc:   3.4.5
Tested:    ATMega48, ATMega88

This library, based on that of Chris Efstathiou, provides basic functions for
A/D conversion using interrupt/polled and noise reduction sleep operations.
Support for continuous operation is also provided.

   1. Single A/D conversion of one or a group of channels. This starts the
conversion and returns. An ISR reads the result to a temporary location. This
way either interrupt driven or polled mode can be used in the application.

   2. Free running A/D conversion. Once started the A/D converter continues to
restart automatically. Channel scanning can be used, in which case all channels
are scanned before proceeding to the next round of conversions. The program is
responsible for picking up the results before they are overwritten. This can be
useful for obtaining data at the maximum rate.

The functions are non-blocking. Once the A/D converter is started, a read
function can be used to check that conversion is complete and will return to the
main program flow.

Some AVRs also have external or internal triggers for the ADC, this is not used
in this library.

   1. adcInit(mode,adcClock) Initialise A/D conversion process. This function
will initialise the ADC with the mode of operation and the clock rate to be
used. An error will be returned if the ADC is busy.
    Mode:
            Bit 0 Free Running conversion (0 = Single conversion)
            Bit 1 Interrupts (0 = Polled)
            Bit 2 Power down, noise "cancellation" mode
            All other bit settings ignored.
            Note: Sleep mode is ineffective if more than one conversion occurs.
    adcClock: in kHz.
    Returns TRUE if successfully initialized,

   2. adcStart(channelMask)  Start an A/D conversion. This function will
initiate an A/D conversion on a set of channels given by a channel mask (a 1 in
each bit corresponding to the A/D channel 0-MAX).
    channelMask: the bit mask for channels to be read/scanned.
    Returns TRUE if successfully started.

   3. adcIsBusy() Check Progress of an A/D conversion.
    Returns TRUE if busy

   4. adcRead(channel)  Return the result of the A/D conversion. This will check
if a result is ready for a given channel, and return the result or -1 if not
available. For the AVRs the result will have a maximum precision of 10 bits.
   channel: the channel result to be read.
   Returns 8 or 10 bit result, or -1 if no result is available

   5. abortConversion() Abort any running A/D conversion.


K. Sarkies
19/6/2007

