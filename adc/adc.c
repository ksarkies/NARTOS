/**
@mainpage Atmel AVR ADC Library
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net) adapted from code by Chris Efstathiou
@date 2 September 2007
@brief Library of A/D conversion functions for the Atmel microcontrollers.

This library provides basic functions for performing A/D conversions in the
following ways:

-# Single A/D conversion of one or a group of channels. This starts
the conversion and returns. An ISR reads the result to a temporary
location. This way either interrupt driven or polled mode can be used in
the application.

-# Free running A/D conversion. Once started the A/D converter continues to
restart automatically. Channel scanning can be used, in which case all
channels are scanned before proceeding to the next round of conversions.
The program is responsible for picking up the results before they are
overwritten. This can be useful for obtaining data at the maximum rate.

The functions are non-blocking. Once the A/D converter is started, a read
function can be used to check that conversion is complete and will return
to the main program flow.

ADC_DYNAMIC_CLOCK indicates whether the clock rate is defined in code or in
a C macro. In the former case, it can be changed on the fly by the program
if needed. Most of the time a fixed rate is adequate.

ADC_FREERUN_MODE is a mode used by the AVRs to chain ADC conversions. This
may be useful if a number of conversions are used to produce an average
value for example.

ADC_SLEEP_MODE is used to place the microcontroller into a sleep state
during conversion, to reduce digital noise on the measurements.

Some AVRs also have external or internal triggers for the ADC, this is not
used in this library.

@note Based on library by Chris Efstathiou
@note Software: AVR-GCC 3.4.5
@note Target:   ATMega AVR
@note Tested:   ATMega88 at 8MHz.
*/
/***************************************************************************
 *   Copyright (C) 2007 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au                                               *
 *                                                                         *
 *   This file is part of Acquisition                                      *
 *                                                                         *
 *   Acquisition is free software; you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   Acquisition is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Acquisition if not, write to the                           *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/
#define TRUE  1
#define FALSE 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include "adc.h"


/****************************************************************************/
/*                                Global variables
*/
/****************************************************************************/
/** Variable to carry the ADC result in a 16 bit word, accessible also as two
bytes.
*/
static volatile union adcUnion
{
  volatile uint16_t  adcInt;
  volatile uint8_t   adcByte[2];
} adc;

/** Mask of channels with valid converted data at any point of a conversion
occurring on a group of channels. This is used when the user requests
converted data, to indicate that there is indeed a conversion result available
for collection. Each bit is set by the ISR when conversion completes for the
corresponding channel, and is cleared when the result is read by the user.*/
static volatile uint8_t  adcDataValid;
static volatile uint8_t  adcBusy = FALSE;/**< ADC busy flag. */

static volatile uint16_t adcValue[NUMBER_OF_ADC_CHANNELS];/**< Result Buffer.*/
static volatile uint8_t  adcChannel;  /**< Current Channel. */
static volatile uint8_t  adcMode;     /**< Temporary mode variable. */
static volatile uint8_t  adcScanMask; /**< Temporary channel mask. */
#if ADC_FREERUN_MODE == 1
static volatile uint8_t  previousChannel;/**< Needed for free run converting.*/
static volatile uint8_t  adcChannelMask;/**< Mask kept aside for refreshing.*/
#endif

static void writeAdmux(uint8_t channel);

/****************************************************************************/
/**  @brief   Initialise A/D conversion process.

This function forces the A/D converter to prepare for conversion from a reset
state. Once it has been initialized, it can continue to be used for a series
of conversions using the same clock rate and mode.

ADC_DYNAMIC_CLOCK indicates whether the clock rate is defined in code or in a
C macro. In the former case, it can be changed on the fly by the program if
needed. Most of the time a fixed rate is adequate.

Assumes that the ADC is not busy on a conversion. This should be tested first
before calling this if changes to mode or rate are to be made on the fly.

    @param[in]  mode
            Bit 0 Free Running conversion (0 = Single conversion)
            Bit 1 Interrupts (0 = Polled)
            Bit 2 Power down, noise "cancellation" mode
            All other bit settings ignored.
            Note: Sleep mode is ineffective if more than one conversion occurs.
    @param[in]  adcClock in kHz.
    @return TRUE if successfully initialized,
*/

uint8_t adcInit(const uint8_t mode, const uint16_t adcClock)
{
  if (adcBusy) return FALSE;        /* Bomb out if not ready */
  adcMode = mode;                   /* Save this for other functions */

/** Compute and set the ADC prescaler bits if dynamic clock is selected.
The dynamic clock simply allows the clock rate to be set by the program
rather than being compiled in. */
#if ADC_DYNAMIC_CLOCK == 1
  uint16_t cpuFrequency = F_CPU/1000;
  uint8_t adcDivisorTemp = cpuFrequency/adcClock;
  uint16_t adcDivisor = 256;
  uint8_t adcPrescaler = 8;
  uint8_t x = 0;
  do
  {
    adcDivisor = adcDivisor/2;
    x = adcDivisorTemp/adcDivisor;
    adcPrescaler--;
  }
  while(x==0);
/** Allow 20% maximum over the desired adc_clock else go to a higher
adc_clock prescaler value. */
  if((cpuFrequency/adcDivisor) >= ((adcClock*12)/10)) adcPrescaler++;
  if (adcPrescaler > 8) return FALSE;
#endif

/* Make sure that all ADC work has ceased by forcing it off */
  outb(ADC_CONTROL_REG, 0);     /* Turns off ADEN - terminates conversions */
  sbi(ADC_CONTROL_REG, ADIF);   /* force clear the interrupt flag */

#if defined(PRR)
  cbi(PRR, PRADC);              /* Force out of ADC power reduction mode */
#endif

/* Enable the ADC and set the clock*/
#if ADC_DYNAMIC_CLOCK == 1
  outb(ADC_CONTROL_REG, (_BV(ADEN) | adcPrescaler));
#else
  outb(ADC_CONTROL_REG, (_BV(ADEN) | ADC_PRESCALER));
#endif

#if ADC_FREERUN_MODE == 1
/** Enable free running mode if selected. */
  if(adcMode & FREERUN)
  {
    sbi(ADC_CONTROL_REG, AD_FREERUN );
#if defined(AD_TRIGGER_REG)
    cbi(AD_TRIGGER_REG,ADTS0);
    cbi(AD_TRIGGER_REG,ADTS1);
    cbi(AD_TRIGGER_REG,ADTS2);
#endif
  }
#endif

#if ADC_INTERRUPT_MODE == 1
/* Enable interrupts */
  if(adcMode & INTERRUPT)
  {
    sbi(ADC_CONTROL_REG, ADIE);
    sei();
  }
#endif
  return TRUE;
}

/****************************************************************************/
/**  @brief  Start an A/D conversion.

This function will initiate conversion of a group of channels defined by the
channel mask. It makes use of parameters set by the adcInit function.
All channels represented by the channelMask are scanned consecutively.
The first conversion after initialization is slightly different to any later
ones, as it takes about twice as long. The ADC must be idle otherwise it
returns a NOT_READY indication.

The ISR will take over the initiation of consecutive channel scans. The
recommended action for free running mode is to change the channel in the ISR
as it is guaranteed to have allowed sufficient time for the converter to
read its configuration. However this will cause the channel to be changed for
the NEXT conversion, so we need to keep track of this.

Using sleep mode along with free running mode is problematical for this library
and is not practicable, even though it is not prevented. Here it will only
sleep for the first conversion which is discarded anyway.

The adcInit function must be called first. No checks are made on this.

@param[in]  channelMask the bit mask for channels to be read/scanned.
@return TRUE if successfully started.
*/

uint8_t adcStart(const uint8_t channelMask)
{
  if (adcBusy) return FALSE;        /* Skip out if not ready */
  sbi(ADC_CONTROL_REG, ADIF);       /* Force clear the interrupt flag */

/* Find the first channel to scan and set it. If none, bomb out.*/
  if (channelMask == 0) return FALSE;   /* Oops! - Nothing to do! */
  adcScanMask = channelMask;        /* Make global */
  adcChannel = 0;
/** Shift the adcScanMask until the first bit is a 1, to get the first channel
to convert. In the ISR we can check if it is finished when adcScanMask becomes
zero. */
  while((adcScanMask & 1) == 0)
  {
    adcChannel++;
    adcScanMask >>= 1;
  }
  writeAdmux(adcChannel);           /* set the first channel */
  adcDataValid = 0;                 /* mask of valid converted data */

#if ADC_FREERUN_MODE == 1
  previousChannel = adcChannel;     /* we need this to manage freerun mode */
  adcChannelMask = channelMask;     /* keep aside for reloading */
#endif

#if (ADC_PORT_IS_BIDIRECTIONAL == 1)
/* Set the relevant ADC channel ports to be inputs */
/* Preserve the pin directions on all ADC port pins except those mentioned in
the channel mask. Set those to inputs (0) */
  outb(ADC_PORT_DDR_REG, inb(ADC_PORT_DDR_REG) & ~channelMask);
#endif

  adcBusy = TRUE;                   /* Cleared in ISR when all conversion finished */

/** Setup the sleep mode. Different MCUs have the controls in different
registers, and generally mixed up all over the place. ATMegax8 have an SMCR
register specifically for sleeping and power control, while others put these in
the MCUCR register. Note that all three SM bits must be written to define the
ADC Noise Control mode, and we do these separately because they can occur in
different orders in the register for different MCUs.
We can convert a number of channels using sleep mode, as the ISR will change
to the next channel but won't clear the adcBusy flag. The ISR will return to this
function which will simply put the ADC back to sleep.
Enable ADC_SLEEP_MODE for the basic sleep to occur.
Enable SE to set the sleep enable bit in the sleep register.
Enable ADC_SPECIFIC_SLEEP_MODE to set the ADC noise reduction sleep mode bits in
the sleep register (otherwise they need to be setup outside this library).
Note that ADC sleep turns off the USART. If it is essential for the USART to be
active in sleep mode, do not set ADC_SPECIFIC_SLEEP_MODE and set the sleep mode
outside this function to IDLE, which is better than nothing. */
#if ADC_SLEEP_MODE == 1
  if(adcMode & SLEEP)
  {
#if defined(SE)
#if (ADC_SPECIFIC_SLEEP_MODE == 1)
    sbi(SLEEP_REG, SM0);
    cbi(SLEEP_REG, SM1);
    cbi(SLEEP_REG, SM2);
#endif
    sbi(SLEEP_REG, SE);     /* Enable sleep */
#endif
/* Go to sleep. The ADC interrupt ISR will wake up the AVR. If any other interrupt
occurs, sleep mode will return so we check to see if the conversion finished
(adcBusy is turned off in the ISR).
Don't explicitly start a conversion as it starts automatically once the MCU is
asleep.*/
    do
    {
        asm("sleep");
        asm("nop");
        asm("nop");
    }
    while (adcBusy);
  }
  else
#endif
/* If not in sleep mode, fire off the first conversion */
  {
    sbi(ADC_CONTROL_REG, ADSC);
  }
  return TRUE;
}

/****************************************************************************/
/**
   @brief   Return the result of the A/D conversion.

Reads a result back from the A/D converter, returning -1 if the result is not
ready. This function is non-blocking to allow mainline code to use an RTOS or
other scheduling method.

Blocking can be implemented simply by placing the call in a loop. On multiple
channel scans it would be wise to poll each channel in the same order as the
ISR uses, that is, lowest to highest. Be careful of free running mode as the
first channel is scanned twice immediately after activation.

The variable adcValid acts as a status indicator for the channel data. The ISR
will set the bit corresponding to the specified channel when the data has been
accessed (irregardless of whether the bit is already set), and the read
function will clear it. This allows the ISR to signal the availability of the
data, particularly during free running mode.

@param[in]   channel the channel result to be read.
@return  8 or 10 bit result, or -1 if no result is available.
*/

int adcRead(const uint8_t channel)
{
/** For polled operation, we must first check the completion flag to see if
anything interesting has happened. Then just do what the ISR would have done.
This section of the code is effectively independent of the actual channel data
access that the function is purposed to do. We just take the opportunity here
to update our results. */
  if(!(adcMode & INTERRUPT))
  {
    if ((inb(ADC_CONTROL_REG) & _BV(ADIF)) > 0) /* Check for completion */
    {
      adc.adcByte[0]=inb(ADCL);   /* result from hardware data registers */
      adc.adcByte[1]=inb(ADCH);
#if ADC_FREERUN_MODE == 1
      if  (adcMode & FREERUN)
      {
/** If the scan of all channels has finished, reload it for the next round */
        if (adcScanMask == 0) adcScanMask = adcChannelMask;
/** Store result to the previous channel selection as the channel change made
last time will not have taken effect for this cycle. */
        adcValue[previousChannel]=adc.adcValue;
        adcDataValid |= _BV(previousChannel); /* Set the valid data mask bit */
        previousChannel = adcChannel;       /* keep aside for next time */
      }
      else
#endif
      {
        adcValue[adcChannel] = adc.adcInt;  /* Store the channel result */
        adcDataValid |= _BV(adcChannel);    /* Set the valid data mask bit */
      }
      adcScanMask >>= 1;                    /* shift out current channel bit */
      if (adcScanMask == 0) adcBusy = FALSE;/* if none left, we are done */
      else
      {
/** Look for the next channel by shifting adcScanMask until the first bit is a
1, to get the next channel in order. */
        adcChannel++;
        while((adcScanMask & 1) == 0)
        {
          adcChannel++;
          adcScanMask >>= 1;
        }
        writeAdmux(adcChannel);     /* set it for the next scan */

#if ADC_FREERUN_MODE != 1
        sbi(ADC_CONTROL_REG, ADSC); /* Fire off next conversion */
#endif
      }
    }
  }
/** If the data valid flag mask is zero for the particular channel bit, do
nothing more. Otherwise clear the valid data mask and return the result.*/
  if ((adcDataValid & _BV(channel)) == 0) return -1;
  cli();                            /* make sure the ISR doesn't stuff us up */
  adcDataValid &= ~_BV(channel);    /* Clear the valid data mask bit */
  sei();
  return adcValue[channel];         /* and send back the result */
}

/****************************************************************************/
/**
   @brief   Abort any running A/D conversion.

This will simply disable the ADC. Useful for stopping a free running
conversion.
*/

void abortConversion()
{
  outb(ADC_CONTROL_REG, 0);   /* Turns off ADEN - terminates conversions */
  sbi(ADC_CONTROL_REG, ADIF); /* force clear the interrupt flag */
}

/****************************************************************************/
/**  @brief   Interrupt Service Routine

The ISR reads the result from the A/D converter and stores it in the
appropriate result array location. A result flag bit is set to indicate what
channels have been read and which are still to be completed. The next channel
in a scan, if any, is selected and another conversion started.

If freerun mode is active, then when a channel change is made, the conversion
is already taking place and the change will not take effect until the next
cycle. Therefore we must keep a record of the previous channel as the current
conversion result will be for that channel, not the current one.

The scan mask keeps a list of channels to be scanned. When a channel
conversion is completed the mask is bit shifted to find the next channel. A
flag is set for that channel in the adcDataValid byte. When the data is
retrieved for the channel, that bit is reset. If data is not retrieved in
time, it is overwritten by the next scan of the channel.
*/
#if ADC_INTERRUPT_MODE == 1

SIGNAL(SIG_ADC)
{
#if defined(SE)
  sbi(SLEEP_REG, SE);           /* Reenable specific ADC interrupt */
#endif
  sbi(ADC_CONTROL_REG, ADIF);   /* Force clear the interrupt flag */
  adc.adcByte[0]=inb(ADCL);     /* result from the hardware data registers */
  adc.adcByte[1]=inb(ADCH);
#if ADC_FREERUN_MODE == 1
  if  (adcMode & FREERUN)
  {
/** If the scan of all channels has finished, reload it for the next round */
    if (adcScanMask == 0) adcScanMask = adcChannelMask;
/** Store result to the previous channel selection as the channel change made
last time will not have taken effect for this cycle. */
    adcValue[previousChannel]=adc.adcValue;
    adcDataValid |= _BV(previousChannel); /* Set the valid data mask bit */
    previousChannel = adcChannel;         /* keep aside for next time */
  }
  else
#endif
  {
    adcValue[adcChannel] = adc.adcInt;    /* Get the result to the buffer */
    adcDataValid |= _BV(adcChannel);      /* Set the valid data mask bit */
  }
  adcScanMask >>= 1;                      /* shift out current channel bit */
  if (adcScanMask == 0) adcBusy = FALSE;  /* if none left, we are done */
  else
  {
/** Look for the next channel by shifting adcScanMask until the first bit is a
1, to get the next channel in order. */
    adcChannel++;
    while((adcScanMask & 1) == 0)
    {
      adcChannel++;
      adcScanMask >>= 1;
    }
    writeAdmux(adcChannel);               /* set it for the next scan */
/* We need to restart a conversion if neither freerun nor sleep is
enabled. If we are in sleep mode, we need to check that we are actually
asking it to sleep, and the same with freerun. */
#if ADC_FREERUN_MODE == 1
    if(!(adcMode & FREERUN))
#endif
#if ADC_SLEEP_MODE == 1
    if(!(adcMode & SLEEP))
#endif
    sbi(ADC_CONTROL_REG, ADSC);           /* Fire off next conversion */
  }
}
#endif

/****************************************************************************/
/**  @brief   Write the current ADC channel to the Multiplexer register

The other bits of the register are unchanged.

   @param[in]   channel ADC Channel number
*/

static void writeAdmux(uint8_t channel)
{
  outb(ADC_MUX_REG, ((inb(ADC_MUX_REG) & 0xF8)|(channel & 0x07)));
}

/****************************************************************************/

