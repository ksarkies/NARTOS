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

#ifndef ADC_H
#define ADC_H

#ifndef F_CPU               /* CPU speed in Hz */
#define F_CPU               8000000
#endif

/** This allows clock rate to be computed in the init code, thus allowing it to
be changed during execution if desired */
#ifndef ADC_DYNAMIC_CLOCK
#define ADC_DYNAMIC_CLOCK   0
#endif
/** Set the clock rate here if it is not necessary to change it during
execution */
#ifndef ADC_CLOCK_KHZ
#define ADC_CLOCK_KHZ       100
#endif

/** MODE masks used in adcInit(). These define the bit in the mode byte
that controls the particular attribute of the conversion. */
/*@{*/
     /** Single conversion (1) Free running conversion (0) */
#define FREERUN             0x01
     /** Polling (1) Interrupts (0) */
#define INTERRUPT           0x02
     /** Sleep mode (0) */
#define SLEEP               0x04
/*@}*/
extern uint8_t adcInit(const uint8_t mode,const uint16_t adcClock);
extern uint8_t adcStart(const uint8_t channelMask);
extern int adcRead(const uint8_t channel);
extern void abortConversion(void);

/****************************************************************************/
/* Device specific settings */
#if defined(__AVR_ATmega16__) || \
    defined(__AVR_ATmega32__) || \
    defined(__AVR_ATmega8535__)

#define  ADC_CONTROL_REG            ADCSRA
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   A
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADATE
#define  AD_TRIGGER_REG             SFIOR

#elif defined(__AVR_ATmega64__)

#define  ADC_CONTROL_REG            ADCSRA
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   F
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADATE
#define  AD_TRIGGER_REG             ADSCRB

#elif defined(__AVR_ATmega128__)

#define  ADC_CONTROL_REG            ADCSRA
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   F
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADFR

#elif defined(__AVR_ATmega163__) || \
      defined(__AVR_ATtiny26__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   A
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADFR

#elif defined(__AVR_ATmega103__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   F
#define  ADC_PORT_IS_BIDIRECTIONAL  0
#define  ADC_SPECIFIC_SLEEP_MODE    0

#elif defined(__AVR_ATmega8__)

#define  ADC_CONTROL_REG            ADCSRA
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   C
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADFR

#elif defined(__AVR_ATmega48__) || \
      defined(__AVR_ATmega88__) || \
      defined(__AVR_ATmega168__)

#define  ADC_CONTROL_REG            ADCSRA
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   C
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1
#define  AD_FREERUN                 ADATE
#define  AD_TRIGGER_REG             ADSCRB

#elif defined(__AVR_AT90S8535__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   A
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    0
#define  AD_FREERUN                 ADFR

#elif defined(__AVR_AT90S4433__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     6
#define  ADC_PORT                   C
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    0
#define  AD_FREERUN                 ADFR

#else
#error DEVICE DOES NOT HAVE AN A/D CONVERTER!

#endif

/* REGISTER NAME FORMING */
#ifndef CONCAT1
#define CONCAT1(a, b) CONCAT2(a, b)
#endif

#ifndef CONCAT2
#define CONCAT2(a, b) a ## b
#endif

#define ADC_PORT_OUT_REG       CONCAT1(PORT, ADC_PORT)
#define ADC_PORT_DDR_REG       CONCAT1(DDR, ADC_PORT)
#define ADC_PORT_PIN_REG       CONCAT1(PIN, ADC_PORT)

/* Normally you shouldn't need to change the below  lines  */
/* global interrupt enable register name */
#ifndef  GLOBAL_INT_REG
#define  GLOBAL_INT_REG        SREG
#endif
/* global interrupt enable bit position */
#ifndef  GLOBAL_INT_BIT
#define  GLOBAL_INT_BIT        7
#endif

/* Convenience macros (we don't use them all) */
#define  _BV(bit) (1 << (bit))
#define  inb(sfr) _SFR_BYTE(sfr)
#define  inw(sfr) _SFR_WORD(sfr)
#define  outb(sfr, val) (_SFR_BYTE(sfr) = (val))
#define  outw(sfr, val) (_SFR_WORD(sfr) = (val))
#define  cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define  sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/** The sleep control register used by the ATMega*8 MCUs */
#if defined(SMCR)
#define  SLEEP_REG                  SMCR
#else
#define  SLEEP_REG                  MCUCR
#endif

/** Freerun mode allows the MCU to begin a new conversion after the previous
one is complete. Any changes to settings however will not correspond to the
current conversion, but to the next. That is, there is a one conversion
delay. We need to take care to match the result to the appropriate settings.

ADC_FREERUN is predefined for each MCU type and allows us to turn off free run
mode if processor doesn't support it.*/
#if !defined(ADC_FREERUN)
#undef ADC_FREERUN_MODE
#define ADC_FREERUN_MODE 0
#endif

/** Define these to allow code size to be reduced by removal of unwanted
functions. Any or all may be used. */
/*@{*/
#ifndef ADC_INTERRUPT_MODE    /**< Interrupts are used */
#define ADC_INTERRUPT_MODE  1
#endif
#ifndef ADC_FREERUN_MODE      /**< Free running scanning is not used */
#define ADC_FREERUN_MODE    0
#endif
#ifndef ADC_SLEEP_MODE        /**< Sleep mode is available */
#define ADC_SLEEP_MODE      0
#endif
/*@}*/

/** Determine the clock settings from the given kHz clock rate. If this
quantity is 1 the clock setting is determined during runtime in the
initialization function, otherwise it is predetermined. */
#if ADC_DYNAMIC_CLOCK == 0

#define ADC_CLOCK_REAL (ADC_CLOCK_KHZ*1000L)

#if   F_CPU/2 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   1
#elif F_CPU/4 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   2
#elif F_CPU/8 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   3
#elif F_CPU/16 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   4
#elif F_CPU/32 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   5
#elif F_CPU/64 < ((ADC_CLOCK_REAL*12)/10)
#define ADC_PRESCALER   6
#else
#define ADC_PRESCALER   7
#endif

#endif

#endif
