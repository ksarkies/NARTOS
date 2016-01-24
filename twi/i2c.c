/**
@mainpage Atmel AVR I2C Master Library
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net) adapted from code by Chris Efstathiou
@date 2 September 2007
@brief Library of I2C (TWI) functions for the Atmel AVR microcontrollers.

This library implements the TWI (compatible with I2C) functionality for Master
Write and Master Read, allowing access to an external I2C device such as an
EEPROM.

Two buffers are maintained. The write buffer holds the bytes to be transmitted
to the selected device, and the read buffer those to be received. All that is
needed to setup a transaction, is to place the bytes to be written in the write
buffer and to state the number of bytes to be read. The transaction is then
initiated and the type of transaction is inferred. If the number of bytes to 
be written is zero, then a master read is performed. If the number of bytes
to be read is zero, then a master write is performed. If both are nonzero then
an atomic write followed by read is performed (this is needed anyway for
reading an EEPROM as the address needs to be written first). The buffers are
not circular - each transaction must be completed before the next is initiated.

The procedure is:
-# Setup the transaction (clears the buffer pointers and sets the read count).
-# Write data to be sent into the write buffer.
-# Launch the transaction stating the number of bytes to read.
-# Determine the transaction status and the number of bytes sent/received.
-# Read received data from the read buffer.

@note Software: AVR-GCC 3.4.5
@note Target:   All Atmel MCUs with TWI functionality
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
#include "i2c.h"

uint8_t twiWriteBuffer[BUFFER_SIZE];/**< Output buffer. */
uint8_t twiReadBuffer[BUFFER_SIZE]; /**< Input buffer. */
uint8_t writeInPointer;             /**< Pointer for writing to buffer. */
uint8_t writeOutPointer;            /**< Pointer for transmitting from buffer.*/
uint8_t readInPointer;              /**< Pointer for receiving to buffer. */
uint8_t readOutPointer;             /**< Pointer for reading from buffer. */
uint8_t readCount;                  /**< Number of bytes to receive. */
uint8_t twiState = TWI_IDLE;        /**< State variable for ISR state machine.*/
uint8_t twiTransactionType;         /**< Type of transaction to execute. */
uint8_t twiLastError;               /**< Error code of latest error detected. */
uint8_t twiDevice;                  /**< Device address. */

/****************************************************************************/
/**
    @brief   Initialise the TWI interface.

This function will initialise the I2C interface with bit rate settings.

    @param[in] bitRate The bitrate to use for the TWI in kHz (maximum 400)
*/

void twiInit(uint16_t bitRate)
{
  if (bitRate > 400) bitRate = 400;
  uint16_t scale = (F_CPU/2000)/bitRate;
  uint8_t prescale = 0;
  while ((scale > 255) && (prescale < 4))
  {
    ++prescale;
    scale /= 4;
  }
  if (scale < 8) scale = 8;
  outb(TWBR,scale - 8);
  outb(TWSR,prescale);
}
/****************************************************************************/
/**
    @brief   Check the error status of a transaction.

This function will read the current state variable and report back the
transaction status in the form of an error code.

    @return a status variable:
            Not finished        0xFF
            Success             0
            or an error code if it has aborted

The software state variable is checked for the IDLE state, and for any stop
condition to have been completed. If finished, then the last error code is
returned. This is zero if no error occurred.
*/

uint8_t twiErrorStatus(void)
{
  if (twiState != TWI_IDLE)
    return 0xFF;    /* not done yet */
  return twiLastError;
}

/****************************************************************************/
/**
    @brief   Check the completion status of a transaction.

The software state variable is checked for the IDLE state, and for any stop
condition to have been completed. If finished, then a code is returned
indicating:

    @return a status variable:
            Not finished        0xFF
            Success             0
            Write interrupted   0x80 + number of bytes written
            Read interrupted    0x40 + number of bytes read
*/

uint8_t twiCompletionStatus(void)
{
  if (twiState != TWI_IDLE)
    return 0xFF;                            /* not done yet */
  if (writeInPointer != writeOutPointer)
    return 0x80+writeOutPointer;
  if (readInPointer != readCount)
    return 0x40+readInPointer;
  return 0;
}

/****************************************************************************/
/**
    @brief   Clear the read and write buffers and set the bytes to be read.

All in and out pointers are reset and the read count is set to the number of
bytes to be read. The input pointers are incremented as each byte is written to
the buffer, and the output pointers are incremented through the buffer as they
are read out. Once this is done, transmitted bytes must be first written to
the write buffer before launching the transaction.

    @param[in] numBytes number of bytes to be read
*/

void twiSetupTransaction(uint8_t numBytes)
{
  readCount = numBytes;
  readInPointer = 0;
  readOutPointer = 0;
  writeInPointer = 0;
  writeOutPointer = 0;
}

/****************************************************************************/
/**
    @brief   Write the next transmitted byte to the write buffer.

If the buffer is full, the byte is not written.

    @param[in] dataByte to be transmitted next
    @return FALSE if the buffer is full
*/

uint8_t twiWriteByte(uint8_t dataByte)
{
  if (writeInPointer > BUFFER_SIZE) return FALSE;
  twiWriteBuffer[writeInPointer++] = dataByte;
  return TRUE;
}

/****************************************************************************/
/**
    @brief   Read the next received byte from the read buffer.

The calling program is expected to determine how many bytes have been received
and if there was any error occurred before calling this function. The next read
byte is returned, unless attempting to read beyond the end of the buffer whence
0xFF is returned and the pointer is not incremented.

    @return byte from the read buffer
*/

uint8_t twiReadByte(void)
{
  if (readOutPointer > BUFFER_SIZE) return 0xFF;
  return twiReadBuffer[readOutPointer++];
}

/****************************************************************************/
/**
    @brief   Launch a master transaction with a slave device.

The calling program is expected to have setup the transaction by putting
data in the write buffer and a number of bytes to be read. If no bytes to
write, the transaction is a master read. If no bytes to read the transaction is
a master write. If bytes to write and read the transaction is an atomic
write/read. If nothing to do, bomb out.

    @param[in]  device address identifier for the slave
    @return TRUE if there was no error. An error would occur if this was
                  called in the middle of another transaction or if there was
                  nothing to do.
*/

uint8_t twiLaunch(uint8_t device)
{
  while (twiState != TWI_IDLE);
  twiLastError = 0;
  twiDevice = device & 0xFE;                /* R/W bit 0 is overwritten*/
  if (writeInPointer > 0)                   /* something to send */
  {
    if (readCount > 0) twiTransactionType = ATOMIC_TRANSACTION;
    else twiTransactionType = WRITE_TRANSACTION;
  }
  else
  {
    if (readCount > 0) twiTransactionType = READ_TRANSACTION;
    else return FALSE;                      /* nothing to do */
  }
  outb(TWCR,TWI_CONTROL | _BV(TWSTA));      /* Send a start condition */
  twiState = TWI_START;
  sei();
  return TRUE;
}

/****************************************************************************/
/**
    @brief   Force stop a transaction.

This forces a stop condition on the bus in the event that an unrecoverable
error has occurred.

*/

void twiKill(void)
{
  outb(TWCR,TWI_STOP);        /* Send a stop condition */
  twiState = TWI_IDLE;
}

/****************************************************************************/
/** @brief ISR for the TWI transaction

The main grunt in the code is in the ISR. This works through various states of
the TWI transaction. An atomic write/read transaction will convert to a read
transaction when the first write part has been completed. A repeated start will
be made.

If the device responds with a NAK, the ISR simply terminates the transaction.
The program can determine whether the transaction completed fully by
examining the states of the pointers.

Take care not to confuse twiState which controls the states of the
transaction, and twiStatus which is returned by the hardware after an event
occurs.
*/

ISR(SIG_TWI)
{
  uint8_t twiStatus = inb(TWSR) & 0xF8;     /* Mask out prescaler bits */
  switch (twiState)
  {
/** After the START condition has been transmitted and successfully
acknowledged, the ADDRESS state is setup for the next interrupt. The device
address is sent along with the R/W bit */
  case TWI_START:
    if ((twiStatus == TWI_START) || (twiStatus == TWI_REPEATED_START))
    {
      if (twiTransactionType == READ_TRANSACTION)
        outb(TWDR, twiDevice | _BV(0));     /* set up the SLA + R */
      else
        outb(TWDR, twiDevice);              /* or SLA + W */
      outb(TWCR,TWI_CONTROL);               /* send it */
      twiState = TWI_ADDRESS;
    }
/* Returned status indicates some failure of the bus */
    else
    {
      twiLastError = twiStatus;             /* Error Code from status */
      twiState = TWI_IDLE;                  /* Abort transaction */
    }
    break;

/** When the device address is successfully transmitted, prepare for reading
the first data byte */
  case TWI_ADDRESS:
    if (twiTransactionType == READ_TRANSACTION)
    {
      if (twiStatus == MR_SLA_ACK)          /* Accepted */
      {
        if (readCount == 1)                 /* check if only one */
          outb(TWCR,TWI_CONTROL);           /* Setup to NACK first byte */
        else
          outb(TWCR,TWI_CONTROL | _BV(TWEA)); /* Setup to ACK first byte */
        twiState = TWI_DATA_PHASE;
      }
/* The device indicates that it cannot provide any data, so we will stop here.
It may be that it is only temporary, but the program can try again later. */
      else if (twiStatus == MR_SLA_NACK)    /* Device cannot provide data */
      {
        outb(TWCR,TWI_STOP);                /* send Stop condition */
        twiState = TWI_IDLE;
      }
/* Returned status indicates some failure of the bus */
      else
      {
        twiLastError = twiStatus;           /* SLA transmission error */
        twiState = TWI_IDLE;                /* Abort transaction */
      }
    }
/* When the device address is successfully transmitted, prepare for writing
the first data byte */
    else                                    /* Write transaction */
    {
      if (twiStatus == MT_SLA_ACK)          /* Accepted */
      {
        outb(TWDR, twiWriteBuffer[writeOutPointer++]);/* set up first byte */
        outb(TWCR,TWI_CONTROL);             /* send it */
        twiState = TWI_DATA_PHASE;
      }
/* The device indicates that it cannot receive anything */
      else if (twiStatus == MT_SLA_NACK)    /* Device cannot accept any data */
      {
        outb(TWCR,TWI_STOP);                /* send Stop condition */
        twiState = TWI_IDLE;
      }
/* Returned status indicates some failure of the bus */
      else
      {
        twiLastError = twiStatus;           /* SLA transmission error */
        twiState = TWI_IDLE;                /* Abort transaction */
      }
    }
    break;

/** During the data read phase, each byte read must be acknowledged explicitely
on the previous data read by setting the TWEA bit to 1 for an ACK and 0 for a
NACK when sending the last byte. If the slave returns a NACK, then reading must
stop as it has no further data to send. The current data item is valid. */
  case TWI_DATA_PHASE:
    if (twiTransactionType == READ_TRANSACTION)
    {
      twiReadBuffer[readInPointer++] = inb(TWDR); /* get current byte */
      if (twiStatus == MR_DATA_ACK)         /* Accepted */
      {
        if (readInPointer == readCount)     /* Finished gracefully */
        {
          outb(TWCR,TWI_STOP);              /* send Stop condition */
          twiState = TWI_IDLE;              /* shut down transaction */
        }
        else if (readInPointer == (readCount-1))/* is next byte the last? */
          outb(TWCR,TWI_CONTROL);           /* Setup to NACK next byte */
        else
          outb(TWCR,TWI_CONTROL | _BV(TWEA)); /* Setup to ACK next byte */
      }
/* The device indicates that it cannot receive any more data */
      else if (twiStatus == MR_DATA_NACK)   /* Device cannot accept more */
      {
        outb(TWCR,TWI_STOP);                /* send Stop condition */
        twiState = TWI_IDLE;
      }
/* Returned status indicates some failure of the bus */
      else
      {
        twiLastError = MR_DATA_ACK;         /* DATA transmission error */
        twiState = TWI_IDLE;                /* Abort transaction */
      }
    }
/** During the data transmission phase, stop if all bytes have been sent,
otherwise send next byte unless receiver has responded with a NACK to indicate
it cannot accept any more data. When finished, the number of bytes remaining to
be sent is stored back in the beginning of the buffer.
If we have an atomic write/read transaction, send a repeated start when
complete, and convert to a read transaction. */
    else                                    /* Write transaction */
    {
      if (twiStatus == MT_DATA_ACK)         /* Accepted */
      {
        if (writeOutPointer == writeInPointer)  /* Finished gracefully */
        {
          if ((twiTransactionType == ATOMIC_TRANSACTION))
          {
            twiTransactionType = READ_TRANSACTION;
            outb(TWCR,TWI_CONTROL | _BV(TWSTA));  /* Send repeated start */
            twiState = TWI_START;
          }
          else
          {
            outb(TWCR,TWI_STOP);            /* Stop condition */
            twiState = TWI_IDLE;            /* shut down transaction */
          }
        }
        else
        {
          outb(TWDR, twiWriteBuffer[writeOutPointer++]); /* next data byte */
          outb(TWCR,TWI_CONTROL);           /* send it */
        }
      }
/* The device indicates that it cannot receive any more data */
      else if (twiStatus == MT_DATA_NACK)   /* Device cannot accept more */
      {
        outb(TWCR,TWI_STOP);                /* send Stop condition */
        twiState = TWI_IDLE;
      }
/* Returned status indicates some failure of the bus */
      else
      {
        twiLastError = MT_DATA_ACK;         /* DATA transmission error */
        twiState = TWI_IDLE;                /* Abort transaction */
      }
    }
  }
}
