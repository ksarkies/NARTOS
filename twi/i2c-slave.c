/**
@mainpage Atmel AVR I2C Slave Library
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net) adapted from code by Atmel AVR311
@date 1 October 2011
@brief Library of I2C (TWI) slave functions for the Atmel AVR microcontrollers.

This library implements the TWI (compatible with I2C) functionality for Slave
Write and Slave Read.

Two circular buffers are maintained. The write buffer holds the bytes to be
transmitted to the selected device, and the read buffer those that have been
received.

The ISR handles the transactions. When the TWI is initialised to recognize
its address and interrupts are enabled, the ISR expects a command to be sent
by the master. This may have a number of parameters. These are all placed into
the read buffer before the transaction is completed.

If the master has asked for data, it will send a subsequent read request to
retrieve the data as the slave may take some time before it is ready. A global
variable numberWriteBytes is used to indicate to the master how many bytes are
available for sending. This is always transmitted as the first byte, and will
allow the master to gracefully end the transaction. Any subsequent data written
to the transmit buffer will be held until the next master request.

If the read buffer fills, the slave will send a NACK to the master but the last
datum received will be held in a temporary buffer. When a receive buffer
becomes available, this data will be transferred to it.

An incoming frame finishes when the master sends a STOP. The TWI is disabled
and a flag is set. When the entire frame has been read out of the buffer, the
buffer is reset and the TWI re-enabled. This should allow seamless synchronization
with the master communications.
TBD currently the ISR sets a flag at the end of the read transaction and disables
the TWI until the main program has read all bytes from the buffer. It would be more
general and robust to flag the buffer location with the last byte, which means we
need to use a 16 bit buffer with the data byte and a status flag.

General calls are recognised.

In this implementation there is no need to check arbitration situations as
the slave will not be acting as a master.

@note Software: AVR-GCC 3.4.5
@note Target:   All Atmel MCUs with TWI functionality
@note Tested:   ATMega88 at 8MHz.
*/
/***************************************************************************
 *   Copyright (C) 2011 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au                                               *
 *                                                                         *
 *   This file is part of Acqnew.                                      *
 *                                                                         *
 *   Acqnew is free software; you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   Acqnew is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Acqnew if not, write to the                           *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

#define TRUE  1
#define FALSE 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c-slave.h"

uint8_t twiWriteBuffer[BUFFER_SIZE];/**< Output buffer. */
uint8_t twiReadBuffer[BUFFER_SIZE]; /**< Input buffer. */
uint8_t writeInPointer;             /**< Pointer for writing to buffer. */
uint8_t writeOutPointer;            /**< Pointer for transmitting from buffer.*/
uint8_t readInPointer;              /**< Pointer for receiving to buffer. */
uint8_t readOutPointer;             /**< Pointer for reading from buffer. */
uint8_t twiState;                   /**< State variable for ISR state machine.*/
uint8_t twiLastError;               /**< Error code of latest error detected. */
uint8_t receivedData;               /**< Temporary received data store in case buffer full */
uint8_t readBufferOverflow;         /**< Flag to manage buffer full condition */
uint8_t numberWriteBytes;           /**< Number of bytes in write buffer for transmission */
uint8_t numberToSend;               /**< Number of bytes in write buffer for current transaction */

/****************************************************************************/
/**
    @brief   Initialise the TWI interface.

This function will initialise the TWI interface with bit rate settings and
also enable the TWI interrupts and address recognition. Incoming messages will
not be acted upon until global interrupts are enabled. All buffer and state
variables are initialised. This will be called on reset of the processor,
but may also be called by a master command to synchronize the interface.

    @param[in] bitRate The bitrate to use for the TWI in kHz (maximum 400)
*/

void twiSlaveInit(uint16_t bitRate, uint8_t id)
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
  outb(TWAR,id | 1);            /* Set own TWI slave address, and accept TWI General Calls.*/
  outb(TWDR,0xFF);
  outb(TWCR,TWI_INACTIVE);      /* Enable the TWI interface and address recognition */
  writeInPointer = 0;
  writeOutPointer = 0;
  readInPointer = 0;
  readOutPointer = 0;
  twiState = TWI_IDLE;
  twiLastError = 0;
  readBufferOverflow = FALSE;
  numberWriteBytes = 0;
}

/****************************************************************************/
/**
    @brief   Check if the read buffer has data.

    @return Returns number of bytes in the buffer.
*/

uint8_t twiReadCount(void)
{
  return (readOutPointer + BUFFER_SIZE - readInPointer) % BUFFER_SIZE;
}
/****************************************************************************/
/**
    @brief   Read the next received byte from the read buffer.

The next read byte is returned. If the buffer is empty, just return with a flag.
If an overflow had occurred, move the saved data to the buffer for next time.
When the master transaction has completed the ISR should change the TWI_STATE.
If this is set and the buffer has no more data, then flag this and re-enable the
TWI.

    @return word consisting of byte from the read buffer in the lower byte
            and a status byte in the upper byte
            Status: 0x00  OK
                    0x80  buffer empty
                    0x40  last byte of data frame
*/

uint8_t twiReadByte(void)
{
  uint8_t nextDatum;
/* Stop interrupts to prevent changes being made while doing this */
  cli();
/* Buffer empty, just return rubbish. Calling program should manage this
a bit better. */
  if (readOutPointer == readInPointer) nextDatum = 0xFF;
  else
  {
    nextDatum = twiReadBuffer[readOutPointer];
    readOutPointer = (readOutPointer+1) % BUFFER_SIZE;
/* Check if an overflow had occurred and move temporary data to buffer */
    if (readBufferOverflow)
    {
      twiReadBuffer[readInPointer] = receivedData;
      readInPointer = ((readInPointer+1) % BUFFER_SIZE);
      readBufferOverflow = FALSE;
    }
/* When the master finishes the transaction the ISR will set the idle state
and stop any further transfers from the master. When the last byte is read
out, we then flag this as the last of the frame, and restart the TWI. */
    if (twiState == TWI_IDLE)
    {
      if (readOutPointer == readInPointer)
      {
        nextDatum += 0x4000;
        twiSlaveLaunch();
      }
    }
/* If not frame end and the buffer has room, clear any pending NACK to the
master just in case one was set. */
    else if (((readInPointer+1) % BUFFER_SIZE) != readOutPointer)
      outb(TWCR,TWI_SLAVE_ACK);
  }
/* Re-enable interrupts */
  sei();
  return nextDatum;
}

/****************************************************************************/
/**
    @brief   Write to the write buffer the next byte to be transmitted.

If the buffer is full, the byte is not written. A global parameter
numberWriteBytes is incremented if the write is successful. This is used by
the ISR when responding to a master read request, to tell the master how
many bytes will be transferred.

    @param[in] dataByte to be transmitted next
    @return FALSE if the buffer is full
*/

uint8_t twiWriteByte(uint8_t dataByte)
{
  cli();
  uint8_t nextWritePointer = (writeInPointer + 1) % BUFFER_SIZE;
  uint8_t bufferFree = (nextWritePointer != writeOutPointer);
  if (bufferFree)
  {
    twiWriteBuffer[writeInPointer] = dataByte;
    writeInPointer = nextWritePointer;
    numberWriteBytes++;
  }
  sei();
  return bufferFree;
}

/****************************************************************************/
/**
    @brief   Ready the slave TWI to receive data or restart a transmission.

The TWI is setup for interrupts and address recognition, and waits to be addressed.

    @return FALSE if this was called in the middle of another transaction.
*/

uint8_t twiSlaveLaunch(void)
{
  if (twiState != TWI_IDLE) return FALSE;       /* Return if transceiver is busy */
  twiLastError = 0;
  outb(TWCR,TWI_SLAVE_ACK);
  sei();
  return TRUE;
}

/****************************************************************************/
/**
    @brief   Check the error status of a transaction.

This function will check if the interface is idle and report back the
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
    return 0xFF;                          /* not done yet */
  return twiLastError;
}

/****************************************************************************/
/** @brief ISR for the TWI transaction

The main grunt in the code is in the ISR. This works through various states of
the TWI transaction.

Take care not to confuse twiState which controls the states of the
transaction, and twiStatus which is returned by the hardware after an event
occurs, to identify the nature of the event.
*/

ISR(SIG_TWI)
{
/* Mask out the lower three bits of the status register */
  uint8_t twiStatus = (inb(TWSR) & 0xF8);
/* Get the next read buffer address for testing if buffer is full */
  uint8_t nextReadInPointer = ((readInPointer+1) % BUFFER_SIZE);

/* WRITE REQUEST */
/* A general call address or own SLA+W has been received; ACK has been returned.
The master should send subsequent data in the same transaction. */
  if ((twiStatus == TWI_SRX_GEN_ACK) || (twiStatus == TWI_SRX_ADR_ACK))
  {
/* Setup TWI to acknowledge further receives unless the receive buffer is full.
The slave should receive the next data byte but will return a NACK unless the
buffer read function has cleared it in the meantime. */
    if (nextReadInPointer == readOutPointer)
      outb(TWCR, TWI_SLAVE_NACK);       /* This should make the master pause */
    else
      outb(TWCR, TWI_SLAVE_ACK);
    twiState = TWI_READING;
  }
/* Previously addressed with general call or own SLA+W; data has been received;
ACK has been returned */
  else if ((twiStatus == TWI_SRX_GEN_DATA_ACK) || (twiStatus == TWI_SRX_ADR_DATA_ACK))
  {
/* Store data and setup TWI to acknowledge further receives. If the receive buffer
becomes full we must NACK the master so that the next byte sent can be stored
aside until a space becomes available in the read buffer. */
    readBufferOverflow = FALSE;
    twiReadBuffer[readInPointer] = inb(TWDR);
    if (nextReadInPointer == readOutPointer)
    {
      twiLastError = TWI_RX_BUFFER_FULL;  /* Buffer full */
      outb(TWCR, TWI_SLAVE_NACK);
    }
    else
    {
      twiLastError = 0; 
      outb(TWCR, TWI_SLAVE_ACK);
    }
    readInPointer = nextReadInPointer;
  }
/* Previously addressed with own SLA+W and data has been received; NACK has been
returned. Deactivate the TWI until the reading has caught up. The master may try
again but will not get a response just yet. */
  else if ((twiStatus == TWI_SRX_GEN_DATA_NACK) || (twiStatus == TWI_SRX_ADR_DATA_NACK))
  {
/*  The read function is signalled to transfer the received data to the buffer
when a place becomes available. */
    receivedData = inb(TWDR);           /* Keep aside for now */
    readBufferOverflow = TRUE;          /* Signal to read function to grab this */
    outb(TWCR, TWI_INACTIVE);           /* Disable the TWI until we can take more data */
  }
/* A STOP condition or repeated START condition has been received indicating end
of transaction. Set state to end of frame and disable the interface until the
slave has finished processing. */
  else if (twiStatus == TWI_SRX_STOP_RESTART)
  {
    twiState = TWI_IDLE;
    outb(TWCR, TWI_INACTIVE);
  }

/* READ REQUEST */
/* Read from slave requested, SLA+R received and ACK'ed, or data byte in TWDR has
been transmitted; ACK has been received. The master should have set up with an
earlier command the data to be sent, and the slave should have the data prepared.
Grab the number of bytes available in the read buffer to send, and pop it off as
the first byte. */
  else if ((twiStatus == TWI_STX_ADR_ACK) || (twiStatus == TWI_STX_DATA_ACK))
  {
/* If the number has been sent or the buffer is empty (these should occur together)
just send dummy FFs. */
    if ((writeOutPointer == writeInPointer) || (numberWriteBytes == 0))
      outb(TWDR,0xFF);
    else
    {
/* Send the next byte and decrement counters */
      outb(TWDR,twiWriteBuffer[writeOutPointer]);
      writeOutPointer = ((writeOutPointer+1) % BUFFER_SIZE);
      numberWriteBytes--;
    }
    outb(TWCR, TWI_SLAVE_ACK);
  }
/* Data byte in TWDR has been transmitted; NACK has been received. Take this as
a signal to stop sending and await the next master initiated read transaction
to finish off. The master is exercising flow control most likely and is required
to start again. */
  else if (twiStatus == TWI_STX_DATA_NACK)
  {
    outb(TWCR, TWI_INACTIVE);
    twiState = TWI_IDLE;
  }
/* Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received
although it really shouldn't be. Just disable the bus. */
  else
  {
    outb(TWCR, TWI_INACTIVE);
    twiLastError = TWI_BUS_FAULT;
    twiState = TWI_IDLE;
  }
}
