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

#ifndef I2C_H
#define I2C_H

#ifndef  _SFR_ASM_COMPAT          /* Special function register compatibility */
#define  _SFR_ASM_COMPAT     1
#endif

#ifndef F_CPU               /* CPU speed in Hz */
#define F_CPU               8000000
#endif

extern void twiInit(uint16_t bitRate);
extern uint8_t twiErrorStatus(void);
extern uint8_t twiCompletionStatus(void);
extern void twiSetupTransaction(uint8_t numBytes);
extern uint8_t twiWriteByte(uint8_t dataByte);
extern uint8_t twiReadByte(void);
extern uint8_t twiLaunch(uint8_t device);
extern void twiKill(void);

/****************************************************************************/
/** States of the TWI transmit/receive process. These reflect the status codes
returned by in the TWI status register for convenience. */
/*@{*/
#define TWI_IDLE 0
#define TWI_ADDRESS 2
#define TWI_DATA_PHASE 3
#define TWI_START 0x08
#define TWI_REPEATED_START 0x10
#define MT_SLA_ACK 0x18
#define MT_SLA_NACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NACK 0x30
#define MR_SLA_ACK 0x40
#define MR_SLA_NACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58
/*@}*/

/** Types of transactions */
/*@{*/
#define ATOMIC_TRANSACTION 0
#define WRITE_TRANSACTION 1
#define READ_TRANSACTION 2
/*@}*/

#define TWI_CONTROL (_BV(TWINT) | _BV(TWEN) | _BV(TWIE))
#define TWI_STOP (_BV(TWINT) | _BV(TWEN) | _BV(TWSTO))

#define BUFFER_SIZE 0x20

/* Convenience macros (we don't use them all) */
#define  _BV(bit) (1 << (bit))
#define  inb(sfr) _SFR_BYTE(sfr)
#define  inw(sfr) _SFR_WORD(sfr)
#define  outb(sfr, val) (_SFR_BYTE(sfr) = (val))
#define  outw(sfr, val) (_SFR_WORD(sfr) = (val))
#define  cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define  sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#endif
