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

extern void     twiSlaveInit(uint16_t bitRate, uint8_t id);
extern uint8_t  twiErrorStatus(void);
extern uint8_t  twiWriteByte(uint8_t dataByte);
extern uint8_t  twiReadCount(void);
extern uint8_t  twiReadByte(void);
extern uint8_t  twiSlaveLaunch(void);

/****************************************************************************/
/* Convenience macros (we don't use them all) */
#define  _BV(bit) (1 << (bit))
#define  inb(sfr) _SFR_BYTE(sfr)
#define  inw(sfr) _SFR_WORD(sfr)
#define  outb(sfr, val) (_SFR_BYTE(sfr) = (val))
#define  outw(sfr, val) (_SFR_WORD(sfr) = (val))
#define  cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define  sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/** States of the TWI transmit/receive process. */
/*@{*/
#define TWI_IDLE        0x00
#define TWI_WRITING     0x01
#define TWI_READING     0x02
/*@}*/

/** Error Codes */
/*@{*/
#define TWI_OK          0x00
#define TWI_RX_BUFFER_FULL  0x01
#define TWI_BUS_FAULT   0x02
/*@}*/

/* Enable the TWI interface and but leave inactive */
#define TWI_SLAVE_NACK  (_BV(TWEN))
#define TWI_INACTIVE    (_BV(TWEN))
/* TWI Interface enabled. Enable TWI Interrupt and clear the flag. */
#define TWI_ENABLE      (_BV(TWEN) | _BV(TWIE) | _BV(TWINT))
/* Prepare to ACK next time the Slave is addressed. */
#define TWI_SLAVE_ACK   (_BV(TWEA) | TWI_ENABLE)
#define TWI_ACTIVE      (_BV(TWEA) | TWI_ENABLE)

#define BUFFER_SIZE 0x20

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes                      
#define TWI_START                  0x08  // START has been transmitted  
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes                      
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver status codes  
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = 0
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#endif
