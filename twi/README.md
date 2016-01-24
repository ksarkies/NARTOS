I2C Master library

Copyright 2007 Ken W. Sarkies

Target:    AVR
Author:    Ken Sarkies
Date:      19 June 2007
Licence:   GPL 2
avr-gcc:   3.4.5
Tested:    ATMega48 series

This library provides basic functions for performing master operations over the
I2C interface (designated as the Two-Wire Interface, TWI, by Atmel). The
functions provide read and write of a block (address+data) to and from a slave
device. Each function initiates an operation and returns, that is, it is
nonblocking. The operation is completed through interrupts and a flag is set
when complete. A polling function is provided to allow the calling program to
test for completion. This way the code is suitable for use with a scheduling
operating system.

The library works by maintaining a read and a write buffer. The write buffer
holds all data to be transferred including the address and device ID. Thus a
simple device read will also involve the write buffer and a change in transfer
direction will need to occur. The I2C communication works with block
transactions; the master informs the slave how many bytes it wants to be sent
during a read operation, and supplies the start address. It is possible to have
read only, write only and read/write (atomic) transactions. Much of the work is
done in the interrupt service routine working as a state machine to interpret
the bus activity. The I2C bus allows for multiple masters to contend for control
of the bus. The code at this time only handles a single master.

   1. twiInit(bitRate)  Initialise the TWI interface.
   2. twiErrorStatus() Check the error status of a transaction (whether an
error occurred and its code).
   3. twiCompletionStatus() Check the completion status of a transaction 
(whether it completed a block transfer and how many transfers remain to be
done).
   4. twiSetupTransaction(numBytes) All in and out buffer pointers are reset
and the read count is set to the number of bytes to be read. If numBytes is
zero, then the transaction is taken to be write only, otherwise it must be
read/write.
   5. twiWriteByte(dataByte) Write the next transmitted byte to the write 
buffer. If the buffer is full, the byte is not written. This call simply fills
the write buffer and no transaction takes place. The number of bytes to be
written to the device will be the number in the queue at the time the write
transaction is activated.
   6. twiReadByte() Read the next received byte from the read buffer. Before 
calling this function the calling program is expected to determine how many
bytes have been received from the read transaction, and if any error occurred.
The next read byte is returned, unless attempting to read beyond the end of the buffer 
whence an error indication is returned and the pointer is not incremented.
   7. twiLaunch(device) Launch a master transaction with a slave device. The 
calling program is expected to have setup the transaction by putting data in the
write buffer and specifying a number of bytes to be read. If there are no bytes
to write, the transaction is a master read. If there are no bytes to read 
the transaction is a master write. If there are bytes to write and read the
transaction is an atomic write/read. If there is nothing to do, the function
will return an error indication.

K. Sarkies
19/6/2007

