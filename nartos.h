/*
Title:    Atmel Microcontroller Not a Real Time Operating System (NARTOS)
Author:   Ken Sarkies ksarkies at trinity.asn.au
File:     $Id: nartos.h, v 0.1 6/6/2007 $
Software: AVR-GCC 3.4.5
Target:   ATMega AVR with UART, TWI and ADC functions
Tested:   ATMega48 at 8MHz, using UART, TWI.

DESCRIPTION:
Starts a task by placing its address into a task table and an index (task
ID)into the table onto a scheduling queue. The OS pulls the next index from the
queue and accesses the task address for execution. Tasks can relinquish by
putting their next execution address into their slot in the table and their ID
onto the queue. They can wait indefinitely until another task or an ISR
reschedules them.

No context apart from the execution address is saved. Programmers must ensure
that no context must be preserved across a call to relinquish or wait. This
means that these calls must be made in a top level scope, and variables to be
preserved are made global volatile.

taskStart will create a new task and give a taskID back.
One task at least must be started in the main program, which then jumps into
the nartos to execute the tasks on the queue.

A task must run for a while and then call taskRelinquish or taskWait to give
other tasks a go (usually in a loop waiting for something to happen).

taskWait suspends the task indefinitely until an ISR or another task
reschedules it with taskSchedule.

taskExit must be called when the task has completed.

The main program needs to initialise nartos and call a first task before
exiting to the OS:

int main(void)
{
  initNartos();
  firstTaskID = taskStart((uint16_t)*firstTask,FALSE);
  nartos();
}
*/

/***************************************************************************
 *   Copyright (C) 2007 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au                                               *
 *                                                                         *
 *   This file is part of Nartos                                           *
 *                                                                         *
 *   Nartos is free software; you can redistribute it and/or modify        *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   Nartos is distributed in the hope that it will be useful,             *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with V if not, write to the                                     *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

/* This defines the maximum number of tasks */
#define queueLength 8
/* This allows the use of a two-level priority, but takes more program space. */
//#define PRIORITY_QUEUE
#define NARTOSDEBUG

/* Functions declared "naked" stop any saving of context and remove the
return instruction. They are not really functions as such. */

/*-----------------------------------------------------------------------------
Relinquish control of the processor temporarily.
This drops out to the OS with the task ID placed on the queue
The return address is recovered from the stack and put into the task table
Never to be called from an ISR. */

void taskRelinquish(void) __attribute__ ((naked));
/*---------------------------------------------------------------------------
Terminate the calling task
Pop stack to remove and discard the return address put there by the call.
Remove the task entry from the task table.
Never to be called from an ISR. */

void taskExit(void)       __attribute__ ((naked));
/*---------------------------------------------------------------------------
Enter an indefinite waiting state. It is up to another task or ISR to
reschedule the task.
The return address is recovered from the stack and put into the task table
Never to be called from an ISR. */

void taskWait(void)       __attribute__ ((naked));
/*---------------------------------------------------------------------------
This is the main OS loop. Should not be called directly except at the end of
the main program.
Pop the stack to remove any return address put there by the call.
Wait if there are no tasks present. Assume that an ISR will start a task
sometime.
Get the next task off the queue and jump to its stored address by pushing
it onto the stack and executing a reti (which enables interrupts at the
same time).
taskBufferOutPointer indicates the next location with a taskID to be run,
except in the case where the queue is empty (taskBufferSize = 0).

In the high priority case, taskBufferOutPointer points to a location beyond
the first high priority entry to be accessed, therefore it is decremented
first. If it is zero then it must first be set to point to the top buffer
location. The taskBufferPrioritySize is decremented and the taskBufferSize
must be decremented also as it holds the total length of the queue. */

void nartos(void)         __attribute__ ((naked));
/*---------------------------------------------------------------------------
Initialize the OS by clearing the task table and queue pointers. */

void initNartos(void);
/*---------------------------------------------------------------------------
Start a new task.
This puts the function address pointer in the task table in a blank spot,
and allocates the task ID. Places the task ID on the queue.
Input:   address pointer to the start of the task
         boolean indicating if calling from an ISR
Output:  task ID, 0 - queueLength if valid, =0xFF if no space left. */

uint8_t taskStart(const uint16_t, const uint8_t);
/*---------------------------------------------------------------------------
Schedule a waiting task
The taskID is placed on the queue.
If the wrong taskID is used, the desired task will not be started and the
OS will probably end up scheduling a second version of the task.
If not called from an ISR, interrupts are enabled.
Input:   the current task ID. Don't get it wrong!
         boolean indicating if calling from an ISR
taskBufferOutPointer+taskBufferSize points to a location onto which a new
taskID can be placed.
Schedule a task with high priority by setting bit 7 of the taskID. */

void taskSchedule(const uint8_t, const uint8_t);

/*---------------------------------------------------------------------------*/
#ifdef NARTOSDEBUG
uint8_t getTaskBufferOutPointer(void);
uint8_t getTaskBufferSize(void);
uint16_t getTaskBuffer(const uint8_t entry);
uint16_t getTaskTable(const uint8_t entry);
uint8_t getCurrentTaskID(void);
uint16_t getCurrentAddress(void);
#endif

