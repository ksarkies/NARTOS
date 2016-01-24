/**
@mainpage NARTOS Embedded Operating System
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net)
@date 2 September 2007
@brief Minimal Scheduling Operating System for the Atmel microcontrollers.

NARTOS provides the capabilities of an task-oriented operating system for
embedded microcontrollers with the following features:

- Absolute minimal size.
- Lacks any real-time time-slice interrupt features.
- Provides for task creation, wait, relinquish, reschedule, exit.
- Relies on voluntary relinquishment of tasks.
- Limited priority possible.

NARTOS avoids the need to build complex state machines to provide scheduling of
activities in a small embedded microcontroller based system, thus improving
code readability but also providing additional flexibility.

A single circular buffer is used to hold the queues. A single priority level is
available and can be added through a compilation switch. The two queues are
grown from either direction in the buffer, with the high priority queue growing
backwards from the start point of the low priority queue. As the high priority
queue is always emptied before any low priority task is accessed, there is no
muddling of the queue. The low priority queue can still grow upwards, but the
start point cannot change until all the high priority stuff has gone. If the two
end pointers meet then the buffer is full. To schedule a task on the high
priority queue, set bit 7 of the taskID.

@note
Software: AVR-GCC 3.4.5
@note
Target:   ATMega AVR
@note
Tested:   ATMega88 at 8MHz.
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
 *   along with Nartos if not, write to the Free Software Foundation, Inc.,*
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/
#include <avr/sfr_defs.h>
#include "nartos.h"

#define TRUE 1
#define FALSE 0

/* Global variables */

static volatile uint8_t taskBufferOutPointer;  /**< Next to take off queue */
static volatile uint8_t taskBufferSize;        /**< Length of queue */
#ifdef PRIORITY_QUEUE
static volatile uint8_t taskBufferPrioritySize;/**< Length of priority queue */
#endif
static volatile uint8_t taskBuffer[queueLength];/**< Buffer of task IDs */
static volatile uint16_t taskTable[queueLength];/**< Table of active tasks */
static volatile uint8_t currentTaskID;  /**< Track currently executing task */
static volatile uint16_t currentAddress;

/*---------------------------------------------------------------------------*/
/** @brief Clear the task table and the queue pointers.
*/
void initNartos(void)
{
  volatile uint16_t *taskTablePointer;
  taskTablePointer = &taskTable[0];
  for (uint8_t index = 0; index < queueLength; ++index)
    *(taskTablePointer++) = 0x0000;
  taskBufferOutPointer = 0;
  taskBufferSize = 0;
#ifdef PRIORITY_QUEUE
  taskBufferPrioritySize = 0;
#endif
}

/*---------------------------------------------------------------------------*/
/** @brief Start a new task.

Places the task address in the task table in an empty spot, and schedules its
ID on the queue. Specify isr=TRUE if this is done in an ISR, so that interrupts
are not prematurely re-enabled. If the task table fills, then the task is simply
not scheduled.
@param[in] address The address pointer of the task start point.
@param[in] isr A boolean to indicate if we are being called in an ISR.
@return The index into the task table, which becomes the task ID.
*/

uint8_t taskStart(const uint16_t address, const uint8_t isr)
{
  uint8_t taskID = 0;
  volatile uint16_t* taskTablePointer = &taskTable[0];
  asm("cli\n\t"::);
//! Search the task table for an empty slot (contents = 0x0000)
  while ((*taskTablePointer != 0) && (taskID < queueLength))
  {
    ++taskTablePointer;
    ++taskID;
  }
//! Put the task address in the taskTable, if there is room
  if (*taskTablePointer == 0)
  {
    *taskTablePointer = address;
//! Schedule the task, if there is room
    taskSchedule(taskID,isr);
  }
  else taskID = 0xFF;       // Full up, no action taken
  return taskID;
}

/*---------------------------------------------------------------------------*/
/** @brief Voluntary relinquishment of a task.
 
Pop the task's next execution address off the stack and place it in the task
table at the place reserved for the task. Schedule the task on the queue for
execution at the next opportune moment.
*/

void taskRelinquish(void)
{
  uint8_t addressHigh, addressLow;
  asm( "pop %0\n\t"
       "pop %1\n\t"
       "cli\n\t"
       :"=r" (addressHigh),"=r" (addressLow):);
//  currentAddress = (uint16_t) (addressHigh << 8) + ((uint16_t) addressLow & 0xff);
  taskTable[currentTaskID] = (uint16_t) (addressHigh << 8) +
                             ((uint16_t) addressLow & 0xff);
  taskSchedule(currentTaskID,FALSE);
  nartos();
}

/*---------------------------------------------------------------------------*/
/** @brief Exit a task
 
Pop the stack to discard the next execution address, and quit. This results 
in the task execution being terminated.
*/

void taskExit(void)
{
  asm( "pop __tmp_reg__\n\t"
       "pop __tmp_reg__\n\t"
       "cli\n\t"::);
  taskTable[currentTaskID] = 0x0000;
  nartos();
}

/*---------------------------------------------------------------------------*/
/** @brief Enter a wait state.
 
Pop the task's next execution address off the stack and place it in the task
table at the designated spot. The task is not rescheduled. Another task must
do this at some stage, else the task remains indefinitely suspended.
*/

void taskWait(void)
{
  uint8_t addressHigh, addressLow;
  asm( "pop %0\n\t"
       "pop %1\n\t"
       "cli\n\t"
       :"=r" (addressHigh),"=r" (addressLow):);
  taskTable[currentTaskID] = (uint16_t) (addressHigh << 8) +
                             (uint16_t) addressLow;
  nartos();
}

/*---------------------------------------------------------------------------*/
/** @brief Schedule a new task or Reschedule a suspended task.

This schedules a task on the queue for execution at an opportune time by
computing the next free buffer location and storing the taskID. If the
buffer is filled, then the schedule does not take place. Refer to the
introduction for information about the mechanics of the priority queue.
- taskBufferSize is the total number of tasks waiting in the queue.
- taskBufferPrioritySize is the number of priority tasks waiting.
- taskBufferInPointer indicates the next free location
- taskBufferOutPointer indicates the next ID to access.
Specify isr=TRUE if this is done in an ISR to avoid prematurely enabling
interrupts.
   @param[in] taskID The identifier of the task being rescheduled.
   @param[in] isr A boolean to indicate if we are being called in an ISR.
*/
void taskSchedule(const uint8_t taskID, const uint8_t isr)
{
  asm("cli\n\t"::);
  if (taskBufferSize < queueLength)
  {
    uint8_t taskBufferInPointer;
#ifdef PRIORITY_QUEUE
    if (taskID & _BV(7))     /* Bit 7 indicates it is a priority task */
    {
      taskBufferInPointer = taskBufferOutPointer + queueLength -
                            taskBufferPrioritySize -1;
      ++taskBufferPrioritySize;
    }
    else
#endif
      taskBufferInPointer = taskBufferOutPointer+taskBufferSize;
    taskBufferInPointer %= queueLength;
    taskBuffer[taskBufferInPointer] = taskID & 0x7F;
    ++taskBufferSize;
  }
  if (! isr)           /* do not re-enable interrupts if calling from an ISR. */
    asm("sei\n\t"::);
}

/*---------------------------------------------------------------------------*/
/** @brief Operating System Entry Point
 
This is the entry to the OS. This should never be called except at the very end
of the main program that should launch one or more tasks.

The stack is popped to discard the next execution address. If the queue is
empty, there is nothing but to wait for something, maybe an ISR to reschedule
a task. Normally the main program will have already started one or more tasks
to get things ticking over.

Get the next ID waiting, find its address from the task table and push it onto
the queue to allow a jump to the address that was stored when this function
exits. Refer to the introduction for information about the mechanics of the
priority queue.
*/
void nartos(void)
{
  asm( "pop __tmp_reg__\n\t"
       "pop __tmp_reg__\n\t"
       "cli\n\t"::);
  while (taskBufferSize == 0);      // Loop and wait for something to happen
  uint8_t bufferPointer = taskBufferOutPointer;
#ifdef PRIORITY_QUEUE
  if (taskBufferPrioritySize > 0)
  {
// Take the high priority taskID from the queue
    if (taskBufferOutPointer == 0) taskBufferOutPointer = queueLength;
    bufferPointer = --taskBufferOutPointer;
    --taskBufferPrioritySize;
  }
#endif
// Take the taskID from the queue
  if (++taskBufferOutPointer >= queueLength) taskBufferOutPointer = 0;
  currentTaskID = taskBuffer[bufferPointer];
  uint16_t address = taskTable[currentTaskID];
  --taskBufferSize;
  asm( "push %0\n\t"
       "push %1\n\t"
       "reti\n\t"
       ::"r" ((uint8_t)(address&0xFF)),"r" ((uint8_t)(address>>8)&0xFF));
}

#ifdef NARTOSDEBUG
/*---------------------------------------------------------------------------*/
uint8_t getTaskBufferOutPointer(void)
{
  return taskBufferOutPointer;
}

/*---------------------------------------------------------------------------*/
uint8_t getTaskBufferSize(void)
{
  return taskBufferSize;
}

/*---------------------------------------------------------------------------*/
uint16_t getTaskBuffer(const uint8_t entry)
{
  return taskBuffer[entry];
}

/*---------------------------------------------------------------------------*/
uint16_t getTaskTable(const uint8_t entry)
{
  return taskTable[entry]<<1;
}

/*---------------------------------------------------------------------------*/
uint8_t getCurrentTaskID(void)
{
  return currentTaskID;
}
/*---------------------------------------------------------------------------*/
uint16_t getCurrentAddress(void)
{
  return currentAddress;
}
/*---------------------------------------------------------------------------*/
#endif

