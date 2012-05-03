/*
    NanoStack: MCU software and PC tools for IP-based wireless sensor networking.
		
    Copyright (C) 2006-2007 Sensinode Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

		Address:
		Sensinode Ltd.
		Teknologiantie 6	
		90570 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/**
 *
 * \file event_timer.c
 * \brief Event timer API.
 *
 *  Event timer API: allocation and tick hook function.
 *   
 */


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

#include "event_timer.h"

#ifndef HAVE_EVENT_TIMERS
#define HAVE_EVENT_TIMERS 8
#endif

void vApplicationTickHook(void);

typedef struct evtTimer
{
	xQueueHandle evtQueue;
	portTickType evtTick;
	unsigned portCHAR evtSize;
	unsigned portCHAR evtRunning;
	unsigned portCHAR evtData[8];
}evtTimer;

signed portCHAR evtInit = -1;
evtTimer evtTimers[HAVE_EVENT_TIMERS];
volatile portTickType evtTickCount;

void evtTimerInit(void);

/**
 * Init event timer system.
 *
 */
void evtTimerInit(void)
{	
	if (evtInit == -1)
	{
		unsigned portCHAR i;
		evtTickCount = 0;
		for (i=0; i<HAVE_EVENT_TIMERS ; i++)
		{
			evtTimers[i].evtQueue = 0;
			evtTimers[i].evtRunning = 0;
		}
		evtInit = 1;
	}
}

/**
 * Allocate an event timer.
 *
 * \param queue Queue to pass the data to
 * \param size  Size of event data structure
 *
 * \return -1 no free timers
 * \return timer id
 *
 */
portCHAR evtTimerAlloc(xQueueHandle queue, unsigned portCHAR size)
{
	unsigned portCHAR i;
	
	if (evtInit == -1)
	{
		evtTimerInit();
	}
	for (i=0; i<HAVE_EVENT_TIMERS ; i++)
	{
		if (evtTimers[i].evtQueue == 0)
		{
			evtTimers[i].evtQueue = queue;
			evtTimers[i].evtRunning = 0;
			evtTimers[i].evtSize = size;
			return i;
		}
	}
	return -1;
	
}

/**
 * Free an event timer.
 *
 * \param timer timer ID
 *
 *
 */
void evtTimerFree(unsigned portCHAR timer)
{
	if (timer < HAVE_EVENT_TIMERS)
	{
		evtTimers[timer].evtQueue = 0;
		evtTimers[timer].evtRunning = 0;	
	}
}

/**
 * Start the event timer.
 *
 * \param timer     timer ID
 * \param data      pointer to event data structure
 * \param ticks     time in system ticks
 *
 * \return 0 false timer ID
 * \return 1 OK
 *
 */
portCHAR evtTimerStart(unsigned portCHAR timer, unsigned portCHAR *tmr_data, portTickType ticks)
{
	if (timer < HAVE_EVENT_TIMERS) 
	{
		portENTER_CRITICAL();
		memcpy(evtTimers[timer].evtData, tmr_data, evtTimers[timer].evtSize);
		
		evtTimers[timer].evtTick = evtTickCount + ticks + 1;
		evtTimers[timer].evtRunning = 1;	
		
		portEXIT_CRITICAL();
		return 1;
	}
	return 0;
}

/**
 * Stop the event timer.
 *
 * \param timer     timer ID
 *
 * \return 0 false timer ID
 * \return 1 OK
 *
 */
portCHAR evtTimerStop(unsigned portCHAR timer)
{
	if (timer < HAVE_EVENT_TIMERS) 
	{
		evtTimers[timer].evtRunning = 0;
		return 1;
	}
	return 0;
}

/**
 * Check the event timer.
 *
 * \param timer     timer ID
 *
 * \return pdFALSE not running
 * \return pdTRUE running
 *
 */
portCHAR evtTimerCheck(unsigned portCHAR timer)
{
	if (timer < HAVE_EVENT_TIMERS) 
	{
		if (evtTimers[timer].evtRunning != 0)	return pdTRUE;
	}
	return pdFALSE;
}

#ifdef HAVE_PERIOD_TIMER
extern void period_timer_hook(void);
#endif

/**
 * Timer tick hook.
 *
 */
void vApplicationTickHook(void)
{
	unsigned portCHAR i;
	portBASE_TYPE prev_task = pdFALSE;
	
	if (evtInit == -1)
	{
		evtTimerInit();
	}
#ifdef HAVE_PERIOD_TIMER
	period_timer_hook();
#endif	
	evtTickCount++;
	
	for (i=0; i< HAVE_EVENT_TIMERS ; i++)
	{
		if (evtTimers[i].evtRunning)
		{
			if (evtTimers[i].evtTick == evtTickCount)
			{
				prev_task = xQueueSendFromISR(evtTimers[i].evtQueue, 
						evtTimers[i].evtData,
						prev_task);
				evtTimers[i].evtRunning = 0;
			}
		}
	}
}


