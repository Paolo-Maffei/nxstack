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
 * \file event_timer.h
 * \brief Event timer API.
 *
 *  Function headers: Event timers.
 *   
 */

#ifndef _EVENT_TIMER_H
#define _EVENT_TIMER_H

extern portCHAR evtTimerAlloc(xQueueHandle queue, unsigned portCHAR size);
extern void evtTimerFree(unsigned portCHAR timer);
extern portCHAR evtTimerStart(unsigned portCHAR timer, unsigned portCHAR *tmr_data, portTickType ticks);
extern portCHAR evtTimerStop(unsigned portCHAR timer);
extern portCHAR evtTimerCheck(unsigned portCHAR timer);

#ifdef HAVE_PERIOD_TIMER
void ptTimerStart(uint16_t time_100us, void (*function)(void *));
void ptTimerStop(void);
#endif
#endif

