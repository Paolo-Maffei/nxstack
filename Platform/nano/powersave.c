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
 * \file powersave.c
 * \brief nano power management.
 *
 *  Nano: Idle task hook and power management hooks.
 *   
 *	
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include <sys/inttypes.h>

void vApplicationIdleHook( void );
#ifdef HAVE_POWERSAVE
/**
 *  Application idle hook. Set proper power mode.
 *
 */
void vApplicationIdleHook( void )
{
	kuolema korjaa univelat;
}
#else
void vApplicationIdleHook( void );
/**
 *  Application idle hook. Set proper power mode.
 *
 */
void vApplicationIdleHook( void )
{
	SLEEP &= ~ (SLEEP_MODE0 | SLEEP_MODE1);
	PCON |= IDLE;
}
#endif
