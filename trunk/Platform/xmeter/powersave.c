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
 * \brief micro power saving API.
 *
 *  Micro power saving mode control and support functions.
 *   
 *	
 */


#ifdef HAVE_POWERSAVE

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>
#include <string.h>

#include "powersave.h"

power_mode_t power_current = POWER_LPM0;

power_mode_t power_handle[8];

uint8_t power_handles = 0;

void power_activate(  power_mode_t power_now );
#define power_deactivate(x) _BIC_SR(GIE+CPUOFF+OSCOFF+SCG1+SCG0);
void power_init(void)
{
	return;
}

xPowerHandle power_alloc(void)
{
	power_set(POWER_LPM0, &power_handle[power_handles]);
	return &power_handle[power_handles++];
}

void power_set(power_mode_t new_mode, xPowerHandle ph)
{
	uint8_t i;
	
	*ph = new_mode & 0x7F;
	power_current = new_mode & 0x7F;
	for (i=0; i< power_handles; i++)
	{
		if (power_handle[i] < power_current)
		{
			power_current =	power_handle[i];
		}
	}
}


#include <task.h>
void vApplicationIdleHook( void );

/**
 *  Application idle hook. Set proper power mode.
 *
 */
void vApplicationIdleHook( void )
{
  for(;;)
  {
#ifdef HAVE_IDLE_VIEW
		P5DIR |= 1;
		P5OUT |= 1;
#endif
		switch (power_current)
		{
			case POWER_LPM4:
				_BIS_SR(GIE+CPUOFF+SCG1+SCG0+OSCOFF);
				break;

			case POWER_LPM3:
				_BIS_SR(GIE+CPUOFF+SCG1+SCG0);
				break;

			case POWER_LPM2:
				_BIS_SR(GIE+CPUOFF+SCG1);
				break;
			
			case POWER_LPM1:
				_BIS_SR(GIE+CPUOFF+SCG0);
				break;			
			
			case POWER_LPM0:
				_BIS_SR(GIE+CPUOFF);
				break;
				
			default:
				break;
		}
#ifdef HAVE_IDLE_VIEW
		P5OUT &= ~1;
#endif		
    taskYIELD();
  }
}
#endif
