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


/*
    NanoStack: MCU software and PC tools for sensor networking.
		
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
		PO Box 1
		90571 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/* Standard includes. */
#include <stdlib.h>
#include <signal.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bus.h"
#include "gpio.h"
#include "debug.h"

static void vMain( void *pvParameters );

#include "socket.h"

int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
		for (;;)
		{
			LED1_ON();
			LED2_OFF();
			LED1_OFF();
			LED2_ON();
		}
	}	
	
//	LED1_ON();
	
	xTaskCreate( vMain, "Main", 256, NULL, ( tskIDLE_PRIORITY + 1 ), NULL );	
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vMain( void *pvParameters )
{
	buffer_t *buffer;

	debug_init(115200);
	stack_init();

	debug("Sensinode micro_libpcap v1.0\r\n\r\n");

	for( ;; )
	{
		vTaskDelay(5000 / portTICK_RATE_MS );
#ifdef HAVE_DEBUG
		debug("Alive\r\n\r\n");
#endif
	}
}

