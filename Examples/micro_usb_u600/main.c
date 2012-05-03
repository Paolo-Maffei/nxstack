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
#include "nrp.h"

static void vMain( void *pvParameters );
extern sockaddr_t mac_long;

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
			//LED2_ON();
		}
	}	
	xTaskCreate( vMain, "Main", configMINIMAL_STACK_SIZE+100, NULL, ( tskIDLE_PRIORITY + 1 ), NULL );	
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vMain( void *pvParameters )
{
	buffer_t *buffer;
	vTaskDelay(500 / portTICK_RATE_MS );
	pause(200);
	debug_init(115200);
	pause(300);
	
	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}

	buffer = stack_buffer_get(0);
	if (buffer)
	{
		buffer->from = MODULE_NRP;
		buffer->to = MODULE_NRP;
		buffer->options.type = BUFFER_CONTROL;
		buffer_push_uint8(buffer, NRPC_RESET);
		stack_buffer_push(buffer);
	}
	LED2_ON();
	vTaskDelay(500 / portTICK_RATE_MS );	
	LED2_OFF();
	LED1_OFF();
	for( ;; )
	{
		//LED1_ON();
		vTaskDelay(5000 / portTICK_RATE_MS );
		//LED1_OFF();
	}
}

