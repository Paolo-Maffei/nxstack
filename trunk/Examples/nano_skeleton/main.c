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
 * \file main.c
 * \brief Example bare skeleton for starting a new Nano series application.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <sys/inttypes.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* NanoStack includes */
#include "stack.h"
#include "socket.h"
#include "debug.h"

/* Platform includes */
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"

static void vAppTask( int8_t *pvParameters );

/* Setup a default address structure, short address, broadcast, to port 125 */
sockaddr_t broadcast = 
{
	ADDR_802_15_4_PAN_SHORT, 
	{ 0xff, 0xff, 0xff, 0xff },
	125
};

socket_t *broadcast_socket=0;

/* Main task, initialize hardware and start the FreeRTOS scheduler */
int main( void )
{
	/* Initialize the Nano hardware */
	LED_INIT();
	bus_init();

	/* Setup the application task and start the scheduler */
	xTaskCreate( vAppTask, "App", configMINIMAL_STACK_SIZE+200, NULL, (tskIDLE_PRIORITY + 2 ), ( xTaskHandle * )NULL );
	vTaskStartScheduler();
	
	/* Scheduler has taken control, next vAppTask starts executing. */

	return 0;
}

/** 
 * Skeleton application task 
 */
static void vAppTask( int8_t *pvParameters )
{
	
	/* Start the debug UART at 115k */
	debug_init(115200);
	vTaskDelay( 200 / portTICK_RATE_MS );

	/* Initialize NanoStack with default parameters, NanoStack task automatically created. */
	{
		if(stack_start(NULL)==START_SUCCESS)
		{
			debug("NanoStack Start Ok\r\n");
		}
	}
	LED1_ON();
	vTaskDelay( 500 / portTICK_RATE_MS );
	LED1_OFF();

	/* Open and bind a socket */
	
	broadcast_socket = socket(MODULE_CUDP, 0);
	if (broadcast_socket) {
		if (socket_bind(broadcast_socket, &broadcast) != pdTRUE) debug("Socket bind failed.\r\n");
	}	
	
	/* Start an endless task loop, we must sleep most of the time allowing execution of other tasks. */
	for (;;)
	{
		/* Send a packet */
		buffer_t *buf = socket_buffer_get(broadcast_socket);
		if (buf) {
			buf->buf_end=0;
			buf->buf_ptr=0;
			buf->options.hop_count = 1;
			buf->buf[buf->buf_end++] = 'T';
			buf->buf[buf->buf_end++] = 'e';
			buf->buf[buf->buf_end++] = 's';
			buf->buf[buf->buf_end++] = 't';
			buf->buf[buf->buf_end++] = '!';
		}
		if (socket_sendto(broadcast_socket, &broadcast, buf) == pdTRUE) {
			debug("Packet sent\r\n");
		}
	
		/* Sleep for 1000 ms */
		vTaskDelay( 1000 / portTICK_RATE_MS );
	}
}


