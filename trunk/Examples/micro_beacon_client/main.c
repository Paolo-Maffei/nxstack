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
 * \brief Example for testing beacon enable mode client which discover PAN coordinator and send data. Client cuold send packet only to coordinator.
 *
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
//#include "ssi.h"
//#include "adc.h"
#include "control_message.h"
#include "neighbor_routing_table.h"


typedef uint8_t b1w_reg[8];

static void vbeacon_rfd( void *pvParameters );

#include "socket.h"
#include "rf.h"

extern sockaddr_t mac_long;
extern xQueueHandle     buffers;

sockaddr_t cord_address = 
{
	ADDR_802_15_4_PAN_SHORT,
	{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	61619
};
sockaddr_t test = 
{
	ADDR_802_15_4_PAN_SHORT,
	{ 0x00, 0x00, 0x00, 0x00,0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00 },
	61619
};

stack_event_t stack_event; 

int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
	}	
	LED1_ON();
	xTaskCreate( vbeacon_rfd, "Client", configMINIMAL_STACK_SIZE + 200, NULL, (tskIDLE_PRIORITY + 2 ), NULL );
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/
socket_t *data = 0;

static void vbeacon_rfd( void *pvParameters )
{
	buffer_t *buffer=0;
	uint8_t send=0, *ptr, i, sqn=0;
	int16_t byte;
	uint16_t count = 0;
	pause(200);
	debug_init(115200);
	pause(300);

	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 0 , NULL );	

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}
/*****************************************************************************************************/

/****************************************************************************************************/
	for (;;)
	{
		byte = debug_read_blocking(10);
		if(byte != -1)
		{
			switch(byte)
			{	
				case 'm':
					ptr=mac_get_mac_pib_parameter(MAC_IEEE_ADDRESS);
					if(ptr)
					{
						ptr +=7;
						debug("Devices mac-address: ");
						for(i=0; i< 8 ;i++)
						{
							if(i)
								debug(":");
							debug_hex(*ptr--);
						}
						debug("\r\n");
					}
					break;
				case 'b':
					debug_int(uxQueueMessagesWaiting(buffers));
					debug(" buffers available.\r\n");
					break;	

				case 'c':
					ptr=mac_get_mac_pib_parameter(MAC_CURRENT_CHANNEL);
					if(ptr)
					{
						debug("Current channel: ");
						debug_int(*ptr);
						debug("\r\n");
					}
					break;

				case '\r':
					
					debug("\r\n");
					break;

				default:
					debug_put(byte);
					break;
			}	
		}
		if(send && data)
		{	
			buffer_t *buf = socket_buffer_get(data);
			if (buf)
			{
				/* Create data packet */
				/*buf->buf[buf->buf_end++] = 'S';
				buf->buf[buf->buf_end++] = 'e';
				buf->buf[buf->buf_end++] = 'n';
				buf->buf[buf->buf_end++] = 's';
				buf->buf[buf->buf_end++] = 'o';
				buf->buf[buf->buf_end++] = 'r';
				buf->buf[buf->buf_end++] = '_';
				buf->buf[buf->buf_end++] = mac_long.address[1];
				buf->buf[buf->buf_end++] = mac_long.address[0];*/
				buf->buf[buf->buf_end++] = sqn;
				buf->buf[buf->buf_end++] = 0xc2;
				buf->buf[buf->buf_end++] = 0x00;
				buf->buf[buf->buf_end++] = 0x08;
				buf->buf[buf->buf_end++] = 0x00;
				buf->buf[buf->buf_end++] = 0x08;
				buf->buf[buf->buf_end++] = 0x00;
				buf->buf[buf->buf_end++] = 0x10;
				buf->buf[buf->buf_end++] = 0x23;
				buf->buf[buf->buf_end++] = 0x16;
				if (socket_sendto(data, &cord_address, buf) != pdTRUE)
				{	
					debug("Data send failed.\r\n");
					socket_buffer_free(buf);
					buf=0;
				}
				if(sqn==0x0f)
					sqn=0;
				else
					sqn++;
				send=0;
			}
		}
		if (count++ >= 400)
		{
			send=1;
			count = 0;
		}
		if(stack_event)
		{
			buffer=0;
			buffer = waiting_stack_event(10);
			if(buffer)
			{
				switch (parse_event_message(buffer))
				{
					case BROKEN_LINK:
						debug("Route broken to ");
						debug("\r\n");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;

					case ASSOCIATION_WITH_COORDINATOR:
						debug("Assoc ok ");
						if(get_coord_address(&cord_address) == pdTRUE)
						{
							data = socket(MODULE_CUDP, 0);	/* Socket for response data from port number 61619 */
							if (data)
							{
								if (socket_bind(data, &cord_address) != pdTRUE)
								{
									debug("Bind failed.\r\n");
								}
							}
						}
						break;
					case NOT_ASSOCIATED:
						debug("Application design error:\r\n");
						debug("RFD try send data before association\r\n");
						break;
					case ASSOCIATION_LOST:
						if(socket_close(data) == pdTRUE)
						{
							data=0;
							scan_network();
						}
						break;

					
					case TOO_LONG_PACKET:
						debug("Payload Too Length, to  ");
						debug("\r\n");
						break;
			
					default:
						break;
				}

				if(buffer)
				{
					socket_buffer_free(buffer);
					buffer = 0;
				}
			}
		}
		LED1_ON();
		LED1_OFF();
	}
}
