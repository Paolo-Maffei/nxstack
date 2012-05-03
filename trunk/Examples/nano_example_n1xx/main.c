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
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <sys/inttypes.h>

#include "socket.h"

/*these have to be included, this way interrupt vectors get defined*/
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"
#include "control_message.h"

/*debug library*/
#include "debug.h"

extern void bus_amp(uint8_t on);
extern void bus_gain(uint8_t on);
extern void bus_reg(uint8_t on);

/*-----------------------------------------------------------*/
static void vMain(int8_t *pvParameters );

int main( void )
{
	bus_init();

	LED1_ON();
	LED2_ON();

	xTaskCreate( vMain, "Main", configMAXIMUM_STACK_SIZE, NULL, ( tskIDLE_PRIORITY + 0 ), ( xTaskHandle * )NULL );	

	vTaskStartScheduler();

	return 0;
}

sockaddr_t test_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00 },
	61619
};

sockaddr_t broadcast_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF },
	254
};

discover_res_t echo_result;

buffer_t *test_buffer, *buffer;
stack_event_t stack_event;

extern xQueueHandle buffers;

static void vMain(int8_t *pvParameters )
{
	int16_t byte;
	uint8_t count = (uint8_t) pvParameters;
	uint8_t msg;
	int8_t ping_active=0;
	uint16_t ping_counter = 0;
	portTickType ping_ticks;
	portTickType ping_start = 0;
	uint8_t flags = 1, i;
	uint8_t channel;
	
	
	debug_init(115200);
	
	debug("nano_example\r\n");
	vTaskDelay(200/portTICK_RATE_MS);
	LED1_OFF();
	LED2_OFF();
	
	stack_start(NULL);

	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 0 , NULL );	/* No Gateway discovery */
	
	channel = mac_current_channel();
	
	for( ;; )
	{
		msg = 0;
		
		byte = debug_read_blocking(2 + ((ping_active ^ 1)* 100/portTICK_RATE_MS));

		if (byte != -1)
		{
			count = 3;

			msg = 2;
	 		switch(byte)
			{
				case 'h':
					debug("Shell help:\r\np - Send broadcast ping\r\n");
					debug("u - Send broadcast UDP echo\r\nm - Print MAC address\r\n");
					break;
					
				case 'b':
					debug_int(uxQueueMessagesWaiting(buffers));
					debug(" buffers available.\r\n");
					break;	
				
				case 'C':
					if (channel < 26) channel++;
					channel++;
				case 'c':
					if (channel > 11) channel--;
					mac_set_channel(channel);
					debug("Channel: ");
					debug_int(channel);
					debug("\r\n");
					break;
						
				case 'r':
					flags ^= 1;
					bus_reg(flags & 1);
					if (flags & 1)
					{
						debug("Reg=1\r\n");
					}
					else
					{
						debug("Reg=0\r\n");
					}
					break;
						
				case 'g':
					flags ^= 2;
					bus_gain(flags & 2);
					if (flags & 2)
					{
						debug("Gain=1\r\n");
					}
					else
					{
						debug("Gain=0\r\n");
					}
					break;
						
				case 'a':
					flags ^= 4;
					bus_amp(flags & 4);
					if (flags & 4)
					{
						debug("Amp=1\r\n");
					}
					else
					{
						debug("Amp=0\r\n");
					}
					break;
						
				case 'p':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(ping(NULL, &echo_result) == pdTRUE) /* Broadcast */
						{
							ping_start = xTaskGetTickCount();
							ping_active = 2;
							debug("Ping\r\n");
						}
						else
							debug("No buffer.\r\n");
					}
					break;
					
				case 'u':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(udp_echo(NULL, &echo_result) == pdTRUE)
						{
							ping_start = xTaskGetTickCount();
							ping_active = 1;
							debug("udp echo_req()\r\n");
						}
						else
							debug("No buffer.\r\n");
					}
					break;

				case '\r':
					debug("\r\n");
					msg = 0;
					break;

				case 'm':
					{
						sockaddr_t mac;
						
						rf_mac_get(&mac);
						
						debug("MAC: ");
						debug_address(&mac);
						debug("\r\n");
					}
					break;

				default:
					debug_put(byte);
			}	
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

					case NO_ROUTE_TO_DESTINATION:
						debug("ICMP message back, no route ");
						debug("\r\n");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;
					
					case TOO_LONG_PACKET:
						debug("Payload Too Length\r\n");
						break;
			
					/*case DATA_BACK_NO_ROUTE:
						debug("DATA back, No route");
						debug("\r\n");
						debug("To ");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;*/

					default:

						break;
				}
				
				stack_buffer_free(buffer);
				buffer = 0;
			}
		}

		if ((xTaskGetTickCount() - ping_start)*portTICK_RATE_MS > 1000 && ping_active)
		{
			debug("Ping timeout.\r\n");
			stop_ping();
			if(echo_result.count)
			{
				debug("Response: \r\n");
				for(i=0; i<echo_result.count; i++)
				{
					debug_address(&(echo_result.result[i].src));
					debug(" ");
					debug_int(echo_result.result[i].rssi);
					debug(" dbm, ");
					debug_int(echo_result.result[i].time);
					debug(" ms\r\n");
				}
				echo_result.count=0;
			}
			else
			{
				debug("No response.\r\n");
			}
			test_buffer = 0;
			ping_active = 0;
		}
		if (count > 20)
		{
		 	count = 0;
	 	}
		count++;
	}
}


