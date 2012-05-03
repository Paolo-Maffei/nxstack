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
 * \brief Example for testing beacon enable mode Gateway device. Gateway advertisesment own status to network coordinator by flooding ICMP ROUTER ADVERTISMENT message. Gateway has one socket where its waiting data from coordinator.
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
#include "semphr.h"

#include "bus.h"
#include "gpio.h"
#include "debug.h"
#include "socket.h"
#include "control_message.h"
#include "neighbor_routing_table.h"
static void vgateway( void *pvParameters );
extern sockaddr_t mac_long;

int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
	}	
	LED1_ON();
	
	xTaskCreate( vgateway, "gw", configMINIMAL_STACK_SIZE+150, NULL, ( tskIDLE_PRIORITY + 2 ), NULL );	
	vTaskStartScheduler();

	return 0;
}

/* Address to  Partner */
sockaddr_t data_address = 
{
	ADDR_802_15_4_PAN_SHORT,
	{ 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	61620
};
socket_t *data_rx=0;
stack_event_t stack_event; 
/*-----------------------------------------------------------*/

static void vgateway( void *pvParameters )
{
	buffer_t *data=0, *buffer;
	uint8_t sqn, ind, length, tx_power=RF_DEFAULT_POWER;
	uint16_t count=100;
	int16_t byte;
	stack_init_t *stack_rules=0;
	
	pause(200);
	debug_init(115200);
	pause(300);

	/* Open socket for stack status message and init that */
	stack_event 		= open_stack_event_bus();		
	stack_service_init( stack_event,NULL, 0 , NULL );	
	{
		stack_rules = pvPortMalloc(sizeof(stack_init_t));
		if(stack_rules)
		{
			memset(stack_rules, 0, sizeof(stack_init_t));
			stack_rules->channel = PAN_CHANNEL;
			stack_rules->type = BEACON_ENABLE_GATEWAY;	
			stack_rules->short_address[0] = (SHORT_ADDRESS & 0xff);
			stack_rules->short_address[1] = (SHORT_ADDRESS >> 8);
			stack_rules->pan_id[0] = (PAN_ID & 0xff);
			stack_rules->pan_id[1] = (PAN_ID >> 8);
			stack_rules->use_sw_mac=0;
			if(stack_start(stack_rules)==START_SUCCESS)
			{
				debug("Start Ok\r\n");
			}
			vPortFree(stack_rules);
		}
	}
		
	
	for( ;; )
	{
		vTaskDelay(10 / portTICK_RATE_MS );
		byte = debug_read_blocking(10);
		if(byte != -1)
		{
			switch(byte)
			{
				case '\r':
					debug("\r\n");
					break;

				case 't':
					print_table_information();
					break;

				case '+':	
					if(tx_power==100)
					{
						debug("Max Tx power set up.\r\n");
					}
					else
					{
						tx_power += 10;
						rf_power_set(tx_power);
						debug_printf("Increace Tx power, current state %d.\r\n", tx_power);
					}
					break;

				case '-':	
					if(tx_power==10)
					{
						debug("Min Tx power set up 10.\r\n");
					}
					else
					{
						tx_power -= 10;
						rf_power_set(tx_power);
						debug_printf("Decreace Tx power, current state %d.\r\n", tx_power);
					}
					break;
	
				default:
					break;
			}	
		}

		if(count++ >400)
		{	
			if(data_rx)
			{
				if(gw_advertisment() == pdFALSE)
				{
					debug("Ctr send fail\r\n");
				}
			}
			count=0;
		}

		if (data_rx)
		{
			data=0;
			data = socket_read(data_rx, 10);
			if (data)
			{
				ind=data->buf_ptr;
				if (data->dst_sa.port == 61620)
				{
					length = data->buf_end - data->buf_ptr;
					ind=data->buf_ptr;
					sqn = data->buf[ind++];
					debug_printf("DATA, %d sqn %d bytes ,",sqn,length);
					debug("\r\n--> SRC\r\n");
					debug_address(&(data->src_sa));
					debug("\r\n");
				}
				socket_buffer_free(data);
				data = 0;
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
			

					case GATEWAY_STARTED:
						data_rx = socket(MODULE_CUDP, 		0);	/* Socket for response data from port number 45 */
						if (data_rx)
						{
							if (socket_bind(data_rx, &data_address) != pdTRUE)
							{
								debug("Bind fail rx.\r\n");
							}
							else
								debug("Open Socket for Data\r\n");
						}
						data_address.port = 61619; /* Init data port for TX */;
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
	}
}
