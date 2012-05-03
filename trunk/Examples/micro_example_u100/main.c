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
#include "semphr.h"

#include "bus.h"
#include "gpio.h"
#include "debug.h"
static void vmicro_example( void *pvParameters );

#include "socket.h"
#include "rf.h"
#include "control_message.h"

#define DIDU_BUFFER_LENGTH 59
extern xQueueHandle     buffers;
extern sockaddr_t mac_long;


sockaddr_t datasensor_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	61619
};

sockaddr_t data_address = 
{
	ADDR_BROADCAST,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	7
};


discover_res_t echo_result;

/* Here goes the code */
int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
	}	
	LED1_ON();
	xTaskCreate( vmicro_example, "Term", 250, NULL, ( tskIDLE_PRIORITY + 2 ), NULL );	
	vTaskStartScheduler();
	return 0;
}
/*-----------------------------------------------------------*/
typedef uint8_t b1w_reg[8];


int8_t ping_active=0;
portTickType ping_start;
stack_event_t stack_event;

/**
 *	Application coordinator_6lowpan task. Handles commands triggered by debug
 *  	UART interface. Task open automatic socket for data transmissio, receive and one for control-message.
 * 	Note control-socket bind want to address type ADDR_NONE!!!
 *	
 *	Available commands:
 *	m - get mac address by using control-message
 *	c - get current channel by using control-message
 *  p - send a broadcast discover message
 *	d - send data to discover device
 *
 *	\param pvParameters not used
 *
 */
static void vmicro_example( void *pvParameters )
{
	int16_t byte;
	uint8_t i=0, *ptr;
	uint8_t tx_power= RF_DEFAULT_POWER;
	uint8_t length, ind;
	buffer_t *buffer;
	buffer_t *data;
	socket_t *data_soc=0;

	buffer = 0;
	data=0;
	pause(200);
	debug_init(115200);
	pause(300);

	debug("Start.\r\n");
	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}
/*****************************************************************************************************/

/****************************************************************************************************/


	debug("Micro_example_u100\r\n\r\n");
	
	data_soc 			= socket(MODULE_CUDP, 		0);	/* Socket for response data from port number 45 */
	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */

	if (data_soc)
	{
		if (socket_bind(data_soc, &datasensor_address) != pdTRUE)
			debug("Bind fail rx.\r\n");					
	}

	datasensor_address.addr_type = ADDR_NONE;
	stack_service_init( stack_event,NULL, 0 , NULL );	/* No Gateway discover */

	for( ;; )
	{
		byte = debug_read_blocking(10);
		if(byte != -1)
		{
			switch(byte)
			{
				case 'h':
					debug("Shell help:\r\np - Send broadcast ping\r\n");
					debug("u - Send broadcast UDP echo\r\nm - Print MAC address\r\n");
					vTaskDelay(2);
					debug("+ - Increase Tx Power\r\n- - Decrease Tx Power\r\n");
					debug("c - Show current channel \r\n");
					break;
	

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

				case 'c':
					ptr=mac_get_mac_pib_parameter(MAC_CURRENT_CHANNEL);
					if(ptr)
					{
						debug("Current channel: ");
						debug_int(*ptr);
						debug("\r\n");
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
							debug("echo req\r\n");
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
					debug_put(byte);
					break;
			}	
		}
		if (data_soc)
		{
			data=0;
			data = socket_read(data_soc, 20/portTICK_RATE_MS);
			if (data)
			{
				ind=data->buf_ptr;
				if (data->dst_sa.port == 61619)
				{
					length = data->buf_end - data->buf_ptr;
					debug("Data PAC\r\n");
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

					default:

						break;
				}
				socket_buffer_free(buffer);
				buffer = 0;
			}
		}

		if (ping_active && ((xTaskGetTickCount() - ping_start)*portTICK_RATE_MS) > 500 )
		{
			ping_active = 0;
			debug("Echo timeout.\r\n");
			if(echo_result.count)
			{
				debug("Echo response: \r\n");
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
	
		}
	}
}
