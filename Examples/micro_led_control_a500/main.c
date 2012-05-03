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
 * \brief Example application for sensor I/O board led contol by buttoms, need 2 micro.u100 icluded micro.A500 module.
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

static void vmicro_led_control( void *pvParameters );
#include "socket.h"
#include "rf.h"
#include "control_message.h"
#include "neighbor_routing_table.h"

/* Message types */
#define SINK_REQUEST	0x77
#define SENSOR_DATA	0xaa

/* Control types */
#define LED_CONTROL	0xe0
#define NORMAL_DATA	0xe1
/* Control attributes */
#define BUTTOM_1	0x10
#define BUTTOM_2	0x20

extern sockaddr_t mac_long;

sockaddr_t discover_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	61619
};


/* Address to  Partner */
sockaddr_t data_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x00, 0x00, 
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	61619
};

/* Here goes the code */
int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
	}	
	

	xTaskCreate( vmicro_led_control, "Term", 250, NULL, ( tskIDLE_PRIORITY + 1 ), NULL );	
	vTaskStartScheduler();
	return 0;
}
/*-----------------------------------------------------------*/
typedef uint8_t b1w_reg[8];
buffer_t *buffer;
buffer_t *data;
socket_t *discover_socket=0, *data_rx=0;
control_message_t *msg;
int8_t ping_active=0;
portTickType ping_start;
stack_event_t stack_event;
/**
 *	Application micro_led_control task. Task send Sink request message over network which activated devices which have ligth control application.
 *	When application is activated it sen automatic control messages for control devices led when user push buttom 1 or 2. Buttom 1 turn on/off green led and Buttom 2 red led on other 
 *	application platform. Using this application example you need micro_U100 and micro_A500 sensor I/O board.
 *
 *	\param pvParameters not used
 *
 */
static void vmicro_led_control( void *pvParameters )
{
	uint8_t i=0, sink_active=0, type=0;
	uint8_t length, set_led;
	uint16_t discover_counter = 0, count=100;
	portTickType xLastWakeTime;

	pause(200);
	debug_init(115200);
	pause(300);

	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 0 , NULL );

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}

	buffer = 0;
	discover_socket = 0;
	data=0;
	P6DIR |= 0xC0;
	P6DIR &= ~0x0F;
	P6OUT |= 0xC0;
	P5DIR &= ~0x80;
	P2DIR &= ~0x01;
	debug("Sensinode Micro_light_control_a500 Shell v1.0\r\n\r\n");
	data_rx = socket(MODULE_CUDP, 		0);	/* Socket for response data from port number 45 */

	if (data_rx)
	{
		if (socket_bind(data_rx, &data_address) != pdTRUE)
		{
			debug("Bind failed.\r\n");
		}							
	}
	

	xLastWakeTime = xTaskGetTickCount();
	LED1_ON();
	vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_RATE_MS );
	LED1_OFF();

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, 80 / portTICK_RATE_MS );
		if(count >399) /* Send led control message request over network */
		{
			count=0;
			if (data_rx != 0)
			{
				buffer_t *buf = socket_buffer_get(data_rx);
				if (buf)
				{
					buf->options.hop_count = 5;
					buf->buf[buf->buf_end++] = SINK_REQUEST;
					buf->buf[buf->buf_end++] = LED_CONTROL;
					buf->buf[buf->buf_end++] = (discover_counter >> 8);
					buf->buf[buf->buf_end++] = (uint8_t) discover_counter;
					discover_address.port = 61619;
					if (socket_sendto(data_rx, &discover_address, buf) == pdTRUE)
					{
						discover_counter++;
					}
					else
					{
						debug("Discover send failed.\r\n");
						socket_buffer_free(buf);
						buf=0;
					}
				}
				else
				{
					debug("No buffer.\r\n");
				}
			}
			else
			{
				debug("No socket.\r\n");							
			}
		}
		/*read example board buttons*/
		/* If pressed Buttom 1*/
		if (P2IN & 0x01) 
		{
			while(P2IN & 0x01){}
			if (data_rx  && sink_active == 1)
			{
				
				buffer_t *buf = socket_buffer_get(data_rx);
				if (buf)
				{
					buf->buf[buf->buf_end++] = SENSOR_DATA;
					buf->buf[buf->buf_end++] = BUTTOM_1;
					if (socket_sendto(data_rx, &data_address, buf) != pdTRUE)
					{	
						debug("Data send failed.\r\n");
						socket_buffer_free(buf);
						buf=0;
					}
				}
				else
				{
					debug("No buffer.\r\n");
				}
			}
			else
			{
				debug("No socket or SINK-device.\r\n");							
			}

		}
		/* If pressed Buttom 2*/
		if (P5IN & 0x80)
		{
			while(P5IN & 0x80){}
			if (data_rx  && sink_active == 1)
			{
				
				buffer_t *buf = socket_buffer_get(data_rx);
				if (buf)
				{
					buf->buf[buf->buf_end++] = SENSOR_DATA;
					buf->buf[buf->buf_end++] = BUTTOM_2;
					if (socket_sendto(data_rx, &data_address, buf) != pdTRUE)
					{	
						debug("Data send failed.\r\n");
						socket_buffer_free(buf);
						buf=0;
					}
				}
				else
				{
					debug("No buffer.\r\n");
				}
			}
			else
			{
				debug("No socket or SINK-device.\r\n");							
			}
		}
		/* Read Data socket */
		if (data_rx)
		{
			data = socket_read(data_rx, 30);
			if (data)
			{
				uint8_t ind=0, header=0;
				ind = data->buf_ptr;
				length = data->buf_end - data->buf_ptr;
				if (data->dst_sa.port == 61619)
				{
					header = data->buf[ind++];
					if( header == SINK_REQUEST)
					{
						type = data->buf[ind++];
						if(type == LED_CONTROL)
						{
							debug_printf("Rx SINK-REQUEST, sensor data send activated, seq=%2.2x ",data->buf[ind + 1]);
							debug("\r\n");
							sink_active=1;
							for(i=0; i < 10; i++)
							{
								data_address.address[i] =data->src_sa.address[i];
							}
						}
					}
					if( header == SENSOR_DATA)
					{
						debug("Rx Sensor data from sensor device\r\n");
						debug("SRC ");
						debug_address(&(data->src_sa));
						debug("\r\n");
						set_led = data->buf[ind++];
						if(set_led==BUTTOM_1)
						{
							P6OUT ^= 0x40;
						}
						if(set_led==BUTTOM_2)
						{
							P6OUT ^= 0x80;
						}
					}
				}
				socket_buffer_free(data);
				data = 0;
			}	
		}
		if(stack_event)
		{
			buffer=0;
			buffer = waiting_stack_event(0);
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
		count++;
	}
}
