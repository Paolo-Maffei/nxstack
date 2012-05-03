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
 * \brief 6lowpan Interop Level 0 and Level 1 Terminal
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

static void v6lowpan_interop( void *pvParameters );
#include "control_message.h"
//#include "neighbor_routing_table.h"
#include "socket.h"
#include "rf.h"

portCHAR check_udp_socket(buffer_t *buffer);
portCHAR check_icmp_socket(buffer_t *buffer);
extern sockaddr_t mac_long;

sockaddr_t unicast_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	UDP_ECHO_PORT
};



/* Main */
int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
	}	
	LED1_ON();

	xTaskCreate( v6lowpan_interop, "Term",356 , NULL, ( tskIDLE_PRIORITY + 1 ), NULL );	
	vTaskStartScheduler();
	return 0;
}

portTickType ping_start;
discover_res_t echo_result;
int8_t ping_active=0;
mac_param_t mac_set_req;

stack_event_t stack_event; 
/*-----------------------------------------------------------*/
typedef uint8_t b1w_reg[8];
/**
 *	Application simple_6lowpan task. Handles commands triggered by debug
 *  	UART interface. Task open automatic socket for data transmissio, receive and one for control-message.
 * 	Note control-socket bind want to address type ADDR_NONE!!!
 *	
 *	Available commands:
 *	m - get mac address by using control-message
 *	c - get current channel by using control-message
 *	C - increase the channel
 *  p - send ping a broadcast, only for neighbor
 *  P - send ping a broadcast multi hop
 *  r - Give unicast MAC address
 *  b - Broadcast mode
 *  B - Unicast mode
 *	+ - Increase Transmit power 10 %
 *	- - Decrease Transmit power 10 %
 *
 *	\param pvParameters not used
 *
 */
static void v6lowpan_interop( void *pvParameters )
{
	
	buffer_t *buffer;
	buffer_t *data;
	int16_t byte;
	uint8_t i=0, ind=0, channel=0, tmp_8=0, j=0, unicast=0, unicast_writed=0, *ptr;
	uint8_t tx_power=RF_DEFAULT_POWER, compres_mode=0, echo_req=0;
	buffer = 0;
	data=0;
	channel = RF_DEFAULT_CHANNEL;
	portCHAR response;


	pause(200);
	debug_init(115200);
	pause(300);

	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 0 , NULL );	/* No Gateway discovery */

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}
	

	for( ;; )
	{
		byte = debug_read_blocking(20);
		if(byte != -1)
		{
			switch(byte)
			{
	
				case 'h':
					{
						debug("\r\nHelp:\r\n+ - Increase Tx Power\r\n- - Decrease Tx Power\r\nm - Show MAC address\r\n");
						debug("p - Send ICMP Echo req\r\nu - Send UDP Echo req \r\n");
						debug("b - Broadcast mode\r\nB - Unicast mode \r\n");
						debug("c - Show current channel \r\nC - Change channel \r\n");
						debug("0 - Set interop level 0\r\n1 - Set interop level 1\r\n");
						debug("r - Give unicast MAC address\r\n");
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
		
				case 'C':
					if(channel==26) channel=11;
					else channel++;

					mac_set_req.id = MAC_CURRENT_CHANNEL;
					mac_set_req.param.channel = channel;
					if(mac_pib_set(&mac_set_req) == MAC_SUCCESS)
					{
						debug_printf("Channel changed to %d ", channel);
					}
					else
						debug("Channel changed fail");
					debug("\r\n");
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

					case 'u':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(unicast==0)
						{
							if(udp_echo(NULL, &echo_result) == pdTRUE)
							{
								ping_start = xTaskGetTickCount();
								ping_active = 1;
								debug("udp echo_req()\r\n");
							}
							else
								debug("No buffer.\r\n");
						}
						else
						{
							if(udp_echo(&unicast_address, &echo_result) == pdTRUE)
							{
								ping_start = xTaskGetTickCount();
								ping_active = 1;
								debug("udp echo_req()\r\n");
							}
							else
								debug("No buffer.\r\n");
						}
					}

					break;

				case 'p':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(unicast==0)
								response = ping(NULL, &echo_result);
							else
								response = ping(&unicast_address, &echo_result);

						if(response == pdTRUE) /* Broadcast */
						{
							ping_start = xTaskGetTickCount();
							ping_active = 1;
							echo_req=1;
							debug("ICMP Echo req\r\n");
							if(unicast==0)
							{
								debug("Dest: Broadcast\r\n");
							}
							else
								debug("Dest: Unicast\r\n");
						}
						else
							debug("No buffer.\r\n");
					}

					break;
				case 'b':	
					unicast = 0;
					debug("6lowpan-interop Broadcast-mode\r\n");
					break;

				case 'B':
					if(unicast_writed==1)
					{
						unicast = 1;
						debug("6lowpan-interop Unicast-mode\r\n");
					}
					else
						debug("Press r & give MAC address\r\n");
					break;

				case '1':
					cipv6_compress_mode(1);
					cudp_compress_mode(1);	
					if(compres_mode !=1) compres_mode=1;
					debug("6lowpan-interop Level 1\r\n");
					break;

				case '0':
					cipv6_compress_mode(0);
					cudp_compress_mode(0);	
					if(compres_mode !=0) compres_mode=0;;
					debug("6lowpan-interop Level 0\r\n");
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

				case 'r':
					ind=2;
					j=0;
					debug("Push MAC address MSB first!\r\n");
					while(j!=8)
					{
						byte = debug_read_blocking(20);
						if(byte != -1)
						{
							if((byte >=0x30 && byte <=0x39) || (byte >=0x61 && byte <=0x66) || (byte >=0x41 && byte <=0x46))
							{
								if(byte >=0x30 && byte <=0x39)
									byte -=0x30;
								else if(byte >=0x60 && byte <=0x66)
									byte -=0x57;
								else
									byte -=0x37;

								if(ind==2)
								{
								 tmp_8 = (uint8_t) byte;
								 tmp_8 <<= 4;
								 ind--;
								}
								else
								{
									byte &= 0x0f;
									tmp_8 |= ((uint8_t) byte);
									ind=0;
									i= (7-j);
									unicast_address.address[i] = tmp_8;
									debug_hex(tmp_8);
									if(i) debug(":");
									ind=2;
									j++;
								}
							}
						}
					}
					debug("\r\n");
					unicast=1;
					unicast_writed=1;
					break;

	
				default:
					debug("\r\nNanoStack[20070710] 6lowpan-interop - Press h for help.\r\n\r\n");
					break;
			}	
		}
		if(stack_event)
		{
			buffer=0;
			buffer = waiting_stack_event(20);
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
						debug("Payload Too Length, to  ");
						debug("\r\n");
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


