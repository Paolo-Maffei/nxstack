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
 * \brief Example for testing beacon enable mode Coordinator device. Coordinator handle client Assocation request and buffer their sended data to own data buffer. When coordinator has discovered gateway it send data buffer to gateway when buffer is full. 
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

static void vmicro_beacon_server( void *pvParameters );
#include "control_message.h"
#include "neighbor_routing_table.h"
#include "socket.h"
#include "rf.h"


#define DATA_BUFFER_LENGTH 59
extern xQueueHandle     buffers;
extern sockaddr_t mac_long;

sockaddr_t datasensor_address = 
{
	ADDR_802_15_4_PAN_SHORT,
	{ 0x00, 0x00, 0x00, 0x00},
	61619
};


sockaddr_t test_adr = 
{
	ADDR_802_15_4_PAN_SHORT,
	{ 0x45, 0x45, 0x45, 0x45},
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
	xTaskCreate( vmicro_beacon_server, "cord", 250, NULL, ( tskIDLE_PRIORITY + 2 ), NULL );	
	vTaskStartScheduler();
	return 0;
}
/*-----------------------------------------------------------*/
typedef uint8_t b1w_reg[8];

uint8_t data_buffer[DATA_BUFFER_LENGTH], data_pac_index=0;
uint8_t data_proxy[DATA_BUFFER_LENGTH], data_proxy_index, data_proxy_send=0;

gateway_cache_t gateway_info;
socket_t *data_soc=0;
stack_event_t stack_event; 
int8_t ping_active=0;
portTickType ping_start;
mac_param_t mac_set_req;

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
static void vmicro_beacon_server( void *pvParameters )
{
	int16_t byte;
	uint16_t tx_sqn=0, channel=PAN_CHANNEL;
	uint8_t i=0, j, *ptr;
	uint8_t length, ind, rx_router_adv=0;
	gateway_info.count=0;
	uint8_t addres_length=0;
	portTickType xLastWakeTime=0;
	int8_t discover_active=0;
	portTickType discover_start=0;
	buffer_t *buffer;
	buffer_t *data;

	buffer = 0;
	data=0;
	pause(200);
	debug_init(115200);
	pause(300);

	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 1 , &gateway_info );

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}
/*****************************************************************************************************/

/****************************************************************************************************/


	debug("Coordinator device 1.0\r\n\r\n");
	
	
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{	
		if ((xTaskGetTickCount() - xLastWakeTime)*portTICK_RATE_MS > 10000)
		{
			if(update_gw_info_ttl() != pdTRUE)
			{
				debug(" Got not semaphore.\r\n");
			}
			xLastWakeTime = xTaskGetTickCount();
		}

		byte = debug_read_blocking(10);
		if(byte != -1)
		{
			switch(byte)
			{
				case 'b':
					debug_int(uxQueueMessagesWaiting(buffers));
					debug(" buffers available.\r\n");
					break;	
	
				case 'g':
					if(discover_active==0)
					{
						if(gw_discover() == pdTRUE)
						{
							discover_active=1;
							rx_router_adv=0;
						}
						else
							debug("No buffer\r\n");
					}
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


				case 'P':
					{
						buffer_t *buf = stack_buffer_get(20);
						if (buf)
						{
							buf->buf_end=0;
							buf->buf_ptr=0;
							buf->dst_sa.addr_type = ADDR_NONE;
							buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
							buf->dst_sa.address[0]=0x11;
							buf->dst_sa.address[1]=0x11;
							buf->buf[buf->buf_end++] = 0x03;
							buf->buf[buf->buf_end++] = 0x70;
							if (mac_pending_req(buf) != pdTRUE)
							{
								debug("Pend_fail.\r\n");
								socket_buffer_free(buf);
								buf=0;
							}
						}
					}
					break;
	
				case '\r':
					
					debug("\r\n");
					break;

				case 't':
					print_table_information();
					if(gateway_info.count)
					{
						/*Check first cache*/
						for(i=0; i<gateway_info.count;i++)
						{
							debug("\r\nGW addr:");
							if(gateway_info.gateway_info[i].address_type==ADDR_802_15_4_PAN_LONG)
								addres_length=8;
							else
								addres_length=2;
							for(j=0; j < addres_length ; j++)
							{
								if (j) debug_put(':');
								debug_hex(gateway_info.gateway_info[i].address[j]);
							}
							debug("\r\nHops:");
							debug_hex(gateway_info.gateway_info[i].hop_distance);
							debug("\r\nLQI:");
							debug_hex(gateway_info.gateway_info[i].lqi);
							debug("\r\nTTL:");
							debug_hex(gateway_info.gateway_info[i].ttl);
							debug("\r\n");
						}
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
			data = socket_read(data_soc, 10);
			if (data)
			{
				ind=data->buf_ptr;
				if (data->dst_sa.port == 61619)
				{
					length = data->buf_end - data->buf_ptr;
					length = data->buf_end - data->buf_ptr;
					if(length > 1)
					{
						if(datasensor_address.addr_type != ADDR_NONE)
						{
							uint8_t temp;
							temp= (data_pac_index+length);
							temp +=2; /*css address 2 bytes */
							if(temp < DATA_BUFFER_LENGTH)
							{
								data_buffer[data_pac_index++] = data->src_sa.address[0];
								data_buffer[data_pac_index++] = data->src_sa.address[1];
								for(i=0; i<length; i++)
								{
									data_buffer[data_pac_index++] = data->buf[ind++];
								}
							}
							else
							{
								buffer_t *buf = socket_buffer_get(data_soc);
								if(buf)
								{
									ind=buf->buf_end;
									buf->buf[ind++] = tx_sqn;
									data_proxy[0] = tx_sqn;
									data_proxy_index = data_pac_index;
									for(i=0; i< data_pac_index; i++)
									{
										buf->buf[ind++]=data_buffer[i];
										data_proxy[1+i]=data_buffer[i];
									}
									buf->buf_end=ind;
									data_pac_index=0;
									datasensor_address.port=61620;
									if (socket_sendto(data_soc, &datasensor_address, buf) == pdTRUE)
									{
										debug("Sensor data sended ,");	
										debug_int(tx_sqn);
										debug(" sqn\r\n");
										tx_sqn++;
									}
									else
									{
										socket_buffer_free(buf);
										buf = 0;
									}
								}
								ind=data->buf_ptr;
								data_buffer[data_pac_index++] = data->src_sa.address[0];
								data_buffer[data_pac_index++] = data->src_sa.address[1];
								for(i=0; i<length; i++)
								{
									data_buffer[data_pac_index++] = data->buf[ind++];
								}
							}
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
			buffer = waiting_stack_event(10);
			if(buffer)
			{
				switch (parse_event_message(buffer))
				{
					case BROKEN_LINK:
						debug("Route broken to ");
						if(remove_gw_info(&(buffer->dst_sa)) == pdTRUE)
						{
							buffer->dst_sa.port = datasensor_address.port;
							debug("GW\r\n");
							if(memcmp(&datasensor_address, &(buffer->dst_sa), sizeof(sockaddr_t)) == 0)
							{
								debug("kylla\r\n");
								datasensor_address.addr_type = ADDR_NONE;
							}
						}
						else
						{
							debug("\r\n");
							debug_address(&(buffer->dst_sa));
							debug("\r\n");
						}
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

					case MAC_CCA_ERR:
						debug("TX err:CCA\r\n");
						break;
			

					case COORDINATOR_STARTED:
							data_soc 			= socket(MODULE_CUDP, 		0);	/* Socket for response data from port number 45 */
							if (data_soc)
							{
								if (socket_bind(data_soc, &datasensor_address) != pdTRUE)
								{
									debug("Bind fail rx.\r\n");
								}
							}
							datasensor_address.addr_type = ADDR_NONE;
							break;
				
					case ROUTER_DISCOVER_RESPONSE:
						rx_router_adv=1;
						select_best_gw(&datasensor_address);
						if(data_proxy_send)
						{
							buffer->socket = 0;
							buffer->buf_end = 0;
							buffer->buf_ptr = 0;
							buffer->options.type = BUFFER_DATA;
							ind=buffer->buf_end;
							for(i=0; i< data_proxy_index; i++)
							{
								buffer->buf[ind++]=data_proxy[i];
							}
							buffer->buf_end=ind;
							data_proxy_index=0;
							datasensor_address.port=61620;
							if (socket_sendto(data_soc, &datasensor_address, buffer) == pdTRUE)
							{	
								debug_int(data_proxy[0]);
								debug(" re-tx sqn\r\n");
								buffer=0;
								data_proxy_send=0;
							}
						}
						break;

					case PENDING_TIMEOUT:
						debug("Data not loaded from ");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						update_neighbour_table(buffer->dst_sa.addr_type, buffer->dst_sa.address, 0, 0, REMOVE_NEIGHBOUR);
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
		if ((xTaskGetTickCount() - discover_start)*portTICK_RATE_MS > 1000 && discover_active)
		{
			if(rx_router_adv)
			{
				debug("ok.\r\n");
				rx_router_adv=0;
			}
			else
				rx_router_adv=0;

			discover_active = 0;
		}
		if (ping_active && ((xTaskGetTickCount() - ping_start)*portTICK_RATE_MS) > 500 )
		{
			stop_ping();
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
				debug("No response.\r\n");
		}
	}
}

