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
 * \file     nrp.c
 * \brief    nRP protocol module.
 *
 *  The nRoute protocol module: handler functions.
 *   
 */


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <string.h>

#include "stack.h"
#include "address.h"
#include "buffer.h"

#include "module.h"

#include "debug.h"
#include "nrp.h"
#include "nrp_uart.h"
#include "gpio.h"
#include "mac.h"

/*start module config*/
/*
[NAME]
NRP

[ID]
MODULE_NRP,

[INFO]
#ifdef HAVE_NRP
  {nrp_init, nrp_handle, nrp_check, 0, MODULE_NRP, 0, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR nrp_init(buffer_t *buf);
extern portCHAR nrp_handle( buffer_t *buf );
extern portCHAR nrp_check( buffer_t *buf );


/*end module config*/


#ifndef NRP_SPEED
#define NRP_SPEED 115200
#endif

#ifndef NRP_FILTERS
#define NRP_FILTERS 4
#endif


int8_t nrp_init_done = -1;

void vnrp_task( void *pvParameters );

void nrp_pack_address(nrp_data_tag_t tag, sockaddr_t *sa);
void nrp_parse_address(sockaddr_t *sa, uint8_t *tag, uint8_t tag_len);

module_id_t nrp_filter_protocol[NRP_FILTERS];
sockaddr_t nrp_filter_address[NRP_FILTERS];

void nrp_filter_init(void);
portCHAR nrp_filter_match(buffer_t *buf);
int8_t nrp_filter_add(module_id_t module, sockaddr_t *sa);


buffer_t *nrp_tx[STACK_BUFFERS_MAX];
uint8_t nrp_tx_rd, nrp_tx_wr;

buffer_t *nrp_tx_pull(void)
{
	buffer_t *b;
	uint8_t tmp = nrp_tx_rd;
	
	if (tmp == nrp_tx_wr) return 0;
	b = nrp_tx[tmp];
	nrp_tx[tmp++] = 0;
	if (tmp >= STACK_BUFFERS_MAX) tmp -= STACK_BUFFERS_MAX;
	nrp_tx_rd = tmp;
	
	return b;
}

void nrp_tx_push(buffer_t *b)
{
	uint8_t tmp = nrp_tx_rd;

	if (tmp == 0) tmp = STACK_BUFFERS_MAX - 1;
	else tmp--;
		
	nrp_tx[tmp] = b;
	nrp_tx_rd = tmp;
}

void nrp_tx_add(buffer_t *b)
{
	uint8_t tmp = nrp_tx_wr;
	
	nrp_tx[tmp++] = b;

	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	nrp_tx_wr = tmp;
}

xTaskHandle nrp_task_handle;

/**
 * Initialize nRP module and UART 1.
 *
 *
 * \return SUCCESS
 * \return FAILURE	insufficient memory
 */
portCHAR nrp_init(buffer_t *buf)
{
	buf;		
	if (nrp_init_done > 0) return pdTRUE;

	nrp_filter_init();
	
	xTaskCreate( vnrp_task, "NRP", configMAXIMUM_STACK_SIZE, NULL, 
				(tskIDLE_PRIORITY + 1 ), ( xTaskHandle * ) &nrp_task_handle );

	nrp_init_done = 1;
	
	return pdTRUE;	
}


/**
 *  The nRP buffer handler.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE
 */
portCHAR nrp_handle( buffer_t *buf )
{  /* Process the packet */
	switch(buf->dir)
	{
		case BUFFER_DOWN:	
			stack_buffer_free(buf);
			break;
/*( portTickType ) 20 / portTICK_RATE_MS*/ 
			
		case BUFFER_UP:			/* From driver */
			if (nrp_filter_match(buf) == pdFALSE)
			{	/*not subscribed*/
				stack_buffer_free(buf);
			}
			else
			{
				nrp_tx_add(buf);
			}
			break;

	} 
  return pdTRUE;  
}

/**
 *  The nRP buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE 
 *  \return  pdFALSE 
 */
portCHAR nrp_check( buffer_t *buf )
{
	buffer_t *dup;
	
	switch (buf->from)
	{
#ifdef HAVE_RF_802_15_4
		case MODULE_RF_802_15_4:
#endif
#ifdef HAVE_MAC_15_4
		case MODULE_MAC_15_4:
#endif
			if (nrp_filter_match(buf) == pdTRUE)
			{
				dup = stack_buffer_get(20);
				if (dup != 0)
				{
					memcpy(dup, buf, sizeof(buffer_t) + buf->buf_end);
					dup->to = MODULE_NRP;
					stack_buffer_push(dup);
				}
			}
			break;
			
		default:
			return nrp_filter_match(buf);
	}
	return pdFALSE;  
}

extern volatile uint8_t nrp_uart_txempty;
/*#define nrp_uart_txempty uart1_txempty
extern volatile uint8_t uart1_txempty;*/

uint8_t nrp_tag_buffer[128];
nrp_type_t nrp_type;
uint16_t nrp_tag_len;
uint16_t nrp_tag_count;
uint8_t nrp_tag_id;
buffer_t *nrp_rx_buffer;

void nrp_tag_parse(void);
void nrp_transmit(buffer_t *buf);
void nrp_control(buffer_t *buf);

/**
 * NRP receiver task.
 *
 * Reads packets from serial, timeouts
 *
 * \param pvParameters not used
 */
void vnrp_task( void *pvParameters )
{
  int16_t byte;
	uint8_t nrp_rx_state;
	uint8_t idle_count = 0;
	
	pvParameters;

	vTaskDelay(200/portTICK_RATE_MS);
	
	nrp_uart_init(115200);
		
	nrp_rx_buffer = 0;
	nrp_rx_state = 0;

	vTaskDelay(100/portTICK_RATE_MS);

	debug("NRP task.\n");
			
	for (;;)
	{
	  byte = nrp_uart_get_blocking(10);
		if (byte != -1)
		{
			switch (nrp_rx_state)
			{
				case 0: /*idle*/
					if (byte == 'R') nrp_rx_state = 1;
					if (!nrp_rx_buffer) nrp_rx_buffer = stack_buffer_get(0);
					break;

				case 1:
					if (byte == 'P') nrp_rx_state = 2;
					else nrp_rx_state = 0;
					if (!nrp_rx_buffer) nrp_rx_buffer = stack_buffer_get(0);
					break;
					
				case 2:
					nrp_rx_state = 3;
					nrp_type = byte & 0x0F;
					if (!nrp_rx_buffer) nrp_rx_buffer = stack_buffer_get(0);
					
					nrp_rx_buffer->buf_ptr = 0;	
					nrp_rx_buffer->buf_end = 0;
					nrp_rx_buffer->socket = 0;
					nrp_rx_buffer->dir = BUFFER_DOWN;

					nrp_rx_buffer->src_sa.addr_type = ADDR_NONE;
					nrp_rx_buffer->dst_sa.addr_type = ADDR_NONE;
					nrp_rx_buffer->src_sa.port = 0;
					nrp_rx_buffer->dst_sa.port = 0;
					
					switch(nrp_type)
					{
						case NRP_CONFIG:
							nrp_rx_buffer->from = MODULE_APP;
							nrp_rx_buffer->to = MODULE_NRP;
							nrp_rx_buffer->options.type = BUFFER_CONTROL;
							break;

						case NRP_DATA:	/*parse data packet*/
							nrp_rx_buffer->from = MODULE_NRP;
							nrp_rx_buffer->to = MODULE_NONE;
							nrp_rx_buffer->options.type = BUFFER_DATA;
							break;
						
						default:
							nrp_rx_buffer->from = MODULE_NONE;
							nrp_rx_buffer->to = MODULE_NONE;
							nrp_rx_buffer->options.type = BUFFER_DATA;
							break;
					}
					break;

				case 3:
					nrp_tag_id = byte;
					nrp_rx_state = 4;
					break;
				
				case 4:
					nrp_tag_len = byte;
					nrp_rx_state = 5;
					break;
					
				case 5:
					nrp_tag_len <<= 8;
					nrp_tag_len += byte;
					nrp_tag_count = 0;
					if (nrp_tag_len)	nrp_rx_state = 6;
					else goto parse_tag;
					break;
				
				case 6:
					nrp_tag_buffer[nrp_tag_count++] = byte;
parse_tag:
					if (nrp_tag_count >= nrp_tag_len )
					{
						debug("P");
						nrp_tag_parse();
						if (nrp_tag_id & 0x80)
						{	/*packet complete*/
							if (nrp_rx_buffer->options.type == BUFFER_CONTROL)
							{	/*add control to tx queue*/
								debug("Q");
								nrp_tx_add(nrp_rx_buffer);
							}
							else
							{	/*push buffer to module*/
								debug("M");
								if (nrp_rx_buffer->to == MODULE_CIPV6)
								{
									uint8_t *dptr;
									
									stack_buffer_headroom(nrp_rx_buffer, 4);
									
									nrp_rx_buffer->buf_ptr -= 4;
									
									dptr = nrp_rx_buffer->buf + nrp_rx_buffer->buf_ptr;
									nrp_rx_buffer->options.hop_count = 5;
									nrp_rx_buffer->to 	= 	MODULE_CIPV6; 
									nrp_rx_buffer->from 	= 	MODULE_ICMP;
									/*ICMP type and code*/
									*dptr++ = (nrp_rx_buffer->dst_sa.port >> 8);
									*dptr++ = (nrp_rx_buffer->dst_sa.port);
									/*FCS*/
									*dptr++ = 0x00;
									*dptr++ = 0x00;
								}
								stack_buffer_push(nrp_rx_buffer);
							}
							nrp_rx_buffer = 0;
							nrp_rx_state = 0;
						}
						else nrp_rx_state = 3;
					}
					break;
			}/*end receive switch*/
			idle_count = 0;
		}	/*end data in*/
		else
		{	/*no data received*/
			if (idle_count++ > 20)
			{
				idle_count = 0;
				if (nrp_rx_state != 0)
				{
					debug_put('|');
				}
				nrp_rx_state = 0;	/*time out receiver*/
				nrp_uart_rx_reset();
			}
		}
		if (nrp_uart_txempty)
		{	/*uart transmit complete*/
			buffer_t *buf_out = nrp_tx_pull();
			if (buf_out)
			{	/*have stuff to take care of*/
				if (buf_out->options.type == BUFFER_CONTROL)
				{
					debug("control");
					nrp_control(buf_out);
				}
				else if (buf_out->from != MODULE_NONE)
				{
					debug("data");
					nrp_transmit(buf_out);
				}
				stack_buffer_free(buf_out);
			}
		}
		if ((nrp_rx_state != 0) || (nrp_uart_txempty == 0))
		{
			LED1_ON();
		}
		else
		{
			LED1_OFF();
		}
	}	/*end task loop */
}	/* end task main function */



/**
 * Parse nRP packets.
 *
 *
 */
void nrp_tag_parse(void)
{
	nrp_config_tag_t conf_tag;
	nrp_data_tag_t tag;

	nrp_tag_count = 0;
	
	if ((nrp_type == NRP_CONFIG) && (nrp_rx_buffer->from == MODULE_APP))
	{	/*get config tag*/
		uint8_t *ptr = nrp_rx_buffer->buf;
		nrp_rx_buffer->buf_ptr = 0;
		nrp_rx_buffer->buf_end = nrp_tag_len + 1;
		
		debug("Config");
		nrp_rx_buffer->from = MODULE_NRP;	/*mark buffer so that config tag is parsed*/
		*ptr++ = nrp_tag_id & 0x7F;
		conf_tag = nrp_tag_id & 0x7F;
		switch (conf_tag)
		{
			case NRPC_SUBSCRIBE:
			case NRPC_RF_ID:
			case NRPC_PROTOCOL_ID:
			case NRPC_RESET:
			case NRPC_CHANNEL_SET:
			case NRPC_MAC_GET:
			case NRPC_VERSION:
				memcpy(ptr, nrp_tag_buffer, (nrp_tag_len & 0x7F));	/*copy tag data to buffer*/
				debug("Cpy");
				break;

			default:	/*unknown config, ignore*/
				nrp_rx_buffer->options.type = BUFFER_DATA;	/*mark buffer so that it is discarded*/
				nrp_rx_buffer->from = MODULE_NONE;	/*mark buffer so that it is discarded*/
				break;
		}
		return;
	}
	debug("Data");
	tag = nrp_tag_id & 0x7F;
	switch (tag)
	{
		case NRP_PDATA:
			{
				uint8_t *ptr = nrp_rx_buffer->buf;
				ptr += nrp_rx_buffer->buf_end;
				memcpy(ptr, nrp_tag_buffer, nrp_tag_len);
				nrp_rx_buffer->buf_end += nrp_tag_len;
			}
			break;
						
		case NRP_SRC:
			if (nrp_tag_len != 0)
			{
				nrp_parse_address(&(nrp_rx_buffer->src_sa), nrp_tag_buffer, nrp_tag_len);
			}
			else
			{
				nrp_rx_buffer->src_sa.addr_type = ADDR_NONE;
			}
			break;
						
		case NRP_DST:
			if (nrp_tag_len != 0)
			{
				nrp_parse_address(&(nrp_rx_buffer->dst_sa), nrp_tag_buffer, nrp_tag_len);
			}
			else
			{
				nrp_rx_buffer->dst_sa.addr_type = ADDR_NONE;
			}
			break;
						
		case NRP_PROTOCOL:
			if ((nrp_tag_len == 1) && (nrp_rx_buffer->options.type == BUFFER_CONTROL))
			{
				buffer_push_uint8(nrp_rx_buffer, NRP_PROTOCOL);
				buffer_push_uint8(nrp_rx_buffer, nrp_tag_buffer[0]);
			}
			else if (nrp_tag_len == 1)
			{
				nrp_protocol_t proto;
				proto = nrp_tag_buffer[0];
				switch(proto)
				{
					case NRP_PROTO_802_15_4:
#ifdef HAVE_802_15_4_RAW
						nrp_rx_buffer->to = MODULE_802_15_4_RAW;
#else
						nrp_rx_buffer->to = MODULE_RF_802_15_4;
#endif
#ifdef HAVE_MAC_15_4
						nrp_rx_buffer->to = MODULE_MAC_15_4;
#endif
						break;
#ifdef HAVE_NUDP
					case NRP_PROTO_NUDP:
						nrp_rx_buffer->to = MODULE_NUDP;
						break;
#endif
#ifdef HAVE_CUDP
					/*todo: move header headroom handling to 6lowpan modules*/
					case NRP_PROTO_6LOWPAN:
						nrp_rx_buffer->to = MODULE_CUDP;
						break;
#endif
#ifdef HAVE_ICMP
					case NRP_PROTO_6LOWPAN_ICMP:
						nrp_rx_buffer->to = MODULE_CIPV6;
						break;
#endif
									
#ifdef HAVE_ZIGBEE
					case NRP_PROTO_ZIGBEE:
						nrp_rx_buffer->to = MODULE_ZIGBEE;
						break;
#endif
					default:
						nrp_rx_buffer->to = MODULE_NONE;
						break;
				}
			}
			break;
						
		case NRP_SRC_PORT:
			if (nrp_tag_len == 2)
			{
				nrp_rx_buffer->src_sa.port = nrp_tag_buffer[0];
				nrp_rx_buffer->src_sa.port <<= 8;
				nrp_rx_buffer->src_sa.port += nrp_tag_buffer[1];
			}
			break;
						
		case NRP_DST_PORT:
			if (nrp_tag_len == 2)
			{
				nrp_rx_buffer->dst_sa.port = nrp_tag_buffer[0];
				nrp_rx_buffer->dst_sa.port <<= 8;
				nrp_rx_buffer->dst_sa.port += nrp_tag_buffer[1];
			}
			break;
						
		default:
			break;
	}	
}

void nrp_control(buffer_t *buf)
{
	nrp_config_tag_t msg_type;
	nrp_type_t type = NRP_CONFIG_REPLY;
	uint8_t tag_len = 0;
	
	debug("Hdr");
	
	nrp_uart_put('N');
	nrp_uart_put('R');
	nrp_uart_put('P');
	nrp_uart_put(NRP_VERSION + type);
	
	msg_type = buffer_pull_uint8(buf);
	switch(msg_type & 0x7F)
	{
		case NRPC_VERSION:
			debug("Version");
			nrp_uart_put((nrp_config_tag_t) NRPC_VERSION | 0x80);	/*tag*/
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(NRP_VERSION);			
			break;
		case NRPC_RF_ID:
			debug("ID");
			nrp_uart_put((nrp_config_tag_t)NRPC_RF_ID | 0x80);	/*tag*/
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(0x01);			
			break;
		case NRPC_PROTOCOL_ID:
			debug("Proto");
			nrp_uart_put((nrp_config_tag_t)NRPC_PROTOCOL_ID | 0x80);	/*tag*/
			tag_len = 1;
#ifdef HAVE_NUDP
			tag_len++;
#endif
#ifdef HAVE_CUDP
			tag_len++;
#endif
#ifdef HAVE_ZIGBEE
			tag_len++;
#endif
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(tag_len);	/*length*/
			nrp_uart_put(NRP_PROTO_802_15_4);
#ifdef HAVE_NUDP
			nrp_uart_put(NRP_PROTO_NUDP);
#endif
#ifdef HAVE_CUDP
			nrp_uart_put(NRP_PROTO_6LOWPAN);
#endif
#ifdef HAVE_ZIGBEE
			nrp_uart_put(NRP_PROTO_ZIGBEE);
#endif
			break;
			
		case NRPC_RESET:
			debug("Reset");
			nrp_filter_init();
			rf_channel_set(RF_DEFAULT_CHANNEL);
			nrp_uart_put((nrp_config_tag_t)NRPC_RESET | 0x80);	/*tag*/
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
#ifdef NRP_DEFAULT_MAC
			nrp_uart_put(1);
#else
#ifdef NRP_DEFAULT_NUDP
			nrp_uart_put(2);
#else
			nrp_uart_put(0);
#endif
#endif
			break;
		
		case NRPC_SUBSCRIBE:
			debug("Sub");
			{
				nrp_protocol_t protocol = buffer_pull_uint8(buf);
				module_id_t module = MODULE_NONE;
				protocol = buffer_pull_uint8(buf);
				
				switch(protocol)
				{
					case NRP_PROTO_802_15_4:
#ifdef HAVE_802_15_4_RAW
						module = MODULE_802_15_4_RAW;
#else
						module = MODULE_RF_802_15_4;
#endif
#ifdef HAVE_MAC_15_4
						module = MODULE_MAC_15_4;
#endif
						break;
#ifdef HAVE_NUDP
					case NRP_PROTO_NUDP:
						module = MODULE_NUDP;
						break;
#endif					
#ifdef HAVE_CUDP
					case NRP_PROTO_6LOWPAN:
						module = MODULE_CUDP;
						break;
#endif					
#ifdef HAVE_ZIGBEE
					case NRP_PROTO_ZIGBEE:
						module = MODULE_ZIGBEE;
						break;
#endif
					default:
						break;
				}
				nrp_uart_put((nrp_config_tag_t)NRPC_SUBSCRIBE | 0x80);	/*tag*/
				nrp_uart_put(0);	/*set*/
				nrp_uart_put(1);	/*length*/
				if (module != MODULE_NONE)
				{
					tag_len = nrp_filter_add(module, &(buf->dst_sa));
					nrp_uart_put(tag_len);
				}
				else
				{
					nrp_uart_put(-1);
				}
			}
			break;

		case NRPC_CHANNEL_SET:
			debug("Chn");
			{
				uint8_t channel;
				
				channel = buffer_pull_uint8(buf);
				
				if ((channel >= 11) && (channel <= 25))
				{
					mac_set_channel(channel);
				}
				nrp_uart_put((nrp_config_tag_t)NRPC_CHANNEL_SET | 0x80);	/*tag*/
				nrp_uart_put(0);	/*set*/
				nrp_uart_put(1);	/*length*/
				nrp_uart_put(mac_current_channel());
			}
			break;
						
		case NRPC_MAC_GET:
			debug("MAC");
			{
				sockaddr_t tmp;
				uint8_t i;
				
				tmp.addr_type = ADDR_NONE;
				mac_get(&tmp);
				
				nrp_uart_put((nrp_config_tag_t)NRPC_MAC_GET | 0x80);	/*tag*/
				nrp_uart_put(0);	/*set*/
				nrp_uart_put(8);	/*length*/
				for (i=0; i<8; i++)
				{
					nrp_uart_put(tmp.address[7-i]);
				}
			}
			break;
						
		default:
			debug("Eject");
			debug_hex(msg_type);
			break;
	}
	nrp_uart_launch();
}


void nrp_transmit(buffer_t *buf)
{
	uint16_t tmp_16;
	nrp_type_t type = NRP_DATA;
	nrp_data_tag_t tag = NRP_PROTOCOL;
	uint8_t *ptr;
	
	nrp_uart_put('N');
	nrp_uart_put('R');
	nrp_uart_put('P');
	nrp_uart_put(NRP_VERSION + type);

	switch (buf->from)
	{
#ifdef HAVE_802_15_4_RAW
		case MODULE_802_15_4_RAW:
#else
		case MODULE_RF_802_15_4:
#endif
		case MODULE_MAC_15_4:
			nrp_uart_put(tag);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(NRP_PROTO_802_15_4);	/*Raw MAC data*/
			break;

		case MODULE_NUDP:
			nrp_uart_put(tag);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(NRP_PROTO_NUDP);	/*NUDP data*/
			nrp_uart_put(NRP_SRC_PORT);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(2);	/*length*/
			nrp_uart_put(buf->src_sa.port >> 8);	/*NUDP port*/
			nrp_uart_put(buf->src_sa.port);	/*NUDP port*/
			nrp_uart_put(NRP_DST_PORT);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(2);	/*length*/
			nrp_uart_put(buf->dst_sa.port >> 8);	/*NUDP port*/
			nrp_uart_put(buf->dst_sa.port);	/*NUDP port*/
			break;

#ifdef HAVE_CUDP
#ifdef HAVE_ICMP
		case MODULE_ICMP:
			nrp_uart_put(tag);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(NRP_PROTO_6LOWPAN_ICMP);	/*data*/
			nrp_uart_put(NRP_DST_PORT);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(2);	/*length*/
			ptr = buf->buf;
			ptr += buf->buf_ptr;	/*set index to start of data*/
			nrp_uart_put(*ptr++);	/*type*/
			nrp_uart_put(*ptr);				/*code*/
			buf->buf_ptr += 4;
			break;		
#endif
		case MODULE_CUDP:
			nrp_uart_put(tag);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(1);	/*length*/
			nrp_uart_put(NRP_PROTO_6LOWPAN);	/*data*/
			nrp_uart_put(NRP_SRC_PORT);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(2);	/*length*/
			nrp_uart_put(buf->src_sa.port >> 8);	/*port*/
			nrp_uart_put(buf->src_sa.port);				/*port*/
			nrp_uart_put(NRP_DST_PORT);
			nrp_uart_put(0);	/*set*/
			nrp_uart_put(2);	/*length*/
			nrp_uart_put(buf->dst_sa.port >> 8);	/*port*/
			nrp_uart_put(buf->dst_sa.port);				/*port*/
			break;
#endif

		default:	/*not handled*/
			break;
	}
	nrp_pack_address(NRP_SRC, &(buf->src_sa));
	nrp_pack_address(NRP_DST, &(buf->dst_sa));

	ptr = buf->buf;
	ptr += buf->buf_ptr;	/*set index to start of data*/
	
	tmp_16 = buf->buf_end - buf->buf_ptr; /*length of data*/
	tag = NRP_PDATA;
	nrp_uart_put(tag);
	nrp_uart_put((tmp_16 >> 8));	/*push length*/
	nrp_uart_put(tmp_16);
	while (tmp_16)
	{
		nrp_uart_put(*ptr++);
		tmp_16--;
	}
	tmp_16 = (uint16_t) buf->options.rf_dbm;
	tag = NRP_DBM;
	nrp_uart_put(tag + 0x80);
	nrp_uart_put(0);	/*set*/
	nrp_uart_put(2);	/*length*/
	nrp_uart_put((tmp_16 >> 8));	/*push dbm value*/
	nrp_uart_put(tmp_16);

	nrp_uart_launch();
	return;
}













/*
 * Filter section
 */
void nrp_filter_init(void)
{
	uint8_t i;
	for (i=0; i< NRP_FILTERS; i++)
	{
		nrp_filter_protocol[i] = MODULE_NONE;
		nrp_filter_address[i].addr_type = ADDR_NONE;
		nrp_filter_address[i].port = 0;
	}
#ifdef NRP_DEFAULT_NUDP
	{
		sockaddr_t piip_addr;
		
		piip_addr.port = 0;
		piip_addr.addr_type = ADDR_NONE;
		
		nrp_filter_add(MODULE_NUDP, &piip_addr);
	}
#endif
#ifdef NRP_DEFAULT_MAC
	{
		sockaddr_t piip_addr;
		
		piip_addr.port = 0;
		piip_addr.addr_type = ADDR_NONE;
		
#ifdef HAVE_802_15_4_RAW
		nrp_filter_add(MODULE_802_15_4_RAW, &piip_addr);
#else
		nrp_filter_add(MODULE_RF_802_15_4, &piip_addr);
#endif								
	}
#endif
}

portCHAR nrp_filter_match(buffer_t *buf)
{
	uint8_t i;
	
	for (i=0; i< NRP_FILTERS; i++)
	{
		
		uint8_t match = 0;
		
		if (nrp_filter_protocol[i] != MODULE_NONE)
		{
			
			if ( (nrp_filter_protocol[i] == buf->from) || ((nrp_filter_protocol[i] == MODULE_CUDP) && (buf->from == MODULE_ICMP)) )
			{	/*protocol match*/

				
				if (nrp_filter_address[i].addr_type != ADDR_NONE)
				{
					if (nrp_filter_address[i].addr_type == buf->dst_sa.addr_type)
					{
						if (memcmp(nrp_filter_address[i].address, buf->dst_sa.address, 8) == 0)
						{
							match = 1;
						}
					}
				}
				else
				{
					 match = 1; 	/*no address in filter, match*/
				}
				if (nrp_filter_address[i].port != 0)
				{	
					if (nrp_filter_address[i].port == buf->dst_sa.port)
					{
						match++;
					}
				}
				else
				{
					match++;			/*no port in filter, match*/
					
				}
				if (match > 1)
				{
					return pdTRUE;
				}
			}
		}
	}
	return pdFALSE;
}

int8_t nrp_filter_add(module_id_t module, sockaddr_t *sa)
{
	int8_t filter_id = -1;
	uint8_t i;
		
	for (i=0; i< NRP_FILTERS; i++)
	{
		if (nrp_filter_protocol[i] == module)
		{
			if (nrp_filter_address[i].addr_type == sa->addr_type)
			{
				if (nrp_filter_address[i].port == sa->port)
				{
					return -1;
				}
			}
		}
	}
	
	for (i=0; i< NRP_FILTERS; i++)
	{
		if (nrp_filter_protocol[i] == MODULE_NONE)
		{
			nrp_filter_protocol[i] = module;
			nrp_filter_address[i].addr_type = sa->addr_type;
			memcpy(nrp_filter_address[i].address, sa->address, sizeof(nrp_filter_address[i].address));
			nrp_filter_address[i].port = sa->port;
			filter_id = i;
			return i;
		}
	}
	
	return filter_id;
}

void nrp_parse_address(sockaddr_t *sa, uint8_t *tag, uint8_t tag_len)
{
	uint8_t len;

debug("nrp_parse_addr\r\n");

	switch (*tag)
	{
		case NRP_802_15_4_LONG:
			len = 8;
			if (sa->addr_type == ADDR_PAN)
			{
				sa->addr_type = ADDR_802_15_4_PAN_LONG;
			}
			else
			{
				sa->addr_type = ADDR_802_15_4_PAN_LONG;
				sa->address[8] = 0xFF;
				sa->address[9] = 0xFF;
			}
			tag++;
			break;
			
		case NRP_802_15_4_SHORT:
			len = 2;
			if (sa->addr_type == ADDR_PAN)
			{
				sa->addr_type = ADDR_802_15_4_PAN_SHORT;
				sa->address[3] = sa->address[9];
				sa->address[2] = sa->address[8];
			}
			else
			{
				sa->addr_type = ADDR_802_15_4_PAN_SHORT;
				sa->address[2] = 0xFF;
				sa->address[3] = 0xFF;
			}
			tag++;
			break;
			
		case NRP_802_15_4_PAN:
			tag++;
			sa->address[9] = *tag++;
			sa->address[8] = *tag++;
			if (sa->addr_type == ADDR_802_15_4_PAN_SHORT)
			{
				sa->address[3] = sa->address[9];
				sa->address[2] = sa->address[8];
			}
			else if (sa->addr_type == ADDR_802_15_4_PAN_LONG)
			{
			}
			else
			{
				sa->addr_type = ADDR_PAN;
				sa->address[1] = sa->address[9];
				sa->address[0] = sa->address[8];
			}
			return;
			
		default:
			sa->addr_type = *tag++;
			memcpy(sa->address, tag, tag_len-1);
			return;
	}
	while(len)
	{
		len--;
		sa->address[len] = *tag++;
	}
}

void	nrp_pack_address(nrp_data_tag_t tag, sockaddr_t *sa)
{
	uint8_t i;

debug("nrp_pack_addr\r\n");
	
	switch(sa->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
			if ((sa->address[8] != 0xFF) || (sa->address[9] != 0xFF))
			{
				nrp_uart_put(tag);
				nrp_uart_put(0);
				nrp_uart_put(3);
				nrp_uart_put(NRP_802_15_4_PAN);
				nrp_uart_put(sa->address[9]);
				nrp_uart_put(sa->address[8]);
			}
			/*break missing intentionally*/
		case ADDR_802_15_4_LONG:
			nrp_uart_put(tag);
			nrp_uart_put(0);
			nrp_uart_put(9);
			nrp_uart_put(NRP_802_15_4_LONG);
			for (i=0; i < 8; i++)
			{
				nrp_uart_put(sa->address[7-i]);
			}
			break;
			
		case ADDR_BROADCAST:
			for (i=0; i<3; i++)
			{
				sa->address[i] = 0xFF;
			}
			/*break missing intentionally*/
		case ADDR_802_15_4_PAN_SHORT:
			if ((sa->address[2] != 0xFF) || (sa->address[3] != 0xFF))
			{
				nrp_uart_put(tag);
				nrp_uart_put(0);
				nrp_uart_put(3);
				nrp_uart_put(NRP_802_15_4_PAN);
				nrp_uart_put(sa->address[3]);
				nrp_uart_put(sa->address[2]);
			}
			/*break missing intentionally*/
		case ADDR_802_15_4_SHORT:
			nrp_uart_put(tag);
			nrp_uart_put(0);
			nrp_uart_put(3);
			nrp_uart_put(NRP_802_15_4_SHORT);
			nrp_uart_put(sa->address[1]);
			nrp_uart_put(sa->address[0]);
			break;
			
		default:
			break;
	}
}
