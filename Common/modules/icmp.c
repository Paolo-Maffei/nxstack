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
 * \file     icmp.c
 * \brief    ICMP protocol module.
 *
 *  Module includes Echo Req/Response... .
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#ifndef ICMP_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"
#include "socket.h"
#include "control_message.h"
#include "neighbor_routing_table.h"
#include "cipv6.h"

/*
[NAME]
ICMP

[ID]
MODULE_ICMP,

[INFO]
#ifdef HAVE_ICMP
  {icmp_init, icmp_handle, icmp_check, 0, MODULE_ICMP, 16, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR icmp_init(buffer_t *buf);
extern portCHAR icmp_handle( buffer_t *buf );
extern portCHAR icmp_check( buffer_t *buf );
uint8_t gateway_features;


/**
 *  Initialize ICMP module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR icmp_init( buffer_t *buf )
{
	buf;
	gateway_features=0;
	return pdTRUE;
}

void add_fcf(buffer_t *buf )
{
#ifndef NO_FCS
	uint8_t *dptr;
	uint16_t fcs=0;		/* Calculate FCS */
	/* IP pseudo header */
	fcs = ipv6_fcf(buf, NEXT_HEADER_ICMP6, 0);
	/* ICMP header */
	dptr = buf->buf + (buf->buf_ptr + 2);
	*dptr++ = (fcs >> 8);
	*dptr++ = (uint8_t)fcs;
#endif /*NO_FCS*/
		
	buf->src_sa.addr_type = ADDR_NONE;
	buf->from = MODULE_ICMP;
	buf->to = MODULE_NONE;
	buf->dir = BUFFER_DOWN;
	buf->options.type = BUFFER_DATA;
	buf->socket=0;
}

uint16_t icmp_temp_16;
/**
 *  Main ICMP buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR icmp_handle( buffer_t *buf )
{
	
	
	uint8_t *dptr;
	control_message_t *ptr;
	uint8_t icmp_type=0, icmp_code=0;
#ifdef HAVE_ROUTING
	uint8_t i=0;
#endif	
	//debug("ICMP: handler.\r\n");

	if(buf->options.type == BUFFER_CONTROL && buf)		/* Control message received */
	{	/* Control-message */
		
		ptr = ( control_message_t*) buf->buf;
		switch(ptr->message.ip_control.message_id)
		{
#ifdef HAVE_ROUTING
			case BROKEN_LINK:
				debug("Control: Broken link mes create & send\r\n");
				if(ptr->message.ip_control.message.broken_link_detect.reason == BROKEN_LINK_TYPE)
					i = ERROR_CODE_BROKEN_LINK;
				if(ptr->message.ip_control.message.broken_link_detect.reason == NO_ROUTE_TO_DESTINATION_TYPE)
					i = ERROR_CODE_NO_ROUTE;

				memset(buf->buf, 0, 24);
				buf->buf_ptr = 0;
				buf->buf_end = 24;
				dptr = buf->buf;
				*dptr++ = ICMP_ERROR_MESSAGE_TYPE;
				*dptr++ = i;
				dptr += 6;
				/* Error packet destination address */
				*dptr++ = 0xfe;
				*dptr++ = 0x80;
				dptr += 6;
				if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
				{
					for(i=0;i<8;i++)
					{
						*dptr++ = buf->dst_sa.address[7-i];
					}
				}
				else
				{
					dptr += 3;
					*dptr++ = 0xff;
					*dptr++ = 0xfe;
					dptr += 1;
					*dptr++ = buf->dst_sa.address[1];
					*dptr++ = buf->dst_sa.address[0];
				}

				buf->dst_sa.addr_type = buf->src_sa.addr_type;
				buf->src_sa.addr_type = ADDR_NONE;
				memcpy(buf->dst_sa.address, buf->src_sa.address, 8);
				break;
#endif
			default:
				stack_buffer_free(buf);
				buf=0;
				break;
		}
	}
	else
	{
		switch(buf->dir)
		{
			case BUFFER_DOWN:
				stack_buffer_free(buf);
				buf=0;
				break;
	
			case BUFFER_UP:

				dptr = (buf->buf + buf->buf_ptr) +2;
				icmp_temp_16 = *dptr++;
				icmp_temp_16 += ((uint16_t) (*dptr++) << 8);
#ifndef NO_FCS
				/* Check FCS first */
				if(icmp_temp_16 && (ipv6_fcf(buf, NEXT_HEADER_ICMP6, 1) != 0xffff))
				{
					debug("ICMP: FCS fail\r\n");
					stack_buffer_free(buf);
					buf=0;
				}
				else
#endif
#ifdef HAVE_NRP
				{
					buf->from = MODULE_ICMP;
					buf->to = MODULE_NRP;
					stack_buffer_push(buf);
						
					return pdTRUE;
				}
#else
				{
					dptr = (buf->buf + buf->buf_ptr);	
					icmp_type = *dptr++;
					icmp_code = *dptr++;

				}
				
				if(icmp_type != ICMP_ERROR_MESSAGE_TYPE && icmp_code != 0)
				{
					stack_buffer_free(buf);
					buf=0;
					return pdTRUE;
				}

				if(buf && icmp_type==ECHO_RESPONSE_TYPE)
				{
					dptr += 4;
					buf->buf_ptr = (dptr - buf->buf);
					buf->from = MODULE_ICMP;
					parse_echo_response(buf);
					buf=0;
				}
				else if(buf && icmp_type==ECHO_REQUEST_TYPE)
				{
#ifndef HAVE_NRP
					/* Copy source -> Destination field */
					buf->dst_sa.addr_type = buf->src_sa.addr_type;
					buf->src_sa.addr_type = ADDR_NONE;

					memcpy(buf->dst_sa.address, buf->src_sa.address, 8);
					dptr = buf->buf + buf->buf_ptr;
					*dptr++ = ECHO_RESPONSE_TYPE;
					*dptr++ = ICMP_CODE;
					*dptr++ = 0x00;
					*dptr++ = 0x00;
#else
					ptr = ( control_message_t*) buf->buf;
					ptr->message.ip_control.message_id = ECHO_REQ;
					push_to_app(buf);
					buf = 0;
#endif
				}
				else if(buf && icmp_type==ROUTER_ADVRT_TYPE)
				{
					gw_table_update(buf);
					buf=0;
				}
#ifdef HAVE_ROUTING
				

				else if(buf && icmp_type == ICMP_ERROR_MESSAGE_TYPE)
				{
					if(icmp_code == ERROR_CODE_BROKEN_LINK || icmp_code == ERROR_CODE_NO_ROUTE)
					{
						ipv6_address_t ipv6_address;
						dptr += 6;
						for(i=0; i<16; i++)
						{
							ipv6_address[i] = *dptr++;
						}
						if(ipv6_address[0] == 0xfe && ipv6_address[1] == 0x80)
						{
							if((ipv6_address[12]== 0xfe && ipv6_address[11] == 0xff) && ipv6_address[13] == 0x00)
							{
								buf->dst_sa.addr_type=ADDR_802_15_4_PAN_SHORT;
								for(i=0; i<2; i++)
								{
									buf->dst_sa.address[i] = ipv6_address[14+i];
								}
							}
							else
							{
								buf->dst_sa.addr_type=ADDR_802_15_4_PAN_LONG;
								for(i=0; i<8; i++)
								{
									buf->dst_sa.address[7-i] = ipv6_address[8+i];
								}
							}
						}

						update_routing_table(buf->dst_sa.addr_type, buf->dst_sa.address, ADDR_NONE, NULL, 0, 0 , REMOVE_ROUTE);
						ptr = ( control_message_t*) buf->buf;
						ptr->message.ip_control.message_id = BROKEN_LINK;
						push_to_app(buf);
						buf = 0;
					}
					if(buf)
					{
						stack_buffer_free(buf);
						buf=0;
						return pdTRUE;
					}
				}

				else if(buf && icmp_type==ROUTER_SOLICICATION_TYPE)
				{
					if(gateway_features)
					{
						buf->dst_sa.addr_type = buf->src_sa.addr_type;
						buf->src_sa.addr_type = ADDR_NONE;
						memcpy(buf->dst_sa.address, buf->src_sa.address, 8);
						memset(buf->buf, 0, 16);
						buf->buf_ptr = 0;
						buf->buf_end = 16;
						dptr = buf->buf + buf->buf_ptr;

						*dptr++ = ROUTER_ADVRT_TYPE;
						*dptr++ = ICMP_CODE;
						*dptr++ = 0x00;
						*dptr++ = 0x00;
						*dptr++ = GENERAL_HOPLIMIT;
					}
					else
					{	
						stack_buffer_free(buf);
						buf=0;
						return pdTRUE;
					}								
				}
#endif /*HAVE_ROUTING*/
				else
				{
					stack_buffer_free(buf);
					buf=0;
				}
#endif	/*HAVE_NRP*/				
			break;	
		}
	}
	/* ICMP message build and forward */
	if(buf)
	{
		add_fcf(buf);
		stack_buffer_push(buf);
		buf=0;
	}
	return pdTRUE;
}

portCHAR icmp_check( buffer_t *buf )
{
	portCHAR retval = pdTRUE;
	if(buf->options.type == BUFFER_CONTROL)
	{
		control_message_t *ptr = ( control_message_t*) buf->buf;
		switch(ptr->message.ip_control.message_id)
		{
			case BROKEN_LINK:
				break;
			default:
				retval= pdFALSE;
				break;
		}
	}
	else
	{
		uint8_t icmp_type=0;
		icmp_type = buf->buf[buf->buf_ptr];
		switch(icmp_type)
		{
			case ROUTER_ADVRT_TYPE:
				break;
			case ROUTER_SOLICICATION_TYPE:
				break;

			case ECHO_REQUEST_TYPE:
				break;
			case ECHO_RESPONSE_TYPE:
				break;
			case ICMP_ERROR_MESSAGE_TYPE:
				break;
	
			default:
				debug("ICMP:not sup\r\n");
				retval = pdFALSE;
				break;
		}
	}
	return retval;
}
/**
 * Function set gateway features for ICMP module.
 *
 */
void enable_router_features( void )
{
	gateway_features=1;
}
