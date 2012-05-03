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
 * \file     protocol_lib.c
 * \brief    Nanostack support applications.
 *
 *  Module includes Ping, Gateway disvover, Gateway advertisment... .
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#undef HAVE_DEBUG

#include "debug.h"
//#include "stack.h"
#include "control_message.h"
#include "neighbor_routing_table.h"
#include "semphr.h"
#include "cipv6.h"
#include "socket.h"


extern uint8_t gateway_features;

#ifdef APP_ECHO
uint16_t ping_sqn=0, udp_echo_sqn=0;
discover_res_t  *result = NULL;
portTickType echo_start;
uint8_t parse=0;
#endif

#ifdef GW_AUTO_DISCOVER_SERVICE
xSemaphoreHandle gw_table_lock = NULL;
#ifdef HAVE_ROUTING
gateway_cache_t *gw_info;
#endif
#endif

stack_event_t event_bus=0;
sock_handler_func event_handler=0;


/**
 * Stack service init.
 *
 * Init stack event bus using message queue or callback function. Also init automatick GW tracking.
 *
 * \param stack_event pointer for stack evet message queue, give NULL if want use function callback
 * \param stack_event_handler handler for stack event callback if want poll message queue give NULL
 * \param gateway_discover boolean value for GW tracking, 1 enable automatick tarcking then have to give pointer for GW-table
 * \param gw_table pointer to global GW-table to application, give NULL if
 * \return pdFALSE service initialize fail
 * \return pdTRIE Initilize complete 
 */
portCHAR stack_service_init( stack_event_t stack_event, sock_handler_func stack_event_handler, uint8_t gateway_discover , gateway_cache_t *gw_table )
{
#ifdef GW_AUTO_DISCOVER_SERVICE
#ifdef HAVE_ROUTING
	if(gateway_discover)
	{
		gw_info = gw_table;
		vSemaphoreCreateBinary( gw_table_lock );
		if(gw_table_lock == NULL)
			return pdFALSE;
	}
#endif
#endif
	if(stack_event != NULL)
	{
		event_bus = stack_event;
	}
	else if(stack_event_handler != NULL)
	{
		event_handler = stack_event_handler;
	}
	else
	{
		return pdFALSE;
	}
	return pdTRUE;
}
/**
 * Parse stack evet message.
 *
 * Function parse stack event message and return event type.
 *
 * \param buf pointer for stack event message
 * \return BROKEN_LINK when packet transmission was failed to destination.
 * \return ROUTER_DISCOVER_RESPONSE GW-table updated by GW advertisment.
 * \return PENDING_TIMEOUT Indirect data send was not Polling from pending proxy.
 * \return TOO_LONG_PACKET application payload too long to send.
 * \return ASSOCIATION_WITH_COORDINATOR RFD device was complete assocation with Cordinator now RFD can send data.
 * \return ASSOCIATION_LOST Assocation to Cordinator losted
 * \return COORDINATOR_STARTED Cordinators network and MAC setups ready.
 * \return GATEWAY_STARTED Gateways network and MAC setups ready.
 * \return NOT_ASSOCIATED Error status when RFD device try to send data before assocation.
 * \return MAC_QUEUE_FULL MAC task message queue full then application shuold wait a while.
 * \return MAC_CCA_ERR Channel reserve fail and transmission fail also.
 */

ip_control_id_t parse_event_message(buffer_t *buf)
{
	
	if(buf->options.handle_type == HANDLE_NO_ROUTE_TO_DEST)
	{
		return DATA_BACK_NO_ROUTE;
	}
	else
	{
		ip_control_id_t response;
		control_message_t *tmp_msg;
		tmp_msg = ( control_message_t*) buf->buf;
		response	=tmp_msg->message.ip_control.message_id;
		return response;
	}
}

#ifdef APP_ECHO
/**
 * Stop echo process.
 *
 * Have to call end of echo process.
 */
void stop_ping(void)
{
	parse = 0;
}
#ifdef HAVE_ICMP
#ifdef APP_ICMP_ECHO
/**
 * ICMP echo service call.
 *
 * Function call send ICMP echo request to network.
 * \param dst destination address for unicast echo, NULL broadcast destination
 * \param result_ptr pointer for echo response table
 */
portCHAR ping(sockaddr_t *dst, discover_res_t  *result_ptr)
{
	uint8_t *dptr;
	buffer_t *buf;
	result = result_ptr;
	
	buf = stack_buffer_get(20);
	if(buf)
	{
		parse=1;
		buf->buf_ptr=0;
		buf->buf_end = 8;
		buf->options.hop_count = 5;
		buf->to 	= 	MODULE_CIPV6; 
		buf->from 	= 	MODULE_ICMP;
		buf->dir 	= 	BUFFER_DOWN;
		dptr = buf->buf + buf->buf_ptr;
		*dptr++ = ECHO_REQUEST_TYPE;
		*dptr++ = ICMP_CODE;
		*dptr++ = 0x00;
		*dptr++ = 0x00;
		/* Group id */
		*dptr++ = 0x00;
		*dptr++ = 0x00;
		/* SQN */
		*dptr++ = (ping_sqn >> 8);
		*dptr++ = (uint8_t) ping_sqn;
		if(ping_sqn==0xffff)
			ping_sqn=0;
		else
			ping_sqn++;
		if(dst==NULL)
		{
			buf->dst_sa.addr_type = ADDR_BROADCAST;
			memset(buf->dst_sa.address, 0xff, 4);
		}
		else
		{
			memcpy(&(buf->dst_sa), dst, sizeof(sockaddr_t));
		}

		add_fcf(buf);
		stack_buffer_push(buf);
		//buf=0;
		echo_start = xTaskGetTickCount();
		return pdTRUE;
	}
	return pdFALSE;
}
#endif
#endif
#ifdef APP_UDP_ECHO
/**
 * UDP echo service call.
 *
 * Function call send UDP echo request to network.
 * \param dst destination address for unicast echo, NULL broadcast destination
 * \param result_ptr pointer for echo response table
 */
portCHAR udp_echo(sockaddr_t *dst, discover_res_t  *result_ptr)
{
	uint8_t *dptr;
	buffer_t *buf;
	result = result_ptr;
	buf = stack_buffer_get(20);
	if(buf)
	{
		parse=1;
		buf->buf_ptr=0;
		buf->buf_end = 2;
		buf->options.hop_count = 5;
		buf->to 	= 	MODULE_CUDP; 
		buf->from 	= 	MODULE_APP;
		buf->dir 	= 	BUFFER_DOWN;
		dptr = buf->buf + buf->buf_ptr;
		/* SQN */
		*dptr++ = (udp_echo_sqn >> 8);
		*dptr++ = (uint8_t) udp_echo_sqn;
		if(udp_echo_sqn==0xffff)
			udp_echo_sqn=0;
		else
			udp_echo_sqn++;
		if(dst==NULL)
		{
			buf->dst_sa.addr_type = ADDR_BROADCAST;
			memset(buf->dst_sa.address, 0xff, 4);
		}
		else
			memcpy(&(buf->dst_sa), dst, sizeof(sockaddr_t));
	
		buf->dst_sa.port = 7;
		buf->src_sa.port = 0;
		stack_buffer_push(buf);
		//buf=0;
		echo_start = xTaskGetTickCount();
		return pdTRUE;
	}
	return pdFALSE;
}
#endif
/**
 * Parse Echo response message.
 *
 * Parse Echo response messages to response table.
 * \param buf pointer for message
 */
portCHAR parse_echo_response(buffer_t *buf)
{
	uint8_t i;
	uint16_t temp_16;
	uint8_t *dptr;
	
	if(parse)
	{
		dptr = buf->buf + buf->buf_ptr;
		temp_16 = ((uint16_t) (*dptr++) << 8);
		temp_16 += *dptr++;
		if(buf->from == MODULE_ICMP)
		{
			if(temp_16 != (ping_sqn - 1))
			{
				debug("dis icmp\r\n");
				stack_buffer_free(buf);
				//buf=0;
				return pdTRUE;
			}
		}
		else
		{
			if(temp_16 != (udp_echo_sqn - 1))
			{
				debug("dis udp\r\n");
				stack_buffer_free(buf);
				//buf=0;
				return pdTRUE;
			}
		}
		i = result->count;
		if(i < PING_RESPONSE_MAX)
		{
			memcpy(&(result->result[i].src), &(buf->src_sa), sizeof(sockaddr_t));
			result->result[i].time = (xTaskGetTickCount()-echo_start)*portTICK_RATE_MS;
			result->result[i].rssi = buf->options.rf_dbm;
			i++;
			result->count=i;
		}
	}
	if(buf)
	{
		stack_buffer_free(buf);
	}
	return pdTRUE;
}
#else
portCHAR parse_echo_response(buffer_t *buf)
{
	stack_buffer_free(buf);
}
#endif


/**
 * Gateway advertisment.
 *
 * Function generate and send GW advertisment message to network.
 */
portCHAR gw_advertisment(void)
{
	uint8_t *dptr;
	buffer_t *buf;
	if(gateway_features)
	{
		buf = stack_buffer_get(20);
		if(buf)
		{
			buf->to 	= 	MODULE_CIPV6; 
			buf->from 	= 	MODULE_ICMP;
			buf->dir 	= 	BUFFER_DOWN;
			memset(buf->buf, 0, 16);
			buf->dst_sa.addr_type = ADDR_BROADCAST;
			memset(buf->dst_sa.address, 0xff, 4);
			buf->options.hop_count = 5;
			buf->buf_ptr = 0;
			buf->buf_end = 16;
			dptr = buf->buf + buf->buf_ptr;
	
			*dptr++ = ROUTER_ADVRT_TYPE;
			*dptr++ = ICMP_CODE;
			*dptr++ = 0x00;
			*dptr++ = 0x00;
			*dptr++ = GENERAL_HOPLIMIT;
			add_fcf(buf);
			stack_buffer_push(buf);
			return pdTRUE;
		}
	}
	return pdFALSE;
}

#ifdef GW_AUTO_DISCOVER_SERVICE
/**
 * Gateway discover.
 *
 * Function generate and send GW discover message to network.
 */
portCHAR gw_discover(void)
{
	uint8_t *dptr;
	buffer_t *buf = stack_buffer_get(20);
	if(buf)
	{
		buf->buf_ptr = 0;
		buf->buf_end = 8;
		buf->to 	= 	MODULE_CIPV6; 
		buf->from 	= 	MODULE_ICMP;
		buf->dir 	= 	BUFFER_DOWN;
		memset(buf->buf, 0, 8);
		buf->options.hop_count = 5;
		buf->dst_sa.addr_type = ADDR_BROADCAST;
		memset(buf->dst_sa.address, 0xff, 4);
		dptr = buf->buf + buf->buf_ptr;
		*dptr++ = ROUTER_SOLICICATION_TYPE;
		*dptr++ = ICMP_CODE;
		add_fcf(buf);
		stack_buffer_push(buf);
		//buf=0;
		return pdTRUE;
	}
	return pdFALSE;
}


/**
 * Gateway table update.
 *
 * Function update GW info to table when GW discover is enabled.
 */
portCHAR gw_table_update(buffer_t *buf)
{
	uint8_t tmp_count, flag=1, i, addres_length;
	control_message_t *ptr;

	if(gw_table_lock == NULL)
	{
		stack_buffer_free(buf);
		//buf=0;
	}
	else
	{
		
		if( xSemaphoreTake( gw_table_lock, ( portTickType ) 5 ) == pdTRUE )
		{
			if(buf->src_sa.addr_type==ADDR_802_15_4_PAN_LONG)
				addres_length=8;
			else
				addres_length=2;
			tmp_count = gw_info->count;
			if(tmp_count)
			{
				/*Check first cache*/
				for(i=0; i<tmp_count;i++)
				{
					if(gw_info->gateway_info[i].address_type == buf->src_sa.addr_type)
					{
						if(memcmp(gw_info->gateway_info[i].address,buf->src_sa.address,addres_length)==0)
						{
							gw_info->gateway_info[i].lqi = buf->options.rf_lqi;
							gw_info->gateway_info[i].ttl = 7;
							if(gw_info->gateway_info[i].hop_distance!=buf->options.hop_count)
								gw_info->gateway_info[i].hop_distance = buf->options.hop_count;
		
							flag=0; 
						}
					}
				}
			}
			if(flag)
			{
				gw_info->gateway_info[tmp_count].address_type = buf->src_sa.addr_type;
				gw_info->gateway_info[tmp_count].hop_distance = buf->options.hop_count;
				gw_info->gateway_info[tmp_count].lqi = buf->options.rf_lqi;
				gw_info->gateway_info[tmp_count].ttl = 7;
				for(i=0;i<addres_length;i++)
				{
					gw_info->gateway_info[tmp_count].address[i]=buf->src_sa.address[i];
				}
				tmp_count++;
				gw_info->count=tmp_count;
			}
			xSemaphoreGive( gw_table_lock ); /*free lock*/
	
			buf->buf_end = 0;
			buf->buf_ptr = 0;
			ptr = ( control_message_t*) buf->buf;
			ptr->message.ip_control.message_id = ROUTER_DISCOVER_RESPONSE;
			push_to_app(buf);
			//if(xQueueSend( event_bus, ( void * ) &buf, ( portTickType ) 0 )!=pdTRUE)
				//debug("event queue full\r\n");
	
			return pdTRUE;
		}
	}
	return pdFALSE;
}

/**
 * Select best Gateway.
 *
 * Function select best GW from table.
 * \param adr pointer to socket address type where best gateway info saved
 * \return pdTRUE When GW selected
 * \return pdFALSE select failed
 */
portCHAR select_best_gw(sockaddr_t *adr)
{
	uint8_t gw_index=0;
	if(gw_table_lock != NULL)
	{

		if(gw_info->count && xSemaphoreTake( gw_table_lock, ( portTickType ) 10 ) == pdTRUE )
		{
			if(gw_info->count==1) gw_index=0;
			else
			{
				/* Compare Hop distance */
				if(gw_info->gateway_info[0].hop_distance != gw_info->gateway_info[1].hop_distance)
				{
					if(gw_info->gateway_info[0].hop_distance < gw_info->gateway_info[1].hop_distance) gw_index=0;
					else gw_index=1;
				}
				else
				{
					/* Compare LQI values */
					if(gw_info->gateway_info[0].lqi > gw_info->gateway_info[1].lqi) gw_index=0;
					else gw_index=1;
				}
			}
			memcpy(adr, &(gw_info->gateway_info[gw_index]), sizeof(sockaddr_t));
			xSemaphoreGive( gw_table_lock ); /*free lock*/
			return pdTRUE;
		}
	}

	return pdFALSE;
}
/**
 * Check gateway info status.
 *
 * Function remove old gateways info if it coming old.
 */
portCHAR update_gw_info_ttl(void)
{
	uint8_t gw_index=0, i, j;
	if(gw_table_lock != NULL)
	{
		
		if( xSemaphoreTake( gw_table_lock, ( portTickType ) 10 ) == pdTRUE )
		{
			gw_index = gw_info->count;
			if(gw_index)
			{
				/*Check first cache*/
				for(i=0; i<gw_index;i++)
				{
					if(gw_info->gateway_info[i].ttl > 1)
						gw_info->gateway_info[i].ttl--;
					else
					{
						if(i==0 && gw_index==2)
						{
							j=i;
							j++;
							memcpy(&(gw_info->gateway_info[i]), &(gw_info->gateway_info[j]), sizeof(gateway_info_t));
							/* Update also copied information */
							if(gw_info->gateway_info[i].ttl > 1)
								gw_info->gateway_info[i].ttl--;
							else
								gw_info->count--;
						}
						
						i=gw_info->count;
						gw_info->count--;
					}
				}
			}
			xSemaphoreGive( gw_table_lock ); /*free lock*/
			return pdTRUE;
		}
	}
	return pdFALSE;
}
/**
 * Manual GW info remove.
 *
 * Remove gateway info.
 */
portCHAR remove_gw_info(sockaddr_t *adr)
{
	uint8_t i;
	uint8_t len=0;

	if(gw_table_lock != NULL)
	{
		
		if( xSemaphoreTake( gw_table_lock, ( portTickType ) 10 ) == pdTRUE )
		{
			if(gw_info->count)
			{
				
				if(adr->addr_type == ADDR_802_15_4_PAN_LONG)
					len=8;
				else
					len=2;
		
				for(i=0; i<gw_info->count; i++)
				{
					if(gw_info->gateway_info[i].address_type == adr->addr_type)
					{
						if(memcmp(gw_info->gateway_info[i].address, adr->address, len)==0)
						{
							gw_info->count--;
							xSemaphoreGive( gw_table_lock ); /*free lock*/
							return pdTRUE;
						}
					}
				}
			}
			xSemaphoreGive( gw_table_lock ); /*free lock*/
		}
	}
	return pdFALSE;
}
#else
portCHAR gw_table_update(buffer_t *buf)
{
	stack_buffer_free(buf);
	return pdTRUE;
}
#endif
#ifndef AD_HOC_STATE
void scan_network(void)
{
	nwk_manager_launch();
}
#endif
/**
 * Stack event to APP.
 *
 * Push stack event messges from stack to message queue.
 */
void push_to_app(buffer_t *buf)
{
	if(event_handler)
	{
		event_handler((void *)buf);
	}
	else if(event_bus)
	{
		if(xQueueSend( event_bus, ( void * ) &buf, ( portTickType ) 0 )!=pdTRUE)
		{
				stack_buffer_free(buf);
		}
	}
	else
	{
		stack_buffer_free(buf);
	}
}


