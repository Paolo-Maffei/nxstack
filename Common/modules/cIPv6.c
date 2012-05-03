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
 * \file     cIPv6.c
 * \brief    Lowpan and cIPv6 module.
 *
 *  Module includes LowPan adaptation module.
 */



#include <string.h>
#include <sys/inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#ifndef CIPV6_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"
#include "stack.h"
#include "buffer.h"
#include "address.h"

#include "module.h"
#include "neighbor_routing_table.h"
#include "control_message.h"
#include "event_timer.h"
#include "rf.h"
#include "cipv6.h"
#include "socket.h"

/*
[NAME]
CIPV6

[ID]
MODULE_CIPV6,

[INFO]
#ifdef HAVE_CIPV6
  {cipv6_init, cipv6_handle, cipv6_check, 0, MODULE_CIPV6, 41, ADDR_802_15_4_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR cipv6_init(buffer_t *buf);
extern portCHAR cipv6_handle( buffer_t *buf );
extern portCHAR cipv6_check( buffer_t *buf );

uint8_t check_broadcast_id(addrtype_t addrtype, address_t address, uint8_t sqn);
extern sockaddr_t mac_long;

#ifdef HAVE_MAC_15_4
extern portCHAR mac_handle(	buffer_t *buf );
#endif

#ifdef HAVE_RF_802_15_4
extern xQueueHandle rf_802_15_4_queue;
#endif

cipv6_ib_t cipv6_pib;
flooding_table_t flooding_table;
route_check_t route_check;
addrtype_t o_addrtype = ADDR_NONE;
address_t ori_address;
uint8_t dest_length=0, hops_to_ori=0;

nano_mesh_response_t nanomesh_delivery;
#ifdef HAVE_ROUTING
buffer_t *forward_buffer = 0;
#endif

/**
 *  Initialize cIPv6 module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR cipv6_init( buffer_t *buf )
{
	uint8_t i=0;	
	buf;
	flooding_table.count=0;
	for(i=0; i<FLOODING_INFO_SIZE; i++)
	{
		flooding_table.info[i].type = ADDR_NONE;
	}
	cipv6_pib.own_brodcast_sqn = 0;
	cipv6_pib.use_short_address=0;
	cipv6_pib.short_address[0] = 0xfe;
	cipv6_pib.short_address[1] = 0xff;
	cipv6_pib.use_full_compress = 1;
	routing_init();
	return pdTRUE;
}

void ip_address_setup( uint8_t short_addr, uint8_t *ptr)
{
	uint8_t i;
	cipv6_pib.use_short_address = short_addr;
	if(short_addr)
	{
		for(i=0; i<2;i++)
		{
			cipv6_pib.short_address[i] = *ptr++;
		}	
	}
} 

/**
 *  Set LoWPAN's compression mode.
 *
 *  \return  pdTRUE    OK
 */
void cipv6_compress_mode( uint8_t mode )
{	
	cipv6_pib.use_full_compress = mode;
}

/**
 *  Main cUDP buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR cipv6_handle( buffer_t *buf )
{
  	/* Process the packet */		
	if(cipv6_pib.own_brodcast_sqn == 0)
	{
		cipv6_pib.own_brodcast_sqn = mac_long.address[0];
	}	
	debug("IP:");
	switch(buf->dir)
	{
		case BUFFER_DOWN:
			//debug("IP:DOWN\r\n");
			debug("DOWN\r\n");
			if(cipv6_pib.use_full_compress)
				build_lowpan_header(buf);
			else
			{
#ifdef SUPPORT_UNCOMP_IPV6
				build_ipv6_header(buf);
#else
				stack_buffer_free(buf);
				buf=0;
#endif
			}
			break;

		case BUFFER_UP:			/* From Lower layer(usually RF802.15.4) */
			/* Lets check header start type */
			debug("UP");
			if(buf)
			{
				if(buf->buf[buf->buf_ptr] == IPV6)
				{
					debug("IPV6\r\n");
#ifdef SUPPORT_UNCOMP_IPV6
					parse_ipv_header(buf);
					buf=0;
#else
					stack_buffer_free(buf);
					buf=0;
#endif
				}
				else
				{
					switch (buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM )
					{
					/* Normal unicast send-mode */
						case DISPATCH_TYPE:
							debug("DIS\r\n");
							/* Parse Unicast packet from neighbor */
							parse_lowpan_packet(buf, DISPATCH_TYPE);
							break;	
			
						/* Packet deliverymode is mesh */	
						case MESH_ROUTING_TYPE: /* Mesh Routing */
							debug("MES\r\n");
							parse_lowpan_packet(buf, MESH_ROUTING_TYPE);
						break;	/* END_OF_MESH_ROUTING_TYPE */
			
						default:
							debug("Discard header\r\n");
							stack_buffer_free(buf);
						break;
					}
				}
			}
		break;
						
	}
return pdTRUE;
}

/**
 *  The cIPv6 buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE    is cIPv6
 *  \return  pdFALSE   is not cIPv6 or broken header
 */
portCHAR cipv6_check( buffer_t *buf )
{
	uint8_t temp;
	if(buf->buf[buf->buf_ptr] == IPV6)
	{
		/* Uncompressed IPv6 header dispatch */
		return pdTRUE; 
	}
	else
	{
		temp = (buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM);
		if(temp == DISPATCH_TYPE)
		{
			return pdTRUE; 
		}
#ifdef FRAGMENTATION
		else if(temp == FRAGMENT_TYPE)
		{
			return pdTRUE; 
		}
#endif
		else if(temp == MESH_ROUTING_TYPE)
		{
			return pdTRUE; 
		}
		else
		{
			return pdFALSE;
		}
	}
}


void parse_lowpan_packet(buffer_t *buf, uint8_t packet_type)
{
 	/* Process the packet */
 	uint8_t tmp_8=0,mesh_header=0,hops_left=0,i,bc_seq=0;
 	uint8_t lowpan_dispatch,tmp, bc=0;
	uint8_t broadcast_ori_check=0;
	uint8_t *dptr;

	match_type_t address_mode = NOT_OWN;

	dptr = buf->buf + buf->buf_ptr;

	if(packet_type==MESH_ROUTING_TYPE)
	{				
		mesh_header = *dptr;
		hops_left = (mesh_header & 0xf0);
		hops_left = hops_left >> 4;
		hops_left--;
		i = (mesh_header & 0x0f);
		tmp = hops_left;
		tmp <<= 4;
		*dptr++ = (tmp | i);
		/* Check Originator and Final-destination address */
		tmp=(mesh_header & MESH_ADDRESSTYPE_BM);
		tmp_8 = (tmp & O_ADDRESSTYPE_BM);

		if(tmp_8 == O_ADDRESSTYPE_16)
		{
			o_addrtype=ADDR_802_15_4_PAN_SHORT;
			dest_length=2;
		}
		else
		{
			o_addrtype=ADDR_802_15_4_PAN_LONG;
			dest_length=8;	
		}
	
		for(i=0; i < dest_length; i++)
		{
			ori_address[i] = *dptr++;
		}
		tmp_8 = (tmp & D_ADDRESSTYPE_BM);
	
		if(tmp_8 == D_ADDRESSTYPE_16)
		{
			buf->dst_sa.addr_type =  ADDR_802_15_4_PAN_SHORT;
			dest_length=2;
		}
		else
		{
			buf->dst_sa.addr_type=ADDR_802_15_4_PAN_LONG;
			dest_length=8;
		}
	
		for(i=0; i < dest_length; i++)
		{
			buf->dst_sa.address[i] = *dptr++;
		}
		/* compare final-destination address to own address */
		address_mode = OWN;
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
		{
			if (memcmp(buf->dst_sa.address, mac_long.address, 8) != 0)
			{
				if(stack_check_broadcast(buf->dst_sa.address, ADDR_802_15_4_PAN_LONG) != pdTRUE)
					address_mode = NOT_OWN;
				else
					address_mode = BCAST;
			}
		}
		else
		{
			if (memcmp(buf->dst_sa.address, cipv6_pib.short_address, 2) != 0)
			{
				if(stack_check_broadcast(buf->dst_sa.address, ADDR_SHORT) != pdTRUE)
					address_mode = NOT_OWN;
				else
					address_mode = BCAST;
			}
		}
		bc_seq=0;
		bc=0;
	}
	else
	{
		address_mode=OWN;
	}
	
	/* Parse rest of IP header */
	lowpan_dispatch=0;
	lowpan_dispatch = *dptr++;
	if(lowpan_dispatch == LOWPAN_BC0)
	{
		bc=1;
		bc_seq = *dptr++;
		lowpan_dispatch=0;
		lowpan_dispatch = *dptr++;
	}

	if(lowpan_dispatch == LOWPAN_HC1)
	{
		/* Check HC1 options */
		tmp_8= *dptr++;
		if((tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP || tmp_8 == HC1_NEXT_HEADER_UNCOMPRES_UDP) || tmp_8 == IP_HEADER_FOR_ICMP)
		{
			if(tmp_8 == IP_HEADER_FOR_ICMP)
			{
				buf->to = MODULE_ICMP;
			}
			else
			{
				buf->to = MODULE_CUDP;
				if(tmp_8 == HC1_NEXT_HEADER_COMPRESSED_UDP) /* Read udp encode field */
					buf->options.lowpan_compressed = *dptr++;
			}

			*dptr -= 1;
			hops_to_ori = (GENERAL_HOPLIMIT - *dptr);
			dptr++;

			if(tmp_8 == IP_HEADER_FOR_ICMP) 
				buf->options.hop_count = hops_to_ori;

			if(packet_type==MESH_ROUTING_TYPE)
			{
				/* Broadcast flow filter */
				i=0;
				if(compare_ori_to_own(o_addrtype, ori_address) == pdTRUE)
				{
					broadcast_ori_check = 2;
				}
				else
				{
					if(address_mode == BCAST || (bc !=0) )
					{
						broadcast_ori_check = check_broadcast_id(o_addrtype, ori_address, bc_seq);
						if(broadcast_ori_check == 0)
						{
							i=1;
						}
					}
				}	
#ifdef HAVE_ROUTING							
				if((hops_to_ori > 1 && broadcast_ori_check!=2) && buf->options.rf_lqi > 0x15)
				{
					update_routing_table( o_addrtype, ori_address, buf->src_sa.addr_type, buf->src_sa.address,  hops_to_ori, buf->options.rf_dbm, i);
				}
#endif
	
				if(bc && broadcast_ori_check !=1)
				{
					address_mode = DISCARD;
				}
			}
		}
		else
		{
			debug("Dis:HC1 err\r\n");
			address_mode = DISCARD;
		}
	}
	else
	{
		debug_int(lowpan_dispatch);
		debug("NOT sup\r\n");
		
		address_mode = DISCARD;
	}

	/* Handle  IP payload payload */
	switch (address_mode)
	{
		case OWN: /* we are final destination */
		case BCAST:
			/* Copy originator address to src field */
			if(packet_type == MESH_ROUTING_TYPE)
			{
				if(o_addrtype == ADDR_802_15_4_PAN_LONG)
					memcpy(buf->src_sa.address,ori_address, 8);
				else
					memcpy(buf->src_sa.address,ori_address, 2);
	
				buf->src_sa.addr_type = o_addrtype;
			}
			if(address_mode == BCAST && hops_left != 0)
			{
				/* Check sequence number if same than last sent/forwarded discard buffer */
#ifdef HAVE_ROUTING
				forward_buffer=0;
				/* Get new buffer for forwarding */
				forward_buffer = stack_buffer_get(0);
				if (forward_buffer)
				{
					/* Copy original buffer for forwarding */
					memcpy(forward_buffer, buf, sizeof(buffer_t) + buf->buf_end);
					forward_buffer->src_sa.addr_type = ADDR_NONE;
					forward_buffer->dst_sa.addr_type = ADDR_BROADCAST;
					forward_buffer->from = MODULE_CIPV6;
					forward_buffer->to = MODULE_NONE;
					forward_buffer->dir = BUFFER_DOWN;
					forward_buffer->options.type = BUFFER_DATA;
#ifdef HAVE_MAC_15_4
					mac_handle(	forward_buffer);
#else
					stack_buffer_push(forward_buffer);
#endif
					forward_buffer=0;
				}
#endif
			}

			if(buf)
			{
				buf->buf_ptr = (dptr - buf->buf); /*cut header*/
				buf->from = MODULE_CIPV6;
				buf->to = MODULE_NONE;
				stack_buffer_push(buf);
				buf=0;
				debug("IP:UP\r\n");
			}
		break;
		case NOT_OWN:/* We are not final destination */
#ifndef HAVE_NRP
#ifdef HAVE_ROUTING
			if(hops_left > 0)
			{
				/* Route packet */
				nanomesh_delivery = nano_mesh_forward(buf->dst_sa.addr_type, buf->dst_sa.address, &route_check );
				switch (nanomesh_delivery)
				{
					case NANOMESH_NO_ROUTE:
						debug("NR\r\n");
						ip_broken_link_notify(buf, 1);
						buf=0;
						break;

					case NANOMESH_FORWARD:
						/* Change next-hop address to rf_802_15_4 dest_address */
						memcpy(buf->dst_sa.address, route_check.next_hop, 10);
						buf->dst_sa.addr_type = route_check.address_type;
					case NANOMESH_NEIGHBOUR:
						buf->src_sa.addr_type = ADDR_NONE;
						buf->from = MODULE_CIPV6;
						buf->to = MODULE_NONE;
						buf->dir = BUFFER_DOWN;
						stack_buffer_push(buf);
						buf=0;
						break;
					default:
						debug("IP:Del err\r\n");
						stack_buffer_free(buf);
						buf=0;
						break;
				}
			}
			else
			{
				stack_buffer_free(buf);
				buf=0;
			}
#endif /*HAVE_ROUTING*/
#endif/*HAVE_NRP*/
		break;
		case DISCARD:
			stack_buffer_free(buf);
			buf=0;
		break;
	}
	/* Discard buffer */
	if(buf)
	{
		debug("Mesh par: dis\r\n");
		stack_buffer_free(buf);
		buf=0;
	}
}

uint8_t add_own_address(uint8_t *d_ptr);

/**
 * Function build lowpan header / mesh header.
 *
 * \param buf pointer to data
 */
portCHAR build_lowpan_header(buffer_t *buf)
{
 	uint8_t mesh_header ,header_size=0;
	control_message_t *ptr;
	uint8_t *dptr;
	uint8_t i;
	dptr=0;
	debug("IP:Build\r\n");
	mesh_header = MESH_ROUTING_TYPE;
	header_size = 3;
	if((buf->from == MODULE_CUDP) && buf->options.lowpan_compressed)
			header_size++;
	
	if(cipv6_pib.use_short_address)
	{
		mesh_header |= O_ADDRESSTYPE_16;
		header_size +=2;
	}
	else
	{
		mesh_header |= O_ADDRESSTYPE_64;
		header_size +=8;
	} 

	/* Check destination address and delivery mode from neighbor table */
	switch(buf->dst_sa.addr_type)
	{
		case ADDR_COORDINATOR:
			nanomesh_delivery = NANOMESH_NEIGHBOUR;
			break;
		
		case ADDR_BROADCAST:
			mesh_header |= D_ADDRESSTYPE_16;
			header_size +=2;
			nanomesh_delivery = NANOMESH_BROADCAST;
			break;
			
		case ADDR_802_15_4_PAN_SHORT:
			mesh_header |= D_ADDRESSTYPE_16;
			header_size +=2;
			goto mesh_neighbour_check;
			
		case ADDR_802_15_4_LONG:
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;			
		case ADDR_802_15_4_PAN_LONG:
			mesh_header |= D_ADDRESSTYPE_64;
			header_size +=8;

		default:
			mesh_neighbour_check:			
			/* Check neighbourtable */
			if(stack_check_broadcast(buf->dst_sa.address, buf->dst_sa.addr_type) == pdTRUE)
			{
					nanomesh_delivery = NANOMESH_BROADCAST;
					if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
					{
						header_size -=6;
						mesh_header &=(~ D_ADDRESSTYPE_64);
						mesh_header |=D_ADDRESSTYPE_16;
					}
			}
			else
			{
#ifdef HAVE_ROUTING
				nanomesh_delivery = nano_mesh_forward(buf->dst_sa.addr_type, buf->dst_sa.address, &route_check );
#else
				nanomesh_delivery = nano_mesh_forward(buf->dst_sa.addr_type, buf->dst_sa.address, NULL );
#endif
			}
			break;
	}
	
	switch (nanomesh_delivery)
	{
		case NANOMESH_NO_ROUTE:
			debug("NR\r\n");
#ifdef AD_HOC_STATE
			if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
			{
				buf->dst_sa.address[8] = 0xff;
				buf->dst_sa.address[9] = 0xff;
			}
#endif
		case NANOMESH_NEIGHBOUR:
			debug("Ne\r\n");
			header_size = 3;
			if(buf->options.lowpan_compressed && buf->from == MODULE_CUDP)
				header_size++;
			if(stack_buffer_headroom(buf,header_size) == pdFALSE)
			{
				ptr = ( control_message_t*) buf->buf;
				ptr->message.ip_control.message_id = TOO_LONG_PACKET;
				push_to_app(buf);
				buf=0;
				return pdTRUE;
			}
			buf->buf_ptr -= header_size; /* lowpan-dispatch+HC1 = header space */
			dptr = buffer_data_pointer(buf);
			break;

		case NANOMESH_BROADCAST:
			debug("BC\r\n");
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
			for(i=0; i<4; i++)
			{
				buf->dst_sa.address[i] =0xff;
			}
		case NANOMESH_FORWARD:
			debug("MESH\r\n");
			if(nanomesh_delivery==NANOMESH_BROADCAST)
			{
				header_size +=3;
#ifdef HAVE_NRP
				mesh_header |= (GENERAL_HOPLIMIT<<4);
#else
#ifdef HAVE_ROUTING
				if(buf->options.hop_count)
					mesh_header |= (buf->options.hop_count<<4);
				else
					mesh_header |= (1<<4);
#else
				mesh_header |= (1<<4);
#endif/*HAVE_ROUTING*/
#endif/*HAVE_NRP*/
			}
			else
			{
				header_size++;		/* Mesh options */
				mesh_header |= BASIC_HOP_VALUE;
			}

			if(stack_buffer_headroom(buf,header_size) == pdFALSE)
			{
				ptr = ( control_message_t*) buf->buf;
				ptr->message.ip_control.message_id = TOO_LONG_PACKET;
				push_to_app(buf);
				buf=0;
				return pdTRUE;
			}
			buf->buf_ptr -= header_size; /* lowpan-dispatch+HC1 = header space */
			dptr = buffer_data_pointer(buf);
			/* Mesh frame type */
			*dptr++ = mesh_header;
			/* Build Mesh delivery address field*/
			/* Originator address */
			dptr += add_own_address(dptr);
			if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
				dest_length=2;
			else
				dest_length=8;

			for(i=0; i<dest_length ;i++)
			{
				*dptr++ = buf->dst_sa.address[i];
			}

			if(nanomesh_delivery == NANOMESH_BROADCAST)
			{
				buf->dst_sa.addr_type = ADDR_BROADCAST;
				buf->src_sa.addr_type = ADDR_NONE;
				/* Mesh broadcast header type and sequence number */
				*dptr++ = LOWPAN_BC0;
				*dptr++ = cipv6_pib.own_brodcast_sqn;
				update_ip_sqn();
			}
			else
			{
				/* Change next-hop address to rf_802_15_4 dest_address */
				memcpy(buf->dst_sa.address, route_check.next_hop, 10);
				buf->dst_sa.addr_type = route_check.address_type;
			}
			break;
	}

	/* General Lowpan and cIPv6 headers */
	if(buf)
	{
		*dptr++ = LOWPAN_HC1; 	/* LowPan Dispatch	 */
		/* HC1 encode */
		if(buf->from == MODULE_ICMP)
			*dptr++ = IP_HEADER_FOR_ICMP;
		else
		{
			if(buf->options.lowpan_compressed)
			{
				*dptr++ = HC1_NEXT_HEADER_COMPRESSED_UDP;
				*dptr++ = buf->options.lowpan_compressed;
			}		
			else 
				*dptr++ = HC1_NEXT_HEADER_UNCOMPRES_UDP;
		}
		*dptr++ = GENERAL_HOPLIMIT;	/* Hop-limit	*/
		buf->from = MODULE_CIPV6;
		buf->to = MODULE_NONE;
		stack_buffer_push(buf);
		buf=0;
	}
	return pdTRUE;
}

uint8_t add_own_address(uint8_t *d_ptr)
{
	uint8_t i;
	if(cipv6_pib.use_short_address)
	{
		for (i=0; i<2; i++)
		{
			*d_ptr++ = cipv6_pib.short_address[i];
		}
		return 2;
	}
	else
	{
		for (i=0; i<8; i++)
		{
			*d_ptr++ = mac_long.address[i];
		}
		return 8;
	}
}

void update_ip_sqn(void)
{
	/* Increace broadcast_counter */
	if(cipv6_pib.own_brodcast_sqn==0xff) cipv6_pib.own_brodcast_sqn=1;
	else cipv6_pib.own_brodcast_sqn++;
}
#ifdef SUPPORT_UNCOMP_IPV6
/**
 * Build IPv6 header.
 *
 * Function create whole uncompressed header and add dispatch for that --> Header size 41 byte.
 * \param buf pointer to data
 */
portCHAR build_ipv6_header(buffer_t *buf)
{
	uint8_t next_header=0, i;
	uint8_t *dptr;
	uint16_t payload_length=0;
	
	dest_delivery_t destination_delivery = NOT_NEIGHBOR;
	memset(ipv6_address, 0,8);
	payload_length = (buf->buf_end - buf->buf_ptr);

	if(stack_buffer_headroom( buf,41)==pdFALSE)
	{
		control_message_t *ptr = ( control_message_t*) buf->buf;
		ptr->message.ip_control.message_id = TOO_LONG_PACKET;
		push_to_app(buf);
		buf=0;
		return pdTRUE;
	}
	buf->buf_ptr -= 41;
	/* Check deliverymode from neighbour table */
	if(buf->dst_sa.addr_type != ADDR_BROADCAST && buf->dst_sa.addr_type != ADDR_COORDINATOR)
	{
		destination_delivery = check_neighbour_table(buf->dst_sa.addr_type, buf->dst_sa.address);
		if(destination_delivery==BROADCAST)
		{
			buf->dst_sa.addr_type = ADDR_BROADCAST;
		}
	}
	if(buf->from == MODULE_CUDP)
		next_header = NEXT_HEADER_UDP;
	else
		next_header = NEXT_HEADER_ICMP6;

	dptr = buf->buf + buf->buf_ptr;
	*dptr++ = IPV6;
	*dptr++ = 0x06;
	*dptr++ = 0x00;
	*dptr++ = 0x00;
	*dptr++ = 0x00;
	*dptr++ = (payload_length >> 8);	
	*dptr++ = (uint8_t) payload_length;			/* Length */
	*dptr++ = next_header;
	*dptr++ = GENERAL_HOPLIMIT;
	/* Add source */
	if(cipv6_pib.use_short_address)
	{
		ipv6_address[4]=0xff;
		ipv6_address[3]=0xfe;
		ipv6_address[1]=cipv6_pib.short_address[1];
		ipv6_address[0]=cipv6_pib.short_address[0];
	}
	else
	{
		for(i=0; i<8; i++)
		{
			ipv6_address[i] = mac_long.address[7-i];
		}
	}
	*dptr++ =0xfe;
	*dptr++ = 0x80;
	for(i=0; i<6; i++)
	{
		*dptr++ = 0x00;
	}
	for(i=0; i<8; i++)
	{
		*dptr++ = ipv6_address[i];
	}
	memset(ipv6_address, 0,8);
	 /* Link Local IPV6 ADDRESS PREFIX */
	/* Check destination */
	/* Destination */
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT ||  buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
	{

		if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
		{
			for(i=0;i<8;i++)
			{
				ipv6_address[i] = buf->dst_sa.address[7-i];
			}
		}
		else
		{
			ipv6_address[3]=0xff;
			ipv6_address[4]=0xfe;
			ipv6_address[6]=buf->dst_sa.address[1];
			ipv6_address[7]=buf->dst_sa.address[0];
		}

		*dptr++ =0xfe;
		*dptr++ = 0x80;

		for(i=0; i<6; i++)
		{
			*dptr++ = 0x00;
		}

		for(i=0; i<8; i++)
		{
			*dptr++ = ipv6_address[i];
		}
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		*dptr++ =0xff;
		*dptr++ = 0x02;	
		for(i=0; i<13; i++)
		{
			*dptr++ = 0x00;
		}
		*dptr++ = 0x01;
	}
	buf->from = MODULE_CIPV6;
	buf->to = MODULE_NONE;
	stack_buffer_push(buf);
	buf=0;
	return pdTRUE;
}

/**
 * Parse IPv6 header.
 *
 * Function support only link-local unicast and multicast for all nodes.
 * \param buf pointer to data
 */
void parse_ipv_header(buffer_t *buf)
{
	uint8_t ind=0, ip_version,hoplimit=0, next_header=0, i, dest_match=0;
	ipv6_address_t destination;
	ind = buf->buf_ptr;
	ind++;
	/* Check IP version */
	ip_version = buf->buf[ind++];
	if( ip_version == 0x06 )
	{	
		debug("IPv6 packet.\r\n");
		/* Check HC1 options */
		ind +=5;
		next_header = buf->buf[ind++];
		hoplimit= buf->buf[ind++];
		hoplimit--;
		ind +=16;
		for(i=0; i<16; i++)
		{
			destination[15-i] = buf->buf[ind++];
		}
		if(destination[15] == 0xfe && destination[14] == 0x80)
		{
			//debug("link local unicast\r\n");
			if(destination[3] == 0xfe && destination[3] == 0xff)
			{
				if(memcmp(destination, cipv6_pib.short_address, 2) ==0)
					dest_match=1;
			}
			else
			{
				if(memcmp(destination, mac_long.address, 8) ==0)
					dest_match=1;
			}	

		}
		if((destination[15] == 0xff && destination[14] == 0x02) && destination[0] == 0x01)
		{	
			//debug("link local multicast for all nodes\r\n");
			dest_match=1;
		}

		buf->options.lowpan_compressed = 0;
		if(next_header==NEXT_HEADER_ICMP6) buf->options.hop_count = 1;
		if(hoplimit > 0 && dest_match)
		{
			buf->buf_ptr = ind; /*cut header*/
			buf->to = MODULE_NONE;
			buf->from = MODULE_CIPV6;
			stack_buffer_push(buf);
			buf=0;
		}
		else
		{	
			//debug("destination not match\r\n");
		}
	}/* END_OF_LOWPAN_HC1 */
	/* General buffer discard part */	
	if(buf)
	{
		debug("Discard.\r\n");
		stack_buffer_free(buf);
		buf=0;
	}
}
#endif
portCHAR compare_ori_to_own(addrtype_t type, address_t address)
{
	if(type==ADDR_802_15_4_PAN_LONG)
	{
		if(memcmp(address,mac_long.address , 8)==0)
			return pdTRUE;
	}
	else
	{
		if(memcmp(address,cipv6_pib.short_address ,2)==0)
			return pdTRUE;
	}
	return pdFALSE;
}


#ifndef NO_FCS
uint8_t ipv6_in_use = 0;

uint8_t ipv6_pseudo[40] = 
{
    // source address
    0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // destination address
    0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // payload length
    0x00, 0x00, 0x00, 0x00,
    // next header
    0x00, 0x00, 0x00, 0x00
};
	
/**
 * Check & Calculate FCF.
 *
 * Function calculate typical IP FCF which is used IPv6 and ICMP protocols.
 * \param buf pointer to data
 * \param next_protocol NEXT_HEADER_UDP / NEXT_HEADER_ICMP6
 * \param rx_case flag when function now it have to check and when calculate
 *
 * \return fcf 16-values
 */
uint16_t ipv6_fcf(buffer_t *buf, uint8_t next_protocol, uint8_t rx_case)
{
	uint32_t sum = 0;
	uint16_t cksum;
	uint8_t i, tmp=0;
	uint8_t *p_payload = buffer_data_pointer(buf);
	uint16_t length = buffer_data_length(buf);
	
	while (ipv6_in_use)
	{
		vTaskDelay(200/portTICK_RATE_MS);
	};
	
	ipv6_in_use = 1;
	//Create and init IPv6 pseudo header array
	memset(ipv6_pseudo, (uint8_t) 0, 40);
	ipv6_pseudo[0] = 0xfe;
	ipv6_pseudo[1] = 0x80;
	ipv6_pseudo[16] = 0xfe;
	ipv6_pseudo[17] = 0x80;

#ifndef AD_HOC_STATE
#ifdef MAC_RFD
	if ((rx_case == 0) && (buf->dst_sa.addr_type == ADDR_COORDINATOR))
	{
		if(get_coord_address(&buf->dst_sa) !=pdTRUE)
		{

		}
	}
#endif
#endif	
	if(rx_case)
	{
		//RX case use buffers source address field
		if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG)
		{
			for(i=0;i<8;i++)
			{
				ipv6_pseudo[8+i] = buf->src_sa.address[7-i];
			}	
		}
		else
		{
			ipv6_pseudo[11]=0xff;
			ipv6_pseudo[12]=0xfe;
			ipv6_pseudo[14]=buf->src_sa.address[1];
			ipv6_pseudo[15]=buf->src_sa.address[0];	
		}
	}
	else
	{
		// Source address
		if(cipv6_pib.use_short_address)
		{
			ipv6_pseudo[11]=0xff;
			ipv6_pseudo[12]=0xfe;
			ipv6_pseudo[14]=cipv6_pib.short_address[1];
			ipv6_pseudo[15]=cipv6_pib.short_address[0];	
		}
		else
		{
			for(i=0; i<8; i++)
			{
				ipv6_pseudo[8+i] = mac_long.address[7-i];	
			}
		}
	}	
	//Destination address
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
	{
		tmp=0;
		for(i=0;i<8;i++)
		{
			if(buf->dst_sa.address[i] !=0xff)
			{
				tmp=1;
				i=8;
			}
		}

		if(tmp)
		{
			for(i=0;i<8;i++)
			{
				ipv6_pseudo[24+i] = buf->dst_sa.address[7-i];
			}	
		}
		else
			buf->dst_sa.addr_type = ADDR_BROADCAST;
	}
	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
	{
		if((buf->dst_sa.address[1] !=0xff) && (buf->dst_sa.address[0] !=0xff))
		{
			ipv6_pseudo[27]=0xff;
			ipv6_pseudo[28]=0xfe;
			ipv6_pseudo[30]=buf->dst_sa.address[1];
			ipv6_pseudo[31]=buf->dst_sa.address[0];	
		}
		else
			buf->dst_sa.addr_type = ADDR_BROADCAST;	
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		// Multicast to all nodes ff02: :01
		ipv6_pseudo[16] =0xff;
		ipv6_pseudo[17] = 0x02;	
		ipv6_pseudo[31] = 0x01;
	}

	ipv6_pseudo[35] = length;
	ipv6_pseudo[39] = next_protocol;

	//Calculate sum of ipv6_pseudo header
	for ( i = 0; i < sizeof(ipv6_pseudo); i += 2 )
	{
		uint16_t x = (uint16_t)(ipv6_pseudo[ i ] << 8) | ipv6_pseudo[ i+1 ];
		sum += x;
	}
	// Calculate sum of upper-layer payload
	for ( i = 0; i < length; i += 2 )
	{
		uint16_t x;
		// account for odd-length payload, pad with 0
		if ( (i+1) == length )
		x = (uint16_t)(p_payload[ i ] << 8);
		else
		x = (uint16_t)(p_payload[ i ] << 8) | p_payload[ i+1 ];
		sum += x;
	}
	cksum = (sum >> 16) + (sum & 0xffff);
	
	if(rx_case==0)
	{
		if ( cksum != 0xffff )
			cksum = ~cksum;
	}
	ipv6_in_use = 0;
	return cksum;
}
#endif /*NO_FCS*/

/**
 * Function notify for Broken link.
 *
 * Function remove allways neighbour info which not answer and if packet is Mesh routing header it will also remove routing info to destination.
 * Then it check originator address, if address is device own it will notify application.
   if address is not own it will forward ctrl message to ICMP module which create ICMP error message to originator.
 * \param buf pointer to data
 */
void ip_broken_link_notify(buffer_t *buf, uint8_t no_route_reason)
{
	uint8_t i, length=0, mesh_header, tmp, *ind;
	control_message_t *msg;
	debug("IP:BR\r\n");
	update_neighbour_table(buf->dst_sa.addr_type, buf->dst_sa.address, -80, 0, REMOVE_NEIGHBOUR);
	if(buf->buf[buf->buf_ptr] == IPV6)
	{
		i=99;
	}
	else
	{
		buf->src_sa.addr_type = ADDR_NONE;
		if((buf->buf[buf->buf_ptr] & LOWPAN_TYPE_BM) ==  MESH_ROUTING_TYPE)
		{
			ind = (buf->buf + buf->buf_ptr);
			mesh_header = *ind++;
			/* Check Originator and Final-destination address */
			tmp=(mesh_header & MESH_ADDRESSTYPE_BM);
			if((tmp & O_ADDRESSTYPE_BM) == O_ADDRESSTYPE_16)
			{
				buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
				length=2;
			}
			else
			{
				buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
				length=8;	
			}
			for(i=0; i < length; i++)
			{
				buf->src_sa.address[i] = *ind++;
			}
		
			if((tmp & D_ADDRESSTYPE_BM) == D_ADDRESSTYPE_16)
			{
				buf->dst_sa.addr_type =  ADDR_802_15_4_PAN_SHORT;
				length=2;
			}
			else
			{
				buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
				length=8;
			}
			for(i=0; i < length; i++)
			{
				buf->dst_sa.address[i] = *ind++;
			}
#ifdef HAVE_ROUTING
			update_routing_table(buf->dst_sa.addr_type, buf->dst_sa.address, ADDR_NONE, NULL, 0, 0 , REMOVE_ROUTE);
#endif
		}
		else
		{
			i=99;	/* Destination is neighbour */
		}
	}

	if(length==0 )
	{
		if(buf->dst_sa.addr_type ==  ADDR_802_15_4_PAN_SHORT)
			length=2;
		else
			length=8;
	}
	buf->options.type = BUFFER_CONTROL;
	msg = ( control_message_t*) buf->buf;
	msg->message.ip_control.message_id = BROKEN_LINK;
	buf->from = MODULE_CIPV6;
	buf->socket = 0;

	if(no_route_reason)
		msg->message.ip_control.message.broken_link_detect.reason = NO_ROUTE_TO_DESTINATION_TYPE;
	else
		msg->message.ip_control.message.broken_link_detect.reason = BROKEN_LINK_TYPE;
	
	msg->message.ip_control.message.broken_link_detect.type = buf->dst_sa.addr_type;
	memcpy(msg->message.ip_control.message.broken_link_detect.address,buf->dst_sa.address,length);

	if(i==99 || buf->src_sa.addr_type == ADDR_NONE)
	{
		//buf->to = MODULE_APP;
		debug("--> APP\r\n");
		push_to_app(buf);
	}
	else
	{
		if(compare_ori_to_own(buf->src_sa.addr_type, buf->src_sa.address) == pdTRUE)
		{
			//buf->to = MODULE_APP;
			debug("--> APP\r\n");
			push_to_app(buf);
		}
		else
		{
			debug("--> ICMP\r\n");
			buf->to = MODULE_ICMP;
			buf->options.handle_type = HANDLE_DEFAULT;
			buf->dir = BUFFER_UP;
			stack_buffer_push(buf);
		}
	}
	/*if(buf->to == MODULE_APP)
	{
		debug("--> APP\r\n");
		push_to_app(buf);
	}
	else
	{
		debug("--> ICMP\r\n");
		buf->options.handle_type = HANDLE_DEFAULT;
		buf->dir = BUFFER_UP;
		stack_buffer_push(buf);
	}*/
}

uint8_t flood_indexs=0;

uint8_t check_broadcast_id(addrtype_t addrtype, address_t address, uint8_t sqn)
{
	uint8_t i, length=0;
	if(addrtype == ADDR_802_15_4_PAN_SHORT)
		length=2;
	else
		length=8;

	if(flooding_table.count)
	{
		for(i=0; i<FLOODING_INFO_SIZE; i++)
		{
			if(flooding_table.info[i].type == addrtype)
			{
				if(memcmp(flooding_table.info[i].address, address, length)==0)
				{
					if(flooding_table.info[i].last_sqn != sqn)
					{
						flooding_table.info[i].last_sqn = sqn;
						return 1;
					}
					else
						return 0;
				}
			}
		}
		if(flooding_table.count == FLOODING_INFO_SIZE)
		{
			flooding_table.info[flood_indexs].type = addrtype;
			memcpy(flooding_table.info[flood_indexs].address, address, length);
			flooding_table.info[flood_indexs].last_sqn = sqn;
			flood_indexs++;
			if(flood_indexs == 8)
				flood_indexs = 0;
			return 1;
		}
	}
	i=flooding_table.count;
	flooding_table.info[i].type = addrtype;
	memcpy(flooding_table.info[i].address, address, length);
	flooding_table.info[i].last_sqn = sqn;
	flooding_table.count++;
	return 1;
}
