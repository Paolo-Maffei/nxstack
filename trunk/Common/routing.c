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
 * \file     routing.c
 * \brief    Routing module which handle routing & neighbourtable informations.
 *
 *  This module will be usefull for every modules who want check neighbor_tables and routing_tables
 *  
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#undef HAVE_DEBUG
#include <sys/inttypes.h>
#include <string.h>
#include "address.h"
#include "neighbor_routing_table.h"
#include "socket.h"
#include "debug.h"
#include "bus.h"

#define TTL		15	
#define ROUTING_TTL	15
#ifdef HAVE_ROUTING
routing_table_t routing_table;
#endif
void check_tables_status(void *unused_parameter);
xSemaphoreHandle table_lock = NULL;
neighbor_table_t neighbor_table;


uint8_t n_neigh_buffer=0;
/**
 * Initialize Neighbour and Routing tables.
 *
 * Create also semaphore for tables.
 *
 */

void routing_init(void)
{
	uint8_t i=0;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
	neighbor_info_t *b;
	vSemaphoreCreateBinary( table_lock );
	if(table_lock != NULL)
	{
		/* Init table info */
		while (i < MAX_NEIGHBOR_COUNT)
		{
			b = &(neighbor_table.neighbor_info[i]);
			b->type=ADDR_NONE;
			i++;
		}
		neighbor_table.count=0;
		neighbor_table.child_count=0;
#ifdef HAVE_ROUTING
		i=0;
		routing_table.count=0;
		while (i < MAX_ROUTE_INFO_COUNT)
		{	
			ptr = &(routing_table.route_info[i]);
			ptr->dest_addr_type=ADDR_NONE;
			i++;
		}
#endif
	}
}
/**
 * Function for look for neighbourtable.
 *
 * After check shuold run free_table_semaphore()
 *
 * \return address for neighbour table
 */
neighbor_table_t* get_neighbour_info(void)
{
	if( xSemaphoreTake( table_lock, ( portTickType ) 0 ) == pdTRUE )
	{
		return &(neighbor_table);
	}
	return 0;
}


/**
 * Update neighbor tables if necessary.
 *
 * Mac-layer use this function every time when received packet which LQI > 0.
 *
 * \param type indicates type of neighbor address mode
 * \param address neighbor address
 * \param lqi Last received LQI value
 * \param last_sqn last MAC sqn from this address
 *
 * \return 1 when last_sqn is different than current
 * \return 0 when sqn is same, now MAC discard packet
 */
uint8_t update_neighbour_table(addrtype_t type, address_t address, int8_t last_rssi, uint8_t last_sqn, uint8_t remove)
{
	neighbor_info_t *b;
	uint8_t i,j, sqn_check=0, length=0;
	dest_delivery_t delivery_mode;
	delivery_mode = NOT_NEIGHBOR;
	

	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		if(type==ADDR_802_15_4_PAN_LONG)
		{
			length=8;					
		}
		if(type == ADDR_802_15_4_PAN_SHORT)
			length=4;

		delivery_mode = NOT_NEIGHBOR;
		if(neighbor_table.count > 0 && remove != ADD_CHILD)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;

				if(b && (type == b->type))
				{
					if(memcmp(b->address, address,length) == 0)
						delivery_mode = NEIGHBOR;
					
					/* Update lqi and compare sqn to old one */
					if( delivery_mode == NEIGHBOR )
					{
						if(type != ADDR_802_15_4_PAN_SHORT)
						{
							for(j=0; j<2; j++)
							{
								b->address[length+j] = address[length+j];
							}
						}
						if(remove == REMOVE_NEIGHBOUR)
						{
							if(b->child_dev)
								neighbor_table.child_count--;

							b->type=ADDR_NONE;
							i=neighbor_table.count;
							neighbor_table.count--;
						}
						else
						{
							/* Duplicated packet check */
							if(b->last_sqn != last_sqn)
							{
								b->last_sqn = last_sqn;
								sqn_check=1;
							}
							b->last_rssi = last_rssi;
							b->ttl=TTL;
						}
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}
		/* Add new neighbor if addresstype is source */
		if((delivery_mode == NOT_NEIGHBOR && remove != REMOVE_NEIGHBOUR) && neighbor_table.count < MAX_NEIGHBOR_COUNT)
		{
			for(i=0; i<MAX_NEIGHBOR_COUNT; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
				{
					i=MAX_NEIGHBOR_COUNT;
				}
			}

				if(type==ADDR_802_15_4_PAN_LONG)
						length+=2;

				for(j=0; j < length ; j++)
				{
					b->address[j] = address[j];
				}				
				/* add lqi value to neighbor */
				if(remove  == ADD_CHILD)
				{
					neighbor_table.child_count++;
					b->child_dev=1;
				}
				b->last_rssi =	last_rssi;
				b->last_sqn  =    last_sqn;
				b->child_dev =	0;
				sqn_check=1;
				b->ttl=TTL;
				b->type = type;
				/* Increace Neigbor count */
				neighbor_table.count++;
		}
		xSemaphoreGive( table_lock ); /*free lock*/
	}
	else
	{
		debug("No sem\r\n");
		sqn_check=1;
	}
	return sqn_check;
}

/**
 * Update neighbor & routing table TTL when unicast TX working.
 *
 * \param type indicates type of address
 * \param address 
 */
void update_tables_ttl(addrtype_t type, address_t address)
{
	uint8_t i, length=0, final_length=0;
	neighbor_info_t *b;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		if(type==ADDR_802_15_4_PAN_LONG)
		{
			length=8;
			final_length =10;					
		}
		if(type == ADDR_802_15_4_PAN_SHORT)
		{
			length=4;
			final_length=4;
		}

		if(neighbor_table.count > 0)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;
	
				if(b && (type == b->type))
				{
					if(memcmp(b->address, address,length) == 0)
					{
						b->ttl=TTL;
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}
#ifdef HAVE_ROUTING
		if(routing_table.count > 0)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
					ptr=0;

				if(ptr && (type == ptr->next_hop_addr_type))
				{
					/* compare next hop address */
					if(memcmp(address, ptr->next_hop, final_length) ==0)
						ptr->ttl=ROUTING_TTL;
				}
			}
		}
#endif
		xSemaphoreGive( table_lock ); /*free lock*/
	}
}

/**
 * Check destination address from neighbor.
 *
 * IP-layer uses this function when it has to forward packet.
 *
 * \param type indicates type of destination address mode
 * \param address destination address
 *
 * \return BROADCAST when address was broadcast.
 * \return NEIGHBOR when address was already in list.
 * \return NO_SUPPORT when address type not valid.
 */
dest_delivery_t check_neighbour_table(addrtype_t type, address_t address)
{
	uint8_t i,j, broadcast=1, length;
	addrtype_t temp = ADDR_NONE;
	dest_delivery_t delivery_mode = NOT_NEIGHBOR;
	neighbor_info_t *b;
	switch (type)
	{
		case ADDR_802_15_4_PAN_SHORT:		
			/* Check if broadcast address */
			length=2;
			temp = ADDR_SHORT;
			break;
		case ADDR_802_15_4_PAN_LONG:
			length=8;
			temp = type;
			break;
		default:
			debug("No supported address field\r\n");
			return NO_SUPPORT;
			break;
	}
	/* Check if broadcast address */
	if(stack_check_broadcast(address, temp) != pdTRUE)
			broadcast = 0;

	/* NOT BROADCAST */
	if(broadcast == 0)
	{
		if(neighbor_table.count > 0)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;

				if(b && (b->type == type))
				{
					if(memcmp(b->address, address,length) == 0)
					{
						if(b->last_rssi < -80)
							delivery_mode = NEIGHBOR_LOW_RSSI;
						else
							delivery_mode = NEIGHBOR;
						
						for(j=0; j<2; j++)
						{
							address[length+j] = b->address[length+j];
						}
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}
		else
			delivery_mode = NOT_NEIGHBOR;
	}
	else
		delivery_mode = BROADCAST;

	return delivery_mode;	
}


nano_mesh_response_t nano_mesh_forward(addrtype_t type, address_t address, route_check_t *r_check )
{
	dest_delivery_t table_check;
	nano_mesh_response_t forward_response = NANOMESH_NO_ROUTE;
	
	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		table_check = check_neighbour_table(type, address);
		switch (table_check)
		{
			case BROADCAST:
				forward_response = NANOMESH_BROADCAST;
				break;
	
			case NEIGHBOR:
				forward_response = NANOMESH_NEIGHBOUR;
				break;
	
			case NEIGHBOR_LOW_RSSI:
			case NOT_NEIGHBOR:
				if(r_check != NULL)
				{
#ifdef HAVE_ROUTING
					if(check_routing_table(type, address, r_check ) == pdTRUE)
						forward_response = NANOMESH_FORWARD;
					else
					{
						if(table_check==NEIGHBOR_LOW_RSSI)
							forward_response = NANOMESH_NEIGHBOUR;
						else
							forward_response = NANOMESH_NO_ROUTE;
					}
#else
					if(table_check==NEIGHBOR_LOW_RSSI)
							forward_response = NANOMESH_NEIGHBOUR;
						else
							forward_response = NANOMESH_NO_ROUTE;
#endif
				}
				else
					forward_response = NANOMESH_NO_ROUTE;
				break;
			default:
				break;
	
		}
		xSemaphoreGive( table_lock ); /*free lock*/
	}
	return forward_response;
}

/**
 * Update new child information.
 *
 * IP-layer uses this function when it has to forward packet.
 *
 * \param type indicates type of child address mode
 * \param address child address
 *
 * \return NOT_CHILD wrong address type.
 * \return CHILD added.
 * \return DISCARD_ASSOC assocation discard.
 */
#ifdef MAC_FFD
#ifndef AD_HOC_STATE
child_status_type_t check_child_role(addrtype_t type, address_t address)
{
	neighbor_info_t *b;
	uint8_t i,j, length;
	child_status_type_t return_value;
	return_value = NOT_CHILD;
	
	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		switch (type)
		{
			case ADDR_802_15_4_PAN_SHORT:		
				/* Check if broadcast address */
				length=4;
				break;
			case ADDR_802_15_4_SHORT:		
				/* Check if broadcast address */
				length=2;
				type=ADDR_802_15_4_PAN_SHORT;
				break;
			case ADDR_802_15_4_PAN_LONG:
				length=8;
				break;
			default:
				xSemaphoreGive( table_lock ); /*free lock*/
				return return_value;
				break;
		}
		if(neighbor_table.count > 0)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
			{
				b = &(neighbor_table.neighbor_info[i]);
				if(b->type == ADDR_NONE)
					b=0;

				if(b && (b->type == type) )
				{
					if(memcmp(b->address, address,length) == 0)
					{
						if(b->child_dev == 0)
						{
							neighbor_table.child_count++;
							b->child_dev=1;
						}
						return_value = CHILD;
						i=MAX_NEIGHBOR_COUNT;
					}
				}
			}
		}

		if((return_value==NOT_CHILD) && (neighbor_table.child_count == NWK_MAX_CHILD) )
		{
			return_value = DISCARD_ASSOC;

		}

		if((return_value==NOT_CHILD) && (neighbor_table.child_count < NWK_MAX_CHILD))
		{
			j =neighbor_table.child_count;
			j++;
				if(j == NWK_MAX_CHILD)
					return_value=NO_CAPASITY_AFTER_NEW_CHILD;
		}
		xSemaphoreGive( table_lock ); /*free lock*/
	}
return return_value;	
}
#endif /* AD_HOC_STATE */
#endif /* MAC_FFD */

/**
 * Check route for destination address.
 *
 * If destination address is not neighbor IP-layer checks route to destination.
 *
 * \param type indicates type of destination address mode
 * \param address destination address
 * \param r_check pointer for route check response
 */
#ifdef HAVE_ROUTING
portCHAR check_routing_table(addrtype_t type, address_t address, route_check_t *r_check )
{
	uint8_t i,j, final_length, next_hop_addr_length=0;
	portCHAR ret_val=pdFALSE;
	route_info_t *ptr;
	/* Check destination address from routing_table */
	if(type==ADDR_802_15_4_PAN_LONG)
		final_length=8;
	else
		final_length=2;

	if(routing_table.count > 0)
	{
		for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
		{
			ptr = &(routing_table.route_info[i]);
			if(ptr->dest_addr_type == ADDR_NONE)
				ptr=0;
			if(ptr && (type == ptr->dest_addr_type))
			{
			
				if(memcmp(ptr->destination, address,final_length) == 0)
				{
					if(ptr->next_hop_addr_type==ADDR_802_15_4_PAN_LONG)
						next_hop_addr_length=10;
					else
						next_hop_addr_length=4;

					for(j=0; j < next_hop_addr_length; j++)
					{
						r_check->next_hop[j] = ptr->next_hop[j];
					}
					r_check->address_type = ptr->next_hop_addr_type;
					i=MAX_ROUTE_INFO_COUNT;
					ret_val=pdTRUE;
				}
			}	
		}
	}
	return ret_val;
}
#endif

/**
 * Update route info to routing table.
 *
 * If IP-layer detects that distance to originator device is over 1, then layer updates route info to routing table.
 *
 * \param type indicates type of destination address mode.
 * \param final_destination final-destination address.
 * \param next_hop next_hop address.
 * \param hop_count hop count to originator.
 * \param lqi link quality indication for next_hop.
 * \param only_check situation when received same broadcast sqn than last forwarded, now only comprare next hop address, lqi & hop count not add new route.
 */
#ifdef HAVE_ROUTING
portCHAR update_routing_table(addrtype_t final_type, address_t final_destination,addrtype_t next_hop_type, address_t next_hop, uint8_t hop_count, int8_t last_rssi , uint8_t only_check)
{
	uint8_t i=0,j, tmp_8=0, final_length, next_hop_length, compare=0, update=0;
	route_info_t *ptr;
	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		if(final_type==ADDR_802_15_4_PAN_LONG)
			final_length=8;
		else
			final_length=2;
		if(next_hop_type==ADDR_802_15_4_PAN_LONG)
			next_hop_length=8;
		else
			next_hop_length=4;

		tmp_8 = 0;
		/* Predict older route information and shuold use route */
		if(only_check != REMOVE_ROUTE)
		{
			switch	(check_time_stamp(final_type, final_destination))
			{
				case MESH_TTL_VALID:
					tmp_8=1;		/* cancel update process */
					break;
				case MESH_LOW_RSSI:
				case MESH_NOT_NEIGHBOR:
					only_check=0;
					break;
				default:
					break;
			}
		}

		if(routing_table.count > 0 && tmp_8==0)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT ; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
					ptr=0;
				/* Check originator address from routing table */
				if(ptr && (final_type == ptr->dest_addr_type))
				{
					if(memcmp(ptr->destination, final_destination,final_length) ==0)
					{
						if(only_check == REMOVE_ROUTE)
						{
							ptr->dest_addr_type=ADDR_NONE;
							routing_table.count--;
						}
						else
						{
							if(next_hop_type==ptr->next_hop_addr_type)
							{
								/* compare next hop address */
								if(memcmp(next_hop, ptr->next_hop, next_hop_length) !=0)
									compare=1;
								else
									update=2;
							}
							else
								compare=1;
	
							if(compare)
							{
								if(hop_count < ptr->hop_count && last_rssi > -85)
								{
									update=1;	
								}
								else
								{
									if(hop_count==ptr->hop_count)
									{
										if(last_rssi > ptr->last_rssi || (ptr->ttl  < (ROUTING_TTL - 2)  ))
											update=1;
									}
								}
							}
							if(update)
							{
								if(update != 2)
								{
									ptr->next_hop_addr_type = next_hop_type;
									next_hop_length+=2;
									/* added new next hop info */
									for(j=0; j < next_hop_length ; j++)
									{
										ptr->next_hop[j] = next_hop[j];
									}
								}
								ptr->last_rssi=last_rssi;
								ptr->hop_count = hop_count;
								ptr->ttl=ROUTING_TTL;
							}
						}
						tmp_8=1;
						i=MAX_ROUTE_INFO_COUNT;
					}
				}	
			}
		}

		if(only_check==0 && (tmp_8==0 && routing_table.count < MAX_ROUTE_INFO_COUNT ))
		{
			//uint8_t count = routing_table.count;
			for(i=0; i<MAX_ROUTE_INFO_COUNT; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type == ADDR_NONE)
				{
					i=MAX_ROUTE_INFO_COUNT;
				}
			}
			for(j=0; j < final_length ; j++)
			{
				ptr->destination[j] = final_destination[j];		
			}
			next_hop_length+=2;
			for(j=0; j < next_hop_length ; j++)
			{
				ptr->next_hop[j] = next_hop[j];
			}
			ptr->next_hop_addr_type = next_hop_type;
			ptr->dest_addr_type = final_type;

			ptr->hop_count = hop_count;
			ptr->ttl=ROUTING_TTL;
			ptr->last_rssi=last_rssi;
			routing_table.count++;
		}
		xSemaphoreGive( table_lock ); /*free lock*/
	}
return pdTRUE;
}
#endif

/**
 * Checkout routing- and neighbor tables time-stamp.
 *
 * If function detect that TTL, time to live value is coming zero it delete route or neighbor-info.
 *
 */
void check_tables_status(void *unused_parameter)
{
	uint8_t i;
	neighbor_info_t *b;
#ifdef HAVE_ROUTING
	route_info_t *ptr;
#endif
	if( xSemaphoreTake( table_lock, ( portTickType ) 5 ) == pdTRUE )
	{
		if(neighbor_table.count)
		{
			for(i=0; i < MAX_NEIGHBOR_COUNT; i++)
			{
				b=&(neighbor_table.neighbor_info[i]);
				if(b->type==ADDR_NONE)
					b=0;

				if(b)
				{
					if(b->ttl > 1)
						b->ttl--;
					else
					{
						if(b->child_dev)
							neighbor_table.child_count--;

						b->type=ADDR_NONE;
						neighbor_table.count--;
						
					}
				}
			}
		}
#ifdef HAVE_ROUTING
		if(routing_table.count)
		{
			for(i=0; i < MAX_ROUTE_INFO_COUNT; i++)
			{
				ptr=&(routing_table.route_info[i]);
				if(ptr->dest_addr_type==ADDR_NONE)
					ptr=0;

				if(ptr)
				{
					if(ptr->ttl > 1)
						ptr->ttl--;
					else
					{
						ptr->dest_addr_type=ADDR_NONE;
						routing_table.count--;
					}
				}
			}
		}
#endif
		xSemaphoreGive( table_lock ); /*free lock*/
	}
}
/**
 * Printout neighbour and routing table information.
 *
 *
 */

#ifdef ROUTING_TEST
void print_table_information(void)
{
	neighbor_info_t *b;
#ifdef HAVE_ROUTING
	uint8_t addres_length=0;
	route_info_t *ptr;
#endif
	if( xSemaphoreTake( table_lock, ( portTickType ) 10 ) == pdTRUE )
	{
		uint8_t i, j;
		if(neighbor_table.count)
		{
			debug("Neighbor Info count:");
			debug_hex(neighbor_table.count);
			debug("\r\n");
			debug("Child count:");
			debug_hex(neighbor_table.child_count);
			debug("\r\n");
			for(i=0; i < MAX_NEIGHBOR_COUNT; i++)
			{
				b=&(neighbor_table.neighbor_info[i]);
				if(b->type==ADDR_NONE)
					b=0;
				if(b)
				{
					if(b->type== ADDR_802_15_4_PAN_LONG)
					{
						debug("Long:  ");
						for(j=0; j < 2 ; j++)
						{
							if (j) debug_put(':');
							debug_hex( b->address[9-j]);
						}
						debug("  ");
						for(j=0; j < 8 ; j++)
						{
							if (j) debug_put(':');
							debug_hex( b->address[7-j]);
						}
						
					}
					if(b->type == ADDR_802_15_4_PAN_SHORT)
					{
						debug("Short:  ");
						for(j=0; j < 2 ; j++)
						{
							if (j) debug_put(':');
							debug_hex( b->address[3-j]);
						}
						debug("  ");
						for(j=0; j < 2 ; j++)
						{
							if (j) debug_put(':');
							debug_hex( b->address[1-j]);
						}
					}
					debug("\r\nrssi: ");
					debug_int(b->last_rssi);
					debug("\r\nTTL: ");
					debug_hex(b->ttl);
					debug("\r\n");
					pause_us(200);
				}
			}
		}
		else
		{
			debug("No Neighbor info\r\n");
		}
#ifdef HAVE_ROUTING
		if(routing_table.count)
		{
			
			debug("\r\nroute Info count:");
			debug_hex(routing_table.count);
			debug("\r\n");
			
			for(i=0; i < MAX_ROUTE_INFO_COUNT; i++)
			{
				ptr = &(routing_table.route_info[i]);
				if(ptr->dest_addr_type==ADDR_NONE)
					ptr=0;

				if(ptr)
				{
					debug("Dest:  ");
					if(ptr->dest_addr_type==ADDR_802_15_4_PAN_LONG)
						addres_length=8;
					else
						addres_length=2;

					for(j=0; j < addres_length ; j++)
					{
						if (j) debug_put(':');
						debug_hex(ptr->destination[(addres_length-1)-j]);
					}
					debug("\r\nNext hop:  ");
					if(ptr->next_hop_addr_type==ADDR_802_15_4_PAN_LONG)
						addres_length=10;
					else
						addres_length=4;

					for(j=0; j < addres_length ; j++)
					{
						if (j) debug_put(':');
						debug_hex(ptr->next_hop[(addres_length-1)-j]);
					}
		
					debug("\r\nrssi: ");
					debug_int(ptr->last_rssi);
					debug("\r\nHop count:  ");
					debug_hex(ptr->hop_count);
					debug("\r\nTTL: ");
					debug_hex(ptr->ttl);
					debug("\r\n");
				}
			}
		}
		else
		{
			debug("No route info\r\n");
		}
#else
		debug("Routing disable\r\n");
#endif
		xSemaphoreGive( table_lock ); /*free lock*/
	}
}
#else
void print_table_information(void)
{
	debug("Table information not required\r\n");
}
#endif
#ifdef MAC_FFD
#ifdef HAVE_ROUTING

/**
 * Function reason is that NanoMesh not add routing info for own neighbours.
 *
 * NanoMesh will call this function automatick when it update routing info.
 * \return pdTRUE when neighbours TTL is coming older
 * \return pdFALSE when current is Active. This time NanoMesh cancel update process.
 */
check_neig_t check_time_stamp(addrtype_t type, address_t address)
{
	neighbor_info_t *b;
	uint8_t i,length=0;
	
	if(type== ADDR_802_15_4_PAN_SHORT)	
		length=2;
	if(type== ADDR_802_15_4_PAN_LONG)
		length=8;

	if(neighbor_table.count > 0)
	{
		for(i=0; i < MAX_NEIGHBOR_COUNT ; i++)
		{
			b=&(neighbor_table.neighbor_info[i]);
			if(b->type==ADDR_NONE)
				b=0;

			if(b && (b->type == type))
			{
				if(memcmp(b->address, address,length) ==0)
				{
					if(b->last_rssi < -88)
					{
						return MESH_LOW_RSSI;
					}
					if(b->ttl > (TTL -2))
						return MESH_TTL_VALID;
					else
						return MESH_TTL_OLD;
				}
			}
		}
	}
	return MESH_NOT_NEIGHBOR;
}
#endif /*HAVE_ROUTING*/
#endif /*MAC_FFD*/
