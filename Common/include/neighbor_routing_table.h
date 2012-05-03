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
 * \file neigbor_routing_table.h
 * \brief control_message type defined
 *
 *  nanoStack: These tables are for zigbee and 6lowpan modules
 *   
 *	
 */

 
#ifndef NEIGHBOR_ROUTING_TABLE_H
#define NEIGHBOR_ROUTING_TABLE_H
#ifdef NEIGHBOR_MAX
#define MAX_NEIGHBOR_COUNT	NEIGHBOR_MAX
#else
#define MAX_NEIGHBOR_COUNT	20
#endif

#ifdef NEIGHBOR_MIN
#define MIN_NEIGHBOR_INFO_COUNT	NEIGHBOR_MIN
#else
#define MIN_NEIGHBOR_INFO_COUNT	15
#endif

#ifdef ROUTE_MAX
#define MAX_ROUTE_INFO_COUNT	ROUTE_MAX
#else
#define MAX_ROUTE_INFO_COUNT	15
#endif

#ifdef ROUTE_MIN
#define MIN_ROUTE_INFO_COUNT	ROUTE_MIN
#else
#define MIN_ROUTE_INFO_COUNT	10
#endif

#ifndef NWK_MAX_CHILD
#define NWK_MAX_CHILD 200
#endif
#define FLOODING_INFO_SIZE 8

#include "address.h"

/** Neighbor info structure. */
typedef struct
{
	address_t address;					/*!< neighbor address. */
	addrtype_t type;					/*!< address type. */
	int8_t last_rssi;
	uint8_t last_sqn;					/*!< Last sqn for detect duplicated packet */
	uint8_t ttl;						/*!< Neighbor infos Time to Live */
	uint8_t child_dev;					/*!< 1=Child */
} neighbor_info_t;

/** Neighbor tables structure. */
typedef struct
{
	uint8_t				count;									/*!< count of neighbor info. */
	uint8_t				child_count;							/*!< count of associated childs. */
	neighbor_info_t 	neighbor_info[MAX_NEIGHBOR_COUNT];		/*!< list of neighbor infos. */	
} neighbor_table_t;


/** Flooding filter. */
typedef struct
{
	uint8_t		address[8];
	addrtype_t type;
	uint8_t last_sqn;					/*!< Last sqn for detect duplicated packet */
} flooding_info_t;

/** Neighbor tables structure. */
typedef struct
{
	uint8_t				count;									/*!< count of neighbor info. */
	flooding_info_t	 	info[FLOODING_INFO_SIZE];								/*!< list of neighbor infos. */	
} flooding_table_t;


/** Destination match type enumeration for received packet check_child_role return this kind of type. */
typedef enum
{
	NOT_CHILD = 0,					/*!< When device addresstype is not write. */
	CHILD,							/*!< Device is current child. */
	DISCARD_ASSOC,					/*!< This device cant join this PAN-network. */	
	NO_CAPASITY_AFTER_NEW_CHILD,	/*!< NWK_MAX child limit is full after this child. */
	NEW_CHILD						/*!< NWK manger has added new child. */
}child_status_type_t;

typedef enum
{
	MESH_NOT_NEIGHBOR = 0,	
	MESH_TTL_OLD,
	MESH_TTL_VALID,
	MESH_LOW_RSSI
}check_neig_t;


/** Check child status. */
typedef struct
{
	uint8_t				 child_count;				/*!< count of neighbor info. */
	child_status_type_t	 status;					/*!< list of neighbor infos. */	
} check_child_status_t;


/** Delivery mode enumeration for neighbortable update function. */
typedef enum
{
	BROADCAST = 0,		/*!< address is broadcast. */
	NEIGHBOR,			/*!< address is Neighbors. */
	NEIGHBOR_LOW_RSSI,	/*!< address is Neighbors which have low RSSI value. */
	NOT_NEIGHBOR,		/*!< address is not neighbors. */
	NEW_NEIGHBOR,		/*!< address is not neighbors. */
    ROUTED_MESSAGE,
	NO_SUPPORT		/*!< No support. */
}dest_delivery_t;


typedef enum
{
	NANOMESH_BROADCAST = 0,		
	NANOMESH_FORWARD,
	NANOMESH_NEIGHBOUR,
	NANOMESH_NO_ROUTE
}nano_mesh_response_t;

/** Destination match type enumeration for received packet. */
typedef enum
{
	BCAST = 0xc0,		/*!< Destination address is broadcast --> use mesh routing. */
	OWN,				/*!< Destination address is device own. */
	NOT_OWN,			/*!< Destination address not own --> check route (neighbor/routing-table). */
	DISCARD				/*!< When header or bco sgn not valid. */
}match_type_t;

/** Gateway information structure. */
typedef struct
{
	addrtype_t	address_type;		/*!< Gateways address type. */
	address_t	address;			/*!< Gateways address. */	
	uint8_t		hop_distance;		/*!< Hop distance to Gateway. */
	uint8_t		lqi;				/*!< LQI to next-hop. */
	uint8_t		ttl;				/*!< Time live value that Gateway is valid. */
}gateway_info_t;

/** Address field */
typedef uint8_t css_addr_t[2];

/** Coordinator cache. */
typedef struct
{
	css_addr_t	css_addr;
}css_cache_t;


/** Coordinator cache. */
typedef struct
{
	css_addr_t	cord_address;
	uint8_t		css_count;
	css_cache_t	css[4];
}coordinator_info_t;

/** Coordinator cache. */
typedef struct
{
	uint8_t				count;
	coordinator_info_t	cord_info[4];
}cord_cache_t;

/** Gateway cache. */
typedef struct
{
	uint8_t				count;				/*!< Count of valid Gateways. */
	gateway_info_t		gateway_info[2];	/*!< Gateway information array. */
}gateway_cache_t;


#ifdef HAVE_ROUTING
/** Route info structure. */
typedef struct
{
	addrtype_t		dest_addr_type;		/*!< final destinatio address type. */
	uint8_t 	destination[8];		/*!< destination address. */
	addrtype_t		next_hop_addr_type;	/*!< next-hop address type. */
	address_t	next_hop;			/*!< next-hop address. */
	uint8_t 	hop_count;			/*!< Hop count to destination. */
	uint8_t 	ttl;				/*!< Routing infos Time to Live */
	int8_t 		last_rssi;
} route_info_t;

/** Routing tables structure. */
typedef struct
{
	uint8_t			count;		/*!< address destination count. */
	route_info_t 	route_info[MAX_ROUTE_INFO_COUNT];	/*!< list of route. */
} routing_table_t;
#endif
/** Route check functions response structure. */
typedef struct
{
	addrtype_t	address_type;		/*!< next-hop address type. */
	address_t	next_hop;			/*!< next-hop address. */
	uint8_t		hop_count;			/*!< Hop count to destination. */
}route_check_t;





/* Functions prototypes for handling routing and neighbor tables. */
extern nano_mesh_response_t nano_mesh_forward(addrtype_t type, address_t address, route_check_t *r_check );
extern dest_delivery_t check_neighbour_table(addrtype_t type, address_t address);
extern uint8_t update_neighbour_table(addrtype_t type, address_t address, int8_t last_rssi, uint8_t last_sqn, uint8_t remove);
//extern uint8_t update_neighbour_table(addrtype_t type, address_t address, int8_t last_rssi, uint8_t last_sqn);
extern void routing_init(void);
extern void neigh_buffer_free( neighbor_info_t *b );
extern neighbor_info_t* neigh_buffer_get( portTickType blocktime );
extern void print_table_information(void);
extern check_neig_t check_time_stamp(addrtype_t type, address_t address);
extern void remove_broken_route(addrtype_t type, address_t address);
//extern void remove_broken_link(addrtype_t type, address_t address);
extern neighbor_table_t* get_neighbour_info(void);
extern void update_tables_ttl(addrtype_t type, address_t address);
extern void check_tables_status(void *unused_parameter);
#ifdef HAVE_ROUTING
extern portCHAR update_routing_table(addrtype_t final_type, address_t final_destination,addrtype_t next_hop_type, address_t next_hop, uint8_t hop_count, int8_t last_rssi , uint8_t only_check);
extern portCHAR check_routing_table(addrtype_t type, address_t address, route_check_t *r_check );
extern void routing_buffer_free( route_info_t *b );
extern route_info_t* routing_buffer_get( portTickType blocktime );
extern child_status_type_t check_child_role(addrtype_t type, address_t address);
#endif

#endif /*NEIGHBOR_ROUTING_TABLE_H*/
