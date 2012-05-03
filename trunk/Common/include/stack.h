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
 * \file stack.h
 * \brief Protocol stack main module headers.
 *
 *  Protocol stack main module definitions
 *   
 */

 
#ifndef _NS_STACK_H
#define _NS_STACK_H 

#include "address.h"
#include "buffer.h"
#include "module.h"
#include "rf_802_15_4.h"
#include "neighbor_routing_table.h"

#ifndef LAYERS_MAX
#define LAYERS_MAX 4
#endif

typedef xQueueHandle stack_event_t;

typedef struct
{
  uint8_t       layers;                 /*!< Number of layers in the stack */
  module_id_t   module[LAYERS_MAX];     /*!< Modules in the stack, from up down*/
} stack_t;



/** Device logical type */ 
typedef enum
{
	AD_HOC_DEVICE					= 0,	
	BEACON_ENABLE_CLIENT  			= 1,	
	BEACON_ENABLE_COORDINATOR		= 2,
	BEACON_ENABLE_GATEWAY		    = 3,
	DEFAULT_MODE					= 4	
}log_dev_type_t;


/** Stack start functions return values */ 
typedef enum
{
	START_SUCCESS			= 0,	
	TYPE_NOT_SUPPORTED  	= 1,	
	CHANNEL_NOT_SUPPORTED	= 2,
    STACK_INIT_FAILED		= 3
}start_status_t;
/** Stack start functions init structure */ 
typedef struct
{
  log_dev_type_t	type;					/*!< AD_HOC_DEVICE / BEACON_ENABLE_CLIENT / BEACON_ENABLE_COORDINATOR / BEACON_ENABLE_ROUTER. */
  uint8_t      	 	channel;				/*!< Routers and coordinattors use this only. */
  uint8_t			pan_id[2];				/*!< Routers and coordinattors use this only. */
  uint8_t			short_address[2];		/*!< Routers and coordinattors use this only. */
  uint8_t			use_sw_mac;				/*!< 1=use software MAC address, 0= use hardware MAC address. */
  uint8_t			mac_address[8];			/*!< Software MAC defined by user. */
  uint8_t			pending_ttl_time;		/*!< Coordinator use this parameter to detect max pending time (pending_ttl_time * 15 seconds period). */
} stack_init_t;

extern portCHAR stack_init(void);
extern start_status_t stack_start(stack_init_t  *stack_parameters);
extern buffer_t * waiting_stack_event(uint16_t time);
extern stack_event_t open_stack_event_bus(void);

extern buffer_t* stack_buffer_get ( portTickType blocktime );
extern void stack_buffer_free ( buffer_t *b );
extern portCHAR stack_buffer_push( buffer_t * b);
extern portCHAR stack_buffer_headroom( buffer_t * b, uint16_t size);
extern uint8_t * stack_insert_address_to_buffer(uint8_t *dptr, addrtype_t type, address_t address);
extern void cudp_compress_mode( uint8_t mode );
extern void cipv6_compress_mode( uint8_t mode );
extern void nwk_manager_set_pan_id(uint8_t *pointer);
extern portCHAR stack_check_broadcast(address_t address, addrtype_t type);

extern portCHAR udp_echo(sockaddr_t *dst, discover_res_t  *result_ptr);
extern portCHAR ping(sockaddr_t *dst , discover_res_t  *result_ptr);
extern void push_to_app(buffer_t *buf);
extern portCHAR parse_echo_response(buffer_t *buf);
extern void stop_ping(void);

extern void nwk_manager_launch(void);
extern void scan_network(void);

extern portCHAR gw_discover(void);
extern portCHAR gw_advertisment(void);
extern portCHAR remove_gw_info(sockaddr_t *adr);
extern portCHAR update_gw_info_ttl(void);
extern portCHAR select_best_gw(sockaddr_t *adr);
extern portCHAR gw_table_update(buffer_t *buf);
typedef struct event_t
{
	void (*process)(void *param);
	void *param;
}event_t;

extern xQueueHandle events; 
extern uint8_t stack_number_get(void);

extern portCHAR stack_compare_address(sockaddr_t *a1, sockaddr_t *a2);
#ifdef HAVE_DYNAMIC_BUFFERS
buffer_t* stack_buffer_allocate( uint8_t size );
#endif
#endif /*_NS_STACK_H*/

