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
 * \file socket.h
 * \brief Protocol stack socket API headers.
 *
 *  Protocol stack socket API definitions
 *   
 */


#ifndef _NS_SOCKET_H
#define _NS_SOCKET_H 
#include "stack.h"
#include "neighbor_routing_table.h"
#ifndef SOCKETS_MAX
/* Max number of sockets, default */
#define SOCKETS_MAX       4     
#endif

#define ICMP_CTRL_PORT 	1
#define CUDP_CTRL_PORT 	2
#define CIPV6_CTRL_PORT	3
#define MAC_CTRL_PORT	4

#define NPING_PORT			254
#define	UDP_ECHO_PORT		7
#define REMOVE_ROUTE		2
#define REMOVE_NEIGHBOUR	2
#define UPDATE_NEIGHBOUR	0
#define ADD_CHILD			1

/** Socket handler type */
typedef portCHAR (*sock_handler_func)(buffer_t *buffer);

/** Socket structure */
typedef struct
{
  uint8_t       protocol;       /*!< protocol ID */
  uint8_t       stack_id;       /*!< protocol ID */
  uint16_t       listen;         /*!< True when enabled for listening */
  uint16_t 			port; 					/*!< Port number */
  sockaddr_t    sa;             /*!< Address structure (destination) */
	xQueueHandle  queue;					/*!< Receive queue, when no callbacks */
  sock_handler_func callback;   /*!< Socket handling callback function */
} socket_t;

#ifdef CELLSENSOR
extern portCHAR init_stack_bus( stack_event_t stack_event, sock_handler_func stack_event_handler);
#else
extern portCHAR stack_service_init( stack_event_t stack_event, sock_handler_func stack_event_handler, uint8_t gateway_discover , gateway_cache_t *gw_table );
#endif
extern void socket_init (void);

extern socket_t *socket(module_id_t protocol,
                sock_handler_func sock_handler);
extern portCHAR socket_close (socket_t * si);

extern portCHAR socket_bind (socket_t * si,
                        sockaddr_t * sa) ;


extern portCHAR socket_connect(socket_t *si, sockaddr_t *dst);

extern socket_t *socket_accept(socket_t *si, sockaddr_t *dst,
                 sock_handler_func sock_handler );

extern portCHAR socket_sendto (socket_t *si, sockaddr_t *dst, buffer_t *buf);

extern buffer_t *socket_read(socket_t * si, uint16_t time);
extern portCHAR socket_write (socket_t *si, buffer_t *buf);

extern socket_t *socket_find (uint16_t port, sockaddr_t *sa);

extern buffer_t *socket_buffer_get(socket_t * si);
extern portCHAR socket_buffer_free(buffer_t *buffer);

extern portCHAR socket_up(buffer_t *b);
#endif /* _NS_SOCKET_H */
