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
 * \file socket.c
 * \brief Protocol stack socket API.
 *
 *  Protocol stack API: socket handling,
 *  buffer management API
 *   
 */


#include <sys/inttypes.h>

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stack.h"
#include "stack.h"
#ifndef SOCKET_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"
#include "module.h"
#include "socket.h"

#include "mac.h"

extern stack_t stacks[];
extern module_t modules[];
extern sockaddr_t mac_long;
portCHAR stack_buffer_push( buffer_t * b);

socket_t        sockets[SOCKETS_MAX];

socket_t *socket_find(uint16_t port, sockaddr_t *sa);
void socket_port_random(socket_t *si);
int8_t socket_id_find(socket_t * si);


/**
 *	Initialize socket API
 */
void socket_init (void) 
{
	uint8_t i;   
	debug("Socket: init.\r\n");

  	for (i = 0; i < SOCKETS_MAX; i++)
	{ /* Init socket array */
    	sockets[i].protocol = 0;   
    	sockets[i].callback = 0;   
    	sockets[i].listen = 0;   
    	sockets[i].queue = 0;
		sockets[i].sa.addr_type = ADDR_NONE;   
	}
}


/**
 *	Create a socket. 
 *
 *	\param protocol 			protocol to use
 *	\param sock_handler		socket receive handler
 *
 *  \return               pointer to new socket
 */
socket_t *socket(module_id_t protocol,
                 sock_handler_func sock_handler )
{
	uint8_t i,j, temp=0;
	socket_t *retvalue = 0;
	int8_t stack = -1;
	module_t *module;
	i = 0;
	while ((stacks[i].layers > 0) && (stack < 0) )
	{
	
		for (j=0; j<stacks[i].layers; j++)
		{

			if (stacks[i].module[j] == protocol)
			{
				stack = i;
				temp=j;
				j = stacks[i].layers;
				debug("Found stack: ");
				debug_int(i);				
				debug("\r\n");
			}
		}
		i++;
	}
	
  for (i = 0; i < SOCKETS_MAX; i++)
	{
		if (sockets[i].protocol == 0) break;
	}
	
	if ((stack >= 0) && (i < SOCKETS_MAX))
	{
		retvalue = &(sockets[i]);
		retvalue->sa.addr_type = ADDR_NONE;
		retvalue->stack_id = stack;
		retvalue->sa.port = 0;		
		retvalue->port = 0;
		retvalue->listen = 0;
		retvalue->protocol = protocol;
		retvalue->callback = sock_handler;
		if ( (retvalue->queue == 0) && (sock_handler == 0) )
		{ 
			retvalue->queue = xQueueCreate( 8, sizeof( buffer_t * ) );
			debug("socket: Queue created.\r\n");
		}
		module = module_get(stacks[stack].module[temp]);
		if (module)
		{
			debug("socket: module ");
			debug_int(module->id);
			debug(" type ");
			debug_int(module->addr_type);
			debug(".\r\n");
		}
		else
		{
			debug("socket: no module ");
			debug_int(stacks[stack].module[i]);
			debug(".\r\n");
		}

		if (module && (module->addr_type != ADDR_NONE))
		{
			retvalue->sa.addr_type = module->addr_type;
			i = stacks[stack].layers;
		}
	}
	
	return retvalue;
}

/**
 *	Free a socket.
 *
 *	\param si 	pointer to socket
 *
 *  \todo   handle private protocol pointers -> closing sequence etc.
 *
 *	\return pdTRUE
 *	\return pdFAIL (caused by invalid socket pointer)
 */
portCHAR socket_close (socket_t * si)
{
	int8_t i;

	i = socket_id_find(si);
  if (i >= 0)
	{
  	si->protocol = 0;
	  return pdTRUE;
	}
	return pdFAIL;
}

/**
 *	Bind socket to a specific port
 *
 *	\param si 		pointer to socket
 *  \param sa 	address to bind to
 *
 *  \todo 			support for address types beside nanoIP
 *
 *	\return pdTRUE
 *	\return pdFAIL  (port out of range or invalid socket pointer)
 */
portCHAR socket_bind (socket_t * si,               /* Socket to bind to */
                        sockaddr_t * sa)         /* Port to bind */
{
  if (((sa->port == 0) || (sa->port == NPING_PORT)) || ((sa->port > 0xF0BF)))     /* 254 reserved for PING */
	{ 
		debug("socket_bind: Illegal port.\r\n");
		return pdFAIL;
  }

  if ((si == NULL) || (si->listen != 0))
  {     /* Socket not available*/
		debug("socket_bind: Already reserved.\r\n");
    return pdFAIL;
  }
	
	if (sa->addr_type != ADDR_NONE)
	{
		if ((si->sa.addr_type == ADDR_NONE) || (sa->addr_type == si->sa.addr_type))
		{
			memcpy(&(si->sa), sa, sizeof(sa));
		}
		else
		{
			debug("socket_bind: Address type:");
			debug_int(sa->addr_type);
			debug("socket type:");
			debug_int(si->sa.addr_type);
			debug(".\r\n");
			return pdFAIL;
		}
	}
	else	/** Control-message socket */
	{	/** Now core detech control-messgae */
		uint8_t stack_numbers = stack_number_get();
		si->stack_id = stack_numbers;
		debug("\r\n Socket type: control-message");
	}

  	si->listen = sa->port;
	si->port = sa->port;
	
	debug("Bind OK for port");
	debug_int(si->listen);
	debug(".\r\n");
  return pdTRUE;
}


/**
 *	Find socket pointer per id (find out if it is valid)
 *
 *	\param si 		pointer to socket
 *
 *	\return socket id
 *	\return -1    not found
 */
int8_t socket_id_find(socket_t * si)
{
	uint8_t i;

  for (i = 0; i < SOCKETS_MAX; i++)
	{
		if ( si == &(sockets[i]) ) break;
	}
	if (i < SOCKETS_MAX)
	{
	  return i;
	}
	return -1;
}


/**
 *	Find socket pointer by port/address pair
 *
 *	\param port   port number
 *	\param sa 		address
 *
 *	\return pointer to socket
 */
socket_t *socket_find (uint16_t port, sockaddr_t *sa)             /* Port to check */
{
	uint8_t i;
	socket_t *sp = 0;
	uint8_t addr_size;
	
	if ((sa->port == 0) || (port == 0)) return 0;

	switch (sa->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
			addr_size = 8;
			break;
		case ADDR_802_15_4_LONG:
			addr_size = 8;
			break;
		case ADDR_802_15_4_PAN_SHORT:
			addr_size = 4;
			break;
		case ADDR_802_15_4_SHORT:
			addr_size = 2;
			break;
		case ADDR_NONE:
		default:
			addr_size = 0;
	}	
		
	/* look for connected socket */
	for (i = 0; i < SOCKETS_MAX; i++)
	{
  	if ((sockets[i].port == port))
		{
			if(sockets[i].sa.port == sa->port)
			{	
			  debug("Found port pair.\r\n"); 
				if ( (addr_size == 0) || (memcmp(sockets[i].sa.address, sa->address, addr_size) == 0) )
				{ sp = ((socket_t *) &sockets[i]);
				  debug("Found address match.\r\n");
					break;
				}
			}
		}
	}

	/* look for listen socket */
	if (sp == 0)
	{
		for (i = 0; i < SOCKETS_MAX; i++)
		{
	  		if ((sockets[i].listen == port) )
			{ sp = ((socket_t *) &sockets[i]);
				break;
			}
		}
	}
	if (sp != 0)
	{
		debug("Found socket.\r\n");
	}
	return sp;
}

/**
 *	Allocate a buffer per socket id.
 *
 *	\param si 		pointer to socket
 *
 *	\return pointer to buffer, 0 if none available
 */
buffer_t *socket_buffer_get(socket_t * si) 
{	 
	buffer_t *buffer = 0;
	//uint8_t i;
	//uint16_t ptr;
	
  if (/*(si->stack_id >= 0) && */(buffer=stack_buffer_get(20)) )
  { /*not a listen socket and we got the buffer*/
		
		//ptr = 0;
		/*for (i=0; i< stacks[si->stack_id].layers; i++)
		{
			ptr += module_get(stacks[si->stack_id].module[i])->hdr_size;
		}
		buffer->buf_ptr = ptr;
		buffer->buf_end = ptr;*/
		buffer->socket = (void *) si;
		buffer->from = MODULE_APP;
		buffer->to = MODULE_NONE;
		buffer->dir = BUFFER_DOWN;
		buffer->options.rf_dbm = 0;
		buffer->options.rf_lqi = 0;
		buffer->options.lowpan_compressed=0;
		buffer->options.type = BUFFER_DATA;
		memcpy(&(buffer->dst_sa), &(si->sa), sizeof(sockaddr_t));
		memcpy(&(buffer->src_sa), &(mac_long), sizeof(sockaddr_t));		
  } 
  return buffer;
}

/**
 *	Free a buffer.
 *
 *	\param buffer 		pointer to buffer
 *
 *	\return pdTRUE
 */

portCHAR socket_buffer_free(buffer_t *buffer) 
{
	stack_buffer_free(buffer);
  return pdTRUE;
}

/**
 *	Send a single packet to specified destination.
 *
 *	\param si				pointer to socket
 *  \param dst 		pointer to destination address
 *  \param buf		pointer to buffer
 *
 *	\return pdTRUE
 *	\return pdFAIL
 */
 
portCHAR socket_sendto (socket_t *si, sockaddr_t *dst, buffer_t *buf)
{
	portCHAR retvalue = pdFAIL;
	uint16_t src_port;
	
	if ( socket_id_find(si) < 0)
	{
		debug("Sendto: Invalid socket.\r\n");
		return pdFAIL;
	}
	if (si->stack_id == stack_number_get())
	{
		debug("\r\n Socket type: control-message");
		buf->options.type = BUFFER_CONTROL;
		buf->to = (module_id_t) si->protocol;
		buf->dir = BUFFER_DOWN;
		buf->from = MODULE_APP;
		retvalue = stack_buffer_push(buf);
		return retvalue;
	}

	src_port = 0;
	if (si->listen != 0)
	{
		src_port = si->listen;
	}
	else
	{
		src_port = si->port;
		if (src_port == 0)
		{
			socket_port_random(si);
			debug("Sendto: Get random port");
			debug_int(si->port);
			debug(".\r\n");
			src_port = si->port;
		}
	}
	
	buf->socket = (void *) si;
	if (buf->src_sa.addr_type == ADDR_NONE)
	{
		memcpy(&(buf->src_sa.address), &(mac_long.address), 8);
	}
	buf->src_sa.port = src_port;
	buf->from = MODULE_APP;
	buf->dir = BUFFER_DOWN;
	buf->to = MODULE_NONE;

	if (dst)
	{
		retvalue = pdTRUE;
		memcpy(&(buf->dst_sa), dst, sizeof(sockaddr_t));
	}
	else 
	{
		debug("Sendto: dst = 0\r\n");
		retvalue = pdFAIL;	/*No destination*/
	}
	if (retvalue == pdTRUE)
	{
			retvalue = stack_buffer_push(buf);
	}
	else
	{
			retvalue = pdFALSE;
	}
	return retvalue;
}

/**
 *	Send data using a connected socket.
 *
 *	\param si				pointer to socket
 *  \param buf 		pointer to buffer
 *
 *	\return pdTRUE
 *  \return pdFALSE
 */

portCHAR socket_write (socket_t *si, buffer_t *buf)
{	
	return socket_sendto(si, &(si->sa), buf);
}

/**
 *	Connect socket to specified destination.
 *
 *	\param si			pointer to socket
 *  \param dst 		pointer to destination address
 *
 *  \todo 				handling of private structures
 *
 *	\return pdTRUE
 *  \return pdFALSE
 */
 
portCHAR socket_connect(socket_t *si, sockaddr_t *dst)
{
	memcpy(&(si->sa), dst, sizeof(sockaddr_t));
	si->listen = 0;
	if (si->port == 0)
	{
		socket_port_random(si);
		debug("socket_connect: random port");
		debug_int(si->port);
		debug(".\r\n");
	}
	
	return pdTRUE;
}


/**
 *	Accept new connection from specified destination.
 *
 *	\param si				    pointer to socket
 *  \param dst 		      pointer to destination address
 *  \param sock_handler pointer to receive handler
 *
 *	\return pointer to new socket
 *
 */
socket_t *socket_accept(socket_t *si, sockaddr_t *dst,
                 sock_handler_func sock_handler )
{
	socket_t *new_si = 0;

	new_si = socket(si->protocol, sock_handler);
	if (new_si)
	{
		new_si->port = si->port;
		memcpy(&(new_si->sa), dst, sizeof(sockaddr_t));
		new_si->listen = 0;
	}
	return new_si;	
}

/**
 *	Get a "random" free port for socket.
 *
 *	\param si				    pointer to socket
 *
 */
void socket_port_random(socket_t *si)
{
	uint16_t new_port = 253;
	uint8_t i = 0;
	uint8_t found = 0;
	while (!found)
	{
		found = 1;
		for (i=0; i<SOCKETS_MAX; i++)
		{
			if ( ((sockets[i].port) == new_port) || ((sockets[i].listen) == new_port) )
			{
				new_port--;
				found = 0;
				i = SOCKETS_MAX;
			}
		}
	}
	
	si->port = new_port;
}

/**
 *	Read a socket. This is used when handler callbacks are not used.
 *
 *	\param si 	pointer to socket
 *  \param time time to wait
 *
 *	\return pointer to buffer
 *	\return 0 (no buffer)
 */

buffer_t *socket_read(socket_t * si, uint16_t time)
{
	int8_t i;
	buffer_t *b;
	
	i = socket_id_find(si);
  	if (i >= 0)
	{
		if ( (si->queue) && 
				 (xQueueReceive(si->queue, &(b), time / portTICK_RATE_MS) == pdTRUE) )
		{
	  	return b;
		}
	}
	return 0;
}

/**
 *	Pass a buffer from stack to a socket.
 *
 *	\param b 	pointer to buffer
 *
 *  \return pdTRUE socket found
 *  \return pdFALSE no socket
 */

portCHAR socket_up(buffer_t *b)
{
	socket_t *psocket;

	if (b->socket == 0)
		psocket = socket_find(b->dst_sa.port, &(b->src_sa));
	else 
		psocket = (socket_t *) b->socket;

	if (psocket != 0)
	{
		if  ((psocket->callback) != 0)
		{
			debug("->CB\r\n");
			psocket->callback((void *)b);
			return pdTRUE;
		}
		else if ( (psocket->queue != 0) && (xQueueSend( psocket->queue, ( void * ) &b, ( portTickType ) 0 )==pdTRUE) )
		{
			debug("->Q\r\n");
			return pdTRUE;
		}
	}
	return pdFALSE;
}
