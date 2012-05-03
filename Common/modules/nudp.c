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
 * \file     nudp.c
 * \brief    nanoUDP protocol module.
 *
 *  The nanoUDP protocol module: handler functions.
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "debug.h"
#include "stack.h"
#include "buffer.h"
#include "module.h"

/*
[NAME]
NUDP

[ID]
MODULE_NUDP,

[INFO]
#ifdef HAVE_NUDP
  {nudp_init, nudp_handle, nudp_check, 0, MODULE_NUDP, 5, ADDR_802_15_4_PAN_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR nudp_init(buffer_t *buf);
extern portCHAR nudp_handle( buffer_t *buf );
extern portCHAR nudp_check( buffer_t *buf );

#define NUDP_NO_FRAG    	0xc
#define NUDP_FIRST        0x8
#define NUDP_MIDDLE       0x0
#define NUDP_LAST         0x4

/**
 *  Initialize nUDP module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR nudp_init( buffer_t *buf )
{  
#ifdef NUDP_DEBUG
	debug("NUDP: init.\r\n");
#endif
	return pdTRUE;
}

extern stack_t stacks[];

/**
 *  Main nUDP buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR nudp_handle( buffer_t *buf )
{
	uint8_t flags;
  uint16_t ind;
	uint16_t length;
	
#ifdef NUDP_DEBUG
	debug("NUDP: handler.\r\n");
#endif
  
  switch (buf->dir)
	{
		case BUFFER_DOWN:
  		buf->from = MODULE_NUDP;
			buf->to = MODULE_NONE;
			
			flags = NUDP_NO_FRAG;
			if (stack_buffer_headroom(buf, 5) == pdFALSE)
			{
				stack_buffer_free(buf);
				return pdTRUE;
			}
			length = buf->buf_end - buf->buf_ptr; 			//payload length
			buf->buf_ptr -= 5; // Move the buffer pointer
	  		// Fill the header 
			ind = buf->buf_ptr;
			
			buf->buf[ind++] = 0 + (flags & 0x0F); /*Protocol ID and flags*/
#ifdef NUDP_DEBUG
			debug_printf("nUDP: out %d bytes.\r\n", length);
#endif
			buf->buf[ind++] = length >> 8; /*Payload length*/
			buf->buf[ind++] = length; /*Payload length*/
			
			buf->buf[ind++] = buf->src_sa.port;
			buf->buf[ind++] = buf->dst_sa.port;

#ifdef NUDP_DEBUG
			debug_printf("nUDP: from %d to %d.\r\n", buf->src_sa.port,
			                                        buf->dst_sa.port);
#endif
			
	  	// Return the buffer to the queue
		  stack_buffer_push(buf);
			break;
			
		case BUFFER_UP:
			/*Check header*/
		  buf->from = MODULE_NUDP;
			buf->to = MODULE_NONE;
			
			ind = buf->buf_ptr;
			flags = buf->buf[ind++];
			length = buf->buf[ind++];
			length <<= 8;
			length += buf->buf[ind++];
			buf->src_sa.port = buf->buf[ind++];
			buf->dst_sa.port = buf->buf[ind++];

#ifndef HAVE_NRP
			if (buf->dst_sa.port == 254)
			{ /*Ping*/
				if (buf->src_sa.port == 254)
				{ /*Evil loop ping*/
#ifdef NUDP_DEBUG
					debug("nUDP: loop ping.\r\n");
#endif
					stack_buffer_free(buf);
				}
				else
				{ /*respond*/
					uint8_t i;
					
#ifdef NUDP_DEBUG
					debug("nUDP: ping->respond.\r\n");
#endif
					buf->socket = 0;
					
					buf->to = MODULE_NONE;
					buf->from = MODULE_NUDP;
					buf->dir = BUFFER_DOWN;
										
					buf->dst_sa.port = buf->src_sa.port;
					buf->dst_sa.addr_type=buf->src_sa.addr_type;
					buf->src_sa.addr_type = ADDR_NONE;
					
					for (i=0; i < 10;i++)
					{
						flags = buf->dst_sa.address[i];
						buf->dst_sa.address[i] = buf->src_sa.address[i];
						buf->src_sa.address[i] = flags;
					}
					ind = buf->buf_ptr + 3;
					flags = buf->buf[ind];
					buf->buf[ind] = buf->buf[ind+1];
					buf->buf[ind+1] = flags;

	  			/* Return the buffer to the queue */
			  	stack_buffer_push( buf );
				}
			}
			else
#endif /*HAVE_NRP*/
			{ /*normal processing*/
#ifdef NUDP_DEBUG
				debug("nUDP: port ");
				debug_int(buf->dst_sa.port);
				debug(".\r\n");
#endif
			  buf->from = MODULE_NUDP;
				buf->to = MODULE_NONE;
				buf->dir = BUFFER_UP;
  			buf->buf_ptr = ind; // Move the buffer pointer
				stack_buffer_push(buf);
			}
			break;

		default:
#ifdef NUDP_DEBUG
			debug("Evil buffer!");
#endif
			stack_buffer_free(buf);
			break;
	}
  
  return pdTRUE;
}

/**
 *  The nUDP buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE    is nUDP
 *  \return  pdFALSE   is not nUDP or broken header
 */
portCHAR nudp_check( buffer_t *buf )
{
	module_t *module = module_get(MODULE_NUDP);
  uint16_t length;
#ifdef NUDP_DEBUG
	debug("nUDP check: ");
#endif
	length = buf->buf_end - buf->buf_ptr;
	length -= module->hdr_size;
	length -= buf->buf[buf->buf_ptr+1]*256;
	length -= buf->buf[buf->buf_ptr+2];
	if ((length == 0) && ((buf->buf[buf->buf_ptr] & 0xF0) == 0) )
	{
		if ( (buf->buf[buf->buf_ptr] & 0x0F) != NUDP_NO_FRAG)
		{
#ifdef NUDP_DEBUG
			debug("no fragmentation.\r\n");
#endif
		}
		else
		{
#ifdef NUDP_DEBUG
			debug("OK.\r\n");
#endif
			return pdTRUE;
		}
	}
#ifdef NUDP_DEBUG
	debug("not nUDP.\r\n");
#endif
	return pdFALSE;  
}
