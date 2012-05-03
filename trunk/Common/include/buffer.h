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
 * \file buffer.h
 * \brief buffer type definitions.
 *
 *  nanoStack: buffer carrier structure.
 *   
 */


#ifndef _NS_BUFFER_H
#define _NS_BUFFER_H
#include "address.h"
#include "module_id.h"

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 128
#endif

#ifndef ACK_BUFFER_SIZE
#define ACK_BUFFER_SIZE 5
#endif

/** buffer types */
typedef enum buffer_type_t
{
	BUFFER_DATA = 0,
	BUFFER_CONTROL
}buffer_type_t;

/** handle types */
typedef enum handle_t
{
	HANDLE_TX 	  			= 0,
	HANDLE_PEND_TX 	  		= 1,
	HANDLE_CONTROL 	  		= 2,
	HANDLE_SUPERFRAME 		= 3,
    HANDLE_BROKEN_LINK 		= 4,
	HANDLE_NO_ROUTE_TO_DEST = 5,
	HANDLE_RX				= 6,
	HANDLE_ACK_REQ			= 7,
	HANDLE_ASSOC_REQ		= 8,
	HANDLE_DATA_REQ			= 9,
    HANDLE_DEFAULT	   		= 10,
}handle_t;

/** buffer direction. */
typedef enum buffer_direction_t
{
	BUFFER_UP,
	BUFFER_DOWN
}buffer_direction_t;


/** buffer options, contains signal info etc. */
typedef struct buffer_options_t
{
	buffer_type_t type;					/*!< Shows type of buffer */
	int8_t rf_dbm;						/*!< Signal strength of received packet */
	uint8_t rf_lqi;						/*!< Link quality indication value to Source address */
	handle_t handle_type;				/*!< Show type of handle */
	uint8_t	hop_count;					/*!< Show data packets hop count over network, used with ROUTER DISCOVER */			
	uint8_t	lowpan_compressed;			/*!< Show cUDP modules compression mode*/
}buffer_options_t;

/** buffer structure */
typedef struct
{
  struct socket_t *  socket;                 /*!< Pointer to the socket for outgoing packets */
  sockaddr_t         dst_sa;                 /*!< Destination sockaddr */
  sockaddr_t         src_sa;                 /*!< Source sockaddr */
  module_id_t        from;                   /*!< Layer buffer is coming from */
  module_id_t        to;                     /*!< Next layer */
  buffer_direction_t dir;                     /*!< Up or down */
  uint16_t           buf_ptr;                /*!< Current pointer in the buffer */
  uint16_t           buf_end;                /*!< End pointer in the buffer */
  uint16_t	     size;						/*!< Buffer size */
  buffer_options_t   options;                /*!< Additional signal info etc */
  uint8_t            buf[2];       			/*!< Buffer pointer */
} buffer_t;


/** get pointer to data*/
#define buffer_data_pointer(x)  &(x->buf[x->buf_ptr])

/** get pointer to end of data*/
#define buffer_data_end(x)  &(x->buf[x->buf_end])

/** get data length*/
#define buffer_data_length(x)  (x->buf_end - x->buf_ptr)

/** free data bytes in buffer */
#define buffer_data_free(x)  (x->size - x->buf_end)

/** append 1 byte to data*/
#define buffer_push_uint8(x, z)  x->buf[x->buf_end++] = z

/** read 1 byte out of the buffer*/
#define buffer_pull_uint8(x)  x->buf[x->buf_ptr++]

#define buffer_headroom(x,y) stack_buffer_headroom(x,y)

#endif
