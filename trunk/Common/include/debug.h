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
 * \file debug.h
 * \brief debug API headers.
 *
 *  Debugging functions: Support for string printouts via UART.
 *   
 */


#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdio.h>
#include <progmem.h>
#include "address.h"
#ifdef HAVE_DEBUG

extern void debug_init(uint32_t speed);
extern void debug_close(void);

extern uint8_t debug_buffer[];

/**
 * Print a constant string to the debug port.
 *
 * \param y pointer to the string
 *
 */
/*#define debug(y) debug_constant((prog_char *)PSTR(y))*/
#ifdef IAR
#define debug(y) { static __flash char __local_string[] = y; debug_constant(__local_string); }
#else
#define debug(y) debug_constant(PROGMEM_STRING(y));
#ifndef GCC_AVR_MEGA
#define debug_printf(x, y...) sprintf(debug_buffer, PROGMEM_STRING(x),  ##y); debug_send(debug_buffer,strlen(debug_buffer))
#else
//#define debug_printf(x, y...)
#endif
#endif

#define debug_hex(y) debug_integer(2, 16, y)
#define debug_int(y) debug_integer(6, 10, y)

extern uint8_t *debug_integer(uint8_t width, uint8_t base, int n);
extern int8_t debug_constant(prog_char *s);
extern int8_t debug_put(uint8_t c);
extern int8_t debug_send(uint8_t *buffer, uint8_t length);

extern int16_t debug_read(void);
extern int16_t debug_read_blocking(uint32_t time);
extern void debug_address(sockaddr_t *addr);

extern void debug_dump(uint8_t *buffer, int16_t length);
#else
#define debug(y)
#define debug_init(x)
#define debug_hex(y)
#define debug_int(y)
#define debug_integer(x,y,z)
#define debug_put(y)
#define debug_send(x,y)
#define debug_read(x) -1
#define debug_address(y)
extern int16_t debug_read_blocking(uint32_t time);
#define debug_dump(x,y)
#ifndef IAR
#define debug_printf(x, y...)
#endif
#endif /*DEBUG*/

#endif
