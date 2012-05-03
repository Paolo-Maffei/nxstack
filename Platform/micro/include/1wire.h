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
 * \file 1wire.h
 * \brief micro.bus 1-wire control headers.
 *
 *  Micro bus: 1-wire mode support function headers.
 *   
 */


#ifndef _BUS_1WIRE_H
#define _BUS_1WIRE_H

typedef uint8_t b1w_reg[8];

extern portCHAR bus_1wire_reset(void);
extern uint8_t bus_1wire_read(void);
extern void bus_1wire_write(uint8_t byte);

extern uint8_t bus_1wire_search(b1w_reg *device, uint8_t num_id);
extern portCHAR bus_1wire_select(b1w_reg device);
extern portCHAR bus_1wire_read_memory(b1w_reg device, uint16_t address, uint8_t *buffer, uint8_t bytes);
extern portCHAR bus_1wire_read_rom(b1w_reg device);
extern void bus_1wire_crc_add(uint8_t *crc, uint8_t byte);

	
#endif /*_BUS_1WIRE_H*/
