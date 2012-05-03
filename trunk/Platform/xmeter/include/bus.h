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
 * \file bus.h
 * \brief micro.bus control headers.
 *
 *  Micro bus: mode control and support function headers.
 *   
 *	
 */


#ifndef _MICRO_BUS_H
#define _MICRO_BUS_H

typedef enum 
{
	BUS_GPIO = 0x00,
	BUS_SPI = 0x01,
	BUS_I2C = 0x02,
	BUS_UART = 0x04,
	BUS_PARALLEL = 0x08,
	BUS_1WIRE = 0x10
}bus_mode; 

typedef enum 
{
	BUS_SPI_DEFAULT = 0x00,
	BUS_STE = 0x01,
	BUS_PHASE_INVERT = 0x02,
	BUS_CLOCK_INVERT = 0x04,
	BUS_MULTIMASTER = 0x08,
	BUS_STE_IN = 0x10,
	BUS_CLOCK_115kHZ = 0x20,
	BUS_CLOCK_1MHZ = 0x40,
	BUS_CLOCK_4MHZ = 0x60,
	BUS_SPI_SLAVE = 0x80
}bus_spi_flags;

extern portCHAR bus_init(void);
extern portCHAR bus_select(uint8_t id, bus_mode mode, uint8_t flags);
extern portCHAR bus_free(void);

extern uint8_t bus_spi_exchange(uint8_t out);

typedef enum 
{
	BUS_I2C_DEFAULT = 0x00
}bus_i2c_flags;

typedef enum 
{
	BUS_UART_9600 = 0x00,
	BUS_UART_19200 = 0x01,
	BUS_UART_28800 = 0x02,
	BUS_UART_115200 = 0x03,	
	BUS_UART_230400 = 0x04,	
	BUS_UART_921600 = 0x05,	
	BUS_UART_PARITY_ODD = 0x40,
	BUS_UART_PARITY_EVEN = 0x80
}bus_uart_flags;

extern portCHAR bus_uart_put( uint8_t byte, portTickType blocktime );
extern portCHAR bus_uart_get( uint8_t *byte, portTickType blocktime );
	
extern void pause (uint16_t time);
extern void pause_us (uint16_t time);
extern uint16_t random_generate(uint16_t range);

#endif /*_MICRO_BUS_H*/
