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
 * \file flash.c
 * \brief micro external flash access.
 *
 *  Micro flash: support functions.
 *  Module selection, block read/write functions.
 *   
 *	
 */

 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>

#include "debug.h"
#include "bus.h"
#include "flash.h"

/*These macros control flash select signal and assume correct bus state*/
#define FLASH_SELECT(x) P3OUT &= ~0x01
#define FLASH_UNSELECT(x) P3OUT |= 0x01

/*Flash control commands*/
#define M25P_WREN 0x06
#define M25P_WRDI 0x04
#define M25P_RDID 0x9F
#define M25P_RDSR 0x05
#define M25P_WRSR 0x01
#define M25P_READ 0x03
#define M25P_FAST_READ 0x0B
#define M25P_PP   0x02
#define M25P_SE   0xD8
#define M25P_BE   0xC7
#define M25P_DP   0xB9
#define M25P_RES  0xAB

/**
 * Read flash block.
 *
 *
 * \param address block address on flash
 * \param buffer  pointer to buffer
 * \param length  length of read buffer
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free or init failed
 */
portCHAR flash_read(uint32_t address, uint8_t *buffer, uint16_t length)
{
	uint16_t i;
	uint8_t *p = buffer;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return pdFALSE;
	}
	bus_spi_exchange(M25P_READ);
	bus_spi_exchange(address >> 16);
	bus_spi_exchange(address >> 8);
	bus_spi_exchange(address);
	for (i = 0; i< length ; i++)
	{
		*p++ = bus_spi_exchange(0);
	}
	bus_free();
	return pdTRUE;
}

/**
 * Write flash page. Page size is 256 bytes.
 * Write address must point at start of page.
 *
 * \param address block address on flash
 * \param buffer  pointer to buffer
 * \param length  length of read buffer
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free or address not valid (must be at start of page)
 */


portCHAR flash_write(uint32_t address, uint8_t *buffer, uint16_t length)
{
	uint16_t i;
	
	if ((address & 0xFF) != 0) return pdFALSE;
	if ((length > 256) || (length == 0)) return pdFALSE;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return pdFALSE;
	}
	bus_spi_exchange(M25P_WREN);
	FLASH_UNSELECT();
	pause_us(10);
	FLASH_SELECT();
	bus_spi_exchange(M25P_PP);
	bus_spi_exchange(address >> 16);
	bus_spi_exchange(address >> 8);
	bus_spi_exchange(address);
	
	for (i = 0; i< length ; i++)
	{
		bus_spi_exchange(buffer[i]);
	}
	bus_free();
	
	return pdTRUE;
}

/**
 * Flash sector erase. Sector size is 64 kilobytes.
 * Address must point at start of sector.
 *
 * \param address block address on flash
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free or address not valid (must be at start of sector)
 */
portCHAR flash_erase_sector(uint32_t address)
{
	if ((address & 0xFFFF) != 0) return pdFALSE;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return pdFALSE;
	}
	bus_spi_exchange(M25P_WREN);
	FLASH_UNSELECT();
	pause_us(10);
	FLASH_SELECT();
	bus_spi_exchange(M25P_SE);
	bus_spi_exchange(address >> 16);
	bus_spi_exchange(address >> 8);
	bus_spi_exchange(address);
	
	bus_free();
	
	return pdTRUE;
}

/**
 * Flash signature read. Wakes the device up from power down mode.
 * Signature value should be 0x12 if the flash is present and working.
 *
 * \return signature value
 * \return -1	bus not free
 */
int16_t flash_signature_read(void)
{
	int16_t retval;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return -1;
	}
	bus_spi_exchange(M25P_RES);
	bus_spi_exchange(0);
	bus_spi_exchange(0);
	bus_spi_exchange(0);
	retval = (int16_t)bus_spi_exchange(0);
	
	bus_free();
	
	return (retval & 0xFF);
}

/**
 * Flash status read. 
 *
 * \return status register value
 * \return -1	bus not free
 */
int16_t flash_status_read(void)
{
	int16_t retval;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return -1;
	}
	bus_spi_exchange(M25P_RDSR);
	retval = (int16_t) bus_spi_exchange(M25P_RDSR);
	
	bus_free();
	
	return (retval & 0xFF);
}

/**
 * Wait flash operation to complete. 
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR flash_write_wait(void)
{
	uint8_t status;
	
	if (bus_select(1, BUS_SPI, BUS_CLOCK_INVERT) == pdFALSE)
	{
		return pdFALSE;
	}
	bus_spi_exchange(M25P_RDSR);
	do 
	{
		status = (int16_t) bus_spi_exchange(M25P_RDSR);
	}while (status & FLASH_SR_WIP);
	
	bus_free();
	
	return pdTRUE;
}
