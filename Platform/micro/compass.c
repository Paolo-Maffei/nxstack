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
 * \file compass.c
 * \brief micro.compass module driver.
 *
 *  Micro: micro.compass module driver functions.
 *   
 *	
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>

#include "compass.h"
#include "bus.h"
#include "debug.h"

uint8_t compass_state = 0;

#define INTERBYTE_DELAY 10
/**
 * Select compass module.
 *
 * \return pdTRUE module selected
 * \return pdFALSE bus reserved 
 */
portCHAR compass_select(void)
{
	portCHAR retval;
	
	retval = bus_select(3, BUS_SPI, BUS_CLOCK_1MHZ);
	if (retval == pdTRUE) compass_state = 1;
	else compass_state = 0;
	pause(INTERBYTE_DELAY);	
	return retval;
}

/**
 * Unselect compass module.
 *
 */
void compass_unselect(void)
{
	if (compass_state != 0)
	{
		bus_free();
		compass_state = 0;
	}
}
 
/**
 * Select compass mode.
 *
 * \param mode mode to select
 *
 * \return selected mode
 * \return -1 compass is not selected
 */
compass_mode_t compass_mode(compass_mode_t mode)
{
	uint8_t byte;
	uint8_t mode_byte;
	uint8_t count = 0;
	
	if (compass_state == 0) return -1;
	
	switch (mode)
	{
		case COMPASS_AUTO:
			mode_byte = 0xA1;
			break;
		case COMPASS_MAG:
			mode_byte = 0xA2;
			break;
		case COMPASS_ACC:
			mode_byte = 0xA3;
			break;
		default:
		case COMPASS_SHUTDOWN:
			mode_byte = 0xA0;
			break;
	}
	
	do
	{
		pause(INTERBYTE_DELAY);
		byte = bus_spi_exchange(mode_byte);
		
		if (count++ > 200) break;
	}while(byte != (mode_byte));
	
	if (count > 200)
	{
		 return COMPASS_ERROR;
	}
	
	return mode;
}

/**
 * Select magnetometer sampling interval.
 *
 * \param interval interval in milliseconds, 0 to 255
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_mag_interval(uint8_t interval)
{
	uint8_t byte;
	
	if (compass_state == 0) return pdFALSE;
	
	byte = bus_spi_exchange(0xB0);
	byte = bus_spi_exchange(interval);
	
	return pdTRUE;
}

/**
 * Select accelerometer sampling interval.
 *
 * \param interval interval in milliseconds, 0 to 255
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_accel_interval(uint8_t interval)
{
	uint8_t byte;
	
	if (compass_state == 0) return pdFALSE;
	
	byte = bus_spi_exchange(0xB1);
	byte = bus_spi_exchange(interval);
	
	return pdTRUE;
}

/**
 * Select accelerometer range.
 *
 * \param range 0=1.5G, 1=3G, 2=4.5G, 3=6G
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_accel_range(uint8_t range)
{
	if (compass_state == 0) return pdFALSE;
	
	bus_spi_exchange(0xA8 + (range & 3));
	
	return pdTRUE;
}


/**
 * Get sampling status.
 *
 * Note that if the module is powered down, reading status will start mode 1.
 *
 * Status byte low nibble contains magnetometer status,
 * 1 means channel value valid
 *
 * Status byte high nibble contains accelerometer status,
 * 1 means channel value valid
 *
 * Bits 7 and 4 are not used.
 *
 * Summary of status byte:
 * | NA | AX | AY | AZ | NA | MX | MY | MZ |
 *
 * \param status pointer to store return value
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_status(uint8_t *status)
{
	uint8_t byte;
	
	if (compass_state == 0) return pdFALSE;
	
	pause(INTERBYTE_DELAY);
	byte = bus_spi_exchange(0x00);
	pause(INTERBYTE_DELAY);
	byte = bus_spi_exchange(0x00);
	
	*status = byte;
	return pdTRUE;
}

/**
 * Get magnetometer values.
 *
 * \param value pointer to store return values, array of 3 int16_t
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_mag_read(int16_t *value)
{
	uint8_t i;
	
	if (compass_state == 0) return pdFALSE;
	
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0xAD);
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0x00);
	for (i = 0; i<3; i++)
	{
		pause(INTERBYTE_DELAY);
		value[i] = bus_spi_exchange(0x00);
		value[i] <<= 8;
		pause(INTERBYTE_DELAY);
		value[i] += bus_spi_exchange(0x00);
	}
	return pdTRUE;
}


/**
 * Get magnetometer values: start module, 
 * do a single read and shutdown module.
 *
 * \param value pointer to store return values, array of 3 int16_t
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_mag_read_single(int16_t *value)
{
	uint8_t i, status;
	
	if (compass_state == 0) return pdFALSE;
	
	if (compass_mode(COMPASS_MAG) != COMPASS_MAG) return pdFALSE;

	pause(INTERBYTE_DELAY);
	i = 0;
	do
	{
		compass_status(&status);
		if (i++ > 200) return pdFALSE;
	}while ((status & 0x07) != 0x07);
	
	pause(INTERBYTE_DELAY);
	
	bus_spi_exchange(0xAD);
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0x00);
	for (i = 0; i<3; i++)
	{
		pause(INTERBYTE_DELAY);
		value[i] = bus_spi_exchange(0x00);
		value[i] <<= 8;
		pause(INTERBYTE_DELAY);
		value[i] += bus_spi_exchange(0x00);
	}
	pause(INTERBYTE_DELAY);
	compass_mode(COMPASS_SHUTDOWN);
	pause(INTERBYTE_DELAY);

	return pdTRUE;
}

/**
 * Get accelerometer values.
 *
 * \param value pointer to store return values, array of 3 int16_t
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_accel_read(int16_t *value)
{
	uint8_t i;
	
	if (compass_state == 0) return pdFALSE;
	
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0xAE);	
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0x00);
	for (i = 0; i<3; i++)
	{
		pause(INTERBYTE_DELAY);
		value[i] = bus_spi_exchange(0x00);
		value[i] <<= 8;
		pause(INTERBYTE_DELAY);
		value[i] += bus_spi_exchange(0x00);
	}
	return pdTRUE;
}


/**
 * Get accelerometer values: start module, 
 * do a single read and shutdown module.
 *
 * \param value pointer to store return values, array of 3 int16_t
 *
 * \return pdTRUE
 * \return pdFALSE compass is not selected
 */
portCHAR compass_accel_read_single(int16_t *value)
{
	uint8_t i, status;
	
	if (compass_state == 0) return pdFALSE;
	
	if (compass_mode(COMPASS_ACC) != COMPASS_ACC) return pdFALSE;

	i = 0;
	do
	{
		compass_status(&status);
		if (i++ > 200) return pdFALSE;
	}while ((status & 0x70) != 0x70);
	
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0xAE);	
	pause(INTERBYTE_DELAY);
	bus_spi_exchange(0x00);
	
	for (i = 0; i<3; i++)
	{
		pause(INTERBYTE_DELAY);
		value[i] = bus_spi_exchange(0x00);
		value[i] <<= 8;
		pause(INTERBYTE_DELAY);
		value[i] += bus_spi_exchange(0x00);
	}
	compass_mode(COMPASS_SHUTDOWN);

	pause(INTERBYTE_DELAY);
	
	return pdTRUE;
}
