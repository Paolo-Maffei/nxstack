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
 * \file compass.h
 * \brief micro.compass module driver.
 *
 *  Micro: micro.compass module driver headers.
 *   
 */


#ifndef _COMPASS_H
#define _COMPASS_H

typedef enum
{
	COMPASS_SHUTDOWN,
	COMPASS_AUTO,
	COMPASS_MAG,
	COMPASS_ACC,
	COMPASS_ERROR
}compass_mode_t;

extern portCHAR compass_select(void);
extern void compass_unselect(void);
extern compass_mode_t compass_mode(compass_mode_t mode);
extern portCHAR compass_mag_interval(uint8_t interval);
extern portCHAR compass_accel_interval(uint8_t interval);
extern portCHAR compass_accel_range(uint8_t range);
extern portCHAR compass_status(uint8_t *status);
extern portCHAR compass_mag_read(int16_t *value);
extern portCHAR compass_accel_read(int16_t *value);
extern portCHAR compass_mag_read_single(int16_t *value);
extern portCHAR compass_accel_read_single(int16_t *value);

#endif /*_COMPASS_H*/
