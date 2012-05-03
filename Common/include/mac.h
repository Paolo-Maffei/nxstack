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
 * \file mac.h
 * \brief MAC API.
 *
 *  Function headers: getting HW MAC from device.
 *   
 */


#ifndef _MAC_H
#define _MAC_H

/** Set the unique device ID.
  * \param address Device ID information in the address field
	* \return pdTRUE or pdFALSE (no storage/not editable)
	*/
extern void mac_set(sockaddr_t *address);

/** Get the unique device ID.
  * \param address Device ID information in the address field on pdTRUE
	* \return pdTRUE or pdFALSE (no storage/not found/corrupted)
	*/
extern portCHAR mac_get(sockaddr_t *address);

#ifdef HAVE_DEBUG
/** Read device ID from the debug interface.
  * Requires the DEBUG preprocessor flag.
  * \param sa Device ID information in the address field on pdTRUE
	* \return pdTRUE or pdFALSE (invalid format/cancelled)
	*/
extern portCHAR mac_read(sockaddr_t *sa);
#endif

#endif

