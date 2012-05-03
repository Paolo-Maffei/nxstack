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
 * \file address.h
 * \brief address type definitions.
 *
 *  nanoStack: supported address types and associated data structures.
 *   
 *	
 */


#ifndef _NS_ADDRESS_H
#define _NS_ADDRESS_H

#include <sys/inttypes.h>

#define ADDR_SIZE 10
/** Address types */
typedef enum
{
  ADDR_NONE,										/*!< No address */
  ADDR_802_15_4_LONG,						/*!< 64-bit 802.15.4 address */
  ADDR_802_15_4_SHORT,					/*!< 16-bit 802.15.4 address */
  ADDR_802_15_4_PAN_SHORT,			/*!< 16-bit PAN with 16-bit 802.15.4 address */
  ADDR_802_15_4_PAN_LONG,				/*!< 16-bit PAN with 64-bit 802.15.4 address */
  ADDR_SHORT,										/*!< 8-bit address in octet 0 */
  ADDR_PAN,											/*!< 16-bit PAN-id */
  ADDR_DATA,										/*!< Attribute-based data-centric query */
  ADDR_BROADCAST,								/*!< Broadcast address */ 
  ADDR_COORDINATOR							/*!< Coordinator address only for Beacon enable mode*/
} addrtype_t;										/*!< Address types in the stack */

/** Address field */
typedef uint8_t address_t[ADDR_SIZE];


typedef uint8_t ipv6_address_t[16];
/** Address structure */
typedef struct
{
  addrtype_t    addr_type;              /*!< Type of address */
  address_t     address;                /*!< Source or destination address */
  uint16_t      port;                   /*!< Source or destination port */
} sockaddr_t;

#endif /*_NS_ADDRESS_H*/
