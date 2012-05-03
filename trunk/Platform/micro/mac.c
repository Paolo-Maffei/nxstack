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
 * \file mac.c
 * \brief MAC API.
 *
 *  Support library: getting HW MAC from device.
 *   
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>
#include "string.h"

#include "debug.h"
#include "bus.h"
#include "stack.h"
#include "mac.h"
#include "rf_802_15_4.h"

#ifdef HAVE_802_15_4_RAW
#define HAVE_RF_MAC_GET
#else
#ifdef HAVE_RF_802_15_4
#define HAVE_RF_MAC_GET
#else
#undef HAVE_RF_MAC_GET
#endif
#endif

sockaddr_t mac_short = { ADDR_NONE, { 0,0,0,0, 0,0,0,0, 0,0 }, 0};
sockaddr_t mac_long = { ADDR_NONE, { 0,0,0,0, 0,0,0,0, 0,0}, 0};

portCHAR mac_get(sockaddr_t *address)
{
	switch(address->addr_type)
	{
		case ADDR_802_15_4_PAN_SHORT:
		case ADDR_802_15_4_SHORT:
			memcpy(address, &(mac_short), sizeof(sockaddr_t));
			break;

		case ADDR_802_15_4_PAN_LONG:
		case ADDR_802_15_4_LONG:
		default:
			memcpy(address, &(mac_long), sizeof(sockaddr_t));
			break;
	}
	return pdTRUE;
}

void mac_set(sockaddr_t *address)
{
	switch(address->addr_type)
	{
		case ADDR_802_15_4_PAN_SHORT:
		case ADDR_802_15_4_SHORT:
			memcpy(&(mac_short), address,  sizeof(sockaddr_t));
#ifdef HAVE_RF_802_15_4
			mac_set_mac_pib_parameter(mac_short.address, MAC_SHORT_ADDRESS);
#endif
			break;

		case ADDR_802_15_4_PAN_LONG:
		case ADDR_802_15_4_LONG:
		default:
			memcpy(&(mac_long), address, sizeof(sockaddr_t));
#ifdef HAVE_RF_802_15_4
			mac_set_mac_pib_parameter(mac_long.address, MAC_IEEE_ADDRESS);
#endif
#ifdef HAVE_CIPV6
			rf_802_15_4_ip_layer_address_mode_set(0);
#endif
			break;
	}
}
