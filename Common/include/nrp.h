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
 * \file nrp.h
 * \brief nRP module definitions.
 *
 *  nRP: the nRoute Protocol module: internal constants and function
 *  headers 
 *   
 */


#ifndef _NRP_H
#define _NRP_H

#define NRP_VERSION (0 << 4)

typedef enum
{
	NRP_DATA = 0x00,
	NRP_CONFIG = 0x01,
	NRP_CONFIG_REPLY = 0x02,
	NRP_ACK = 0x0E,
	NRP_DEBUG = 0x0F
}nrp_type_t;

typedef enum
{
	NRP_IEEE = 0x00,
	NRP_802_15_4_LONG = 0x01,
	NRP_802_15_4_PAN = 0x02,
	NRP_802_15_4_SHORT = 0x03,
	NRP_IP_STRING = 0x10,
	NRP_IPV4 = 0x11,
	NRP_IPV6 = 0x12
}nrp_address_t;

#define NRP_AF_802_15_4 NRP_IEEE
#define NRP_AF_IP NRP_IP_STRING

typedef enum
{
	NRP_PDATA = 0x00,
	NRP_PROTOCOL = 0x01,
	NRP_SRC = 0x02,
	NRP_DST = 0x03,
	NRP_SRC_PORT = 0x04,
	NRP_DST_PORT = 0x05,
	NRP_DBM = 0x06,
	NRP_SEQ = 0x07
}nrp_data_tag_t;

typedef enum
{
	NRPC_VERSION = 0x00,
	NRPC_RF_ID = 0x01,
	NRPC_PROTOCOL_ID = 0x02,
	NRPC_SUBSCRIBE = 0x03,
	NRPC_RESET = 0x07,
	NRPC_CHANNEL_SET = 0x10,
	NRPC_MAC_GET = 0x11
}nrp_config_tag_t;

typedef enum
{
	NRP_PROTO_802_15_4 = 0,
	NRP_PROTO_NUDP = 1,
	NRP_PROTO_6LOWPAN = 2,
	NRP_PROTO_ZIGBEE = 3,
	NRP_PROTO_6LOWPAN_ICMP = 4
}nrp_protocol_t;

#endif /*_NRP_H*/
