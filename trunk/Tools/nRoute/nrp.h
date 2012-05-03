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


#define NRP_H

#define NRP_VERSION 0

#define ADDR_IEEE_48BIT_HW 0x00
#define ADDR_IEEE_802_15_4_DEV_LONG 0x01
#define ADDR_IEEE_802_15_4_PAN_ID 0x02
#define ADDR_IEEE_802_15_4_SHORT 0x03
#define ADDR_IPV4_STRING 0x10
#define ADDR_IPV4_NUMERIC 0x11
#define ADDR_IPV6_NUMERIC 0x12
#define ADDR_15_4_PAN_AND_SHORT 0x80
#define ADDR_UNDEFINED 0xff

#define PROTO_IEEE_802_15_4_RAW 0x00
#define PROTO_NUDP 0x01
#define PROTO_6LOWPAN 0x02
#define PROTO_ZIGBEE 0x03
#define PROTO_6LOWPAN_ICMP 0x04

#define PROTO_UNDEFINED 0xff

#define PORT_UNDEFINED 0xffff

/**	The nRP protocol lookup table.
 *
 *	This table is used to have a relation between the nRP internal protocol numbering and their humanreadable string representations.
 *	The maximum amount of different protocols is at the moment 32 and the maximum length of each string representation is 63 characters.
 *
 */
unsigned char nrp_proto_table[32][64];

/** The nRP address type lookup table.
 *
 *	This table contains the "mappings" between the nRP address type identifiers (one octet values) and their names.
 *
 */
unsigned char nrp_addr_table[32][128];

/* Some useful macros: */

#define CHECK_NRP_HDR(x) ( x[0] == 0x4E && x[1] == 0x52 && x[2] == 0x50 && ((x[3])>>4) == NRP_VERSION ) ? 1 : -1

#define CHECK_8_BYTE_BCAST(x) ( x[0] == 0xFF && x[1] == 0xFF && x[2] == 0xFF && x[3] == 0xFF && x[4] == 0xFF && x[5] == 0xFF && x[6] == 0xFF && x[7] == 0xFF ) ? 1 : 0
