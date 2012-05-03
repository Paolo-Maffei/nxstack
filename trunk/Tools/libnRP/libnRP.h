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


#define LIBNRP_H
#define NRP_VERSION 0
#define NRP_SPEC_VERSION 0.8

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>



/**	The nRP protocol lookup table.
 *
 *	This table is used to have a relation between the nRP internal protocol numbering and their humanreadable string representations.
 *	The maximum amount of different protocols is at the moment 32 and the maximum length of each string representation is 63 characters.
 *
 */
static unsigned char libnrp_proto_table[32][64] = {
	{ "IEEE 802.15.4 MAC data" },
	{ "nUDP" },
	{ "6lowpan (IETF draft not yet complete)" },
	{ "ZigBee" }
};

/** The nRP address type lookup table.
 *
 *	This table contains the "mappings" between the nRP address type identifiers (one octet values) and their names.
 *
 */
static unsigned char libnrp_addr_table[32][128] = {
	{ "IEEE hardware identifier (48 bits)" },			// 0x00
	{ "802.15.4 device long address (64 bits)" },		// 0x01
	{ "802.15.4 PAN ID (16 bits)" },					// 0x02
	{ "802.15.4 short address (16 bits)" },				// 0x03
	{ "" },												// 0x04		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x05		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x06		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x07		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x08		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x09		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0A		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0B		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0C		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0D		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0E		DON'T REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0F		DON'T REMOVE THE EMPTY LINES!!!
	{ "IP address (string, 6-14 bytes)" },				// 0x10
	{ "IPv4 address (32 bits)" },						// 0x11
	{ "IPv6 address (128 bits)" }						// 0x12
};


/**	The nRP packet definitions.
 *
 *	These are from the nRoute Protocol specification document version 0.5.
 */
enum libnrp_pkt_types { DATA = 0x00, CONFIG_QUERY = 0x01, CONFIG_REPLY = 0x02, NROUTED_CONFIG = 0x03, MCU_DEBUG = 0x15 };

enum libnrp_message_level {NRP_CRITICAL = 0, NRP_INFO, NRP_DEBUG };

/*	Some useful macros & definitions: */
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
#define PROTO_UNDEFINED 0xff

#define PORT_UNDEFINED 0xffff

#define LIBNRP_CHECK_HDR(x) ( x[0] == 0x4E && x[1] == 0x52 && x[2] == 0x50 && ((x[3])>>4) == NRP_VERSION ) ? 1 : -1

#define LIBNRP_CHECK_8_BYTE_BCAST(x) ( x[0] == 0xFF && x[1] == 0xFF && x[2] == 0xFF && x[3] == 0xFF && x[4] == 0xFF && x[5] == 0xFF && x[6] == 0xFF && x[7] == 0xFF ) ? 1 : 0
#define LIBNRP_CHECK_2_BYTE_BCAST(x) ( x[0] == 0xFF && x[1] == 0xFF) ? 1 : 0

//	THESE DEFINITIONS ARE OBSOLETE AND WILL BE REMOVED SOMETIME IN THE FUTURE. USE THE libnrp_debug() FUNCTION INSTEAD
#define LIBNRP_DEBUG(x,y) (((x) <= (DEBUGLEVEL)) ? (printf(y)) : printf(""))
#define LIBNRP_DEBUG_DVAL(x,y) (x <= DEBUGLEVEL) ? printf(#y "=%f\n", y) :
#define LIBNRP_DEBUG_IVAL(x,y) (x <= DEBUGLEVEL) ? printf(#y "=%d\n", y) :
#define LIBNRP_DEBUG_HEX(x,y) (x <= DEBUGLEVEL) ? printf(#y "=0x%.2x\n", y) :
#define LIBNRP_DEBUG_STR(x,y) (x <= DEBUGLEVEL) ? printf(#y "=%s\n", y) :

/*	And some functions: */
extern void libnrp_debug(int debuglevel, const char *dbg, ...);
extern int libnrp_check_nRoute_reply(unsigned char *reply, int len);
extern int libnrp_create_data_pkt_hdr(unsigned char *buffer, unsigned char *data, unsigned char datalen, unsigned char proto, unsigned char *source_addr, unsigned char *dest_addr, unsigned char d_addr_type, unsigned short int *source_port, unsigned short int dest_port, unsigned short int *pkt_seq);

extern int libnrp_create_conf_pkt(unsigned char *buffer, unsigned char proto, unsigned char *source_addr, unsigned char *dest_addr, unsigned char d_addr_type, unsigned short int *source_port, unsigned short int dest_port);
extern int libnrp_snd_conf(unsigned char *buffer, int pkt_len, char *addr, unsigned int port);

extern int libnrp_get_source_address(unsigned char *buffer, unsigned int buflen, unsigned char *addr_type, unsigned char *address_bytes, char *address_string);
extern int libnrp_get_destination_address(unsigned char *buffer, unsigned int buflen, unsigned char *addr_type, unsigned char *address_bytes, char *address_string);
extern int libnrp_get_data(unsigned char *buffer, unsigned int buflen, unsigned char *data, unsigned int *datalen);

extern int libnrp_parse_nrp_hdr(unsigned char *buffer, unsigned char buflen, unsigned char *data, unsigned char *datalen, unsigned char *proto, unsigned char *source_addr, unsigned char *s_addr_type, unsigned char *dest_addr, unsigned char *d_addr_type, unsigned short int *source_port, unsigned short int *dest_port, unsigned short int *signal, unsigned short int *pkt_seq);
