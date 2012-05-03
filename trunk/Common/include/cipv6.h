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
 * \file cipv6.h
 * \brief include structure defines and other constant
 *
 *  nanoStack: These tables are for zigbee and 6lowpan modules
 *	
 */

#ifndef _CIPV6_H
#define _CIPV6_H

#include "address.h"

/* LowPan's defined constant */

/* How detec header type */
#define LOWPAN_TYPE_BM		0x03
#define MESH_ROUTING_TYPE	0x01
#define DISPATCH_TYPE		0x02
#define FRAGMENT_TYPE		0x03

/* LowPan HC1 Dispatch Header mask */
#define NALP 		0x00
#define IPV6 		0x82
#define LOWPAN_HC1	0x42
#define LOWPAN_BC0	0x09
#define ESC		0xfe


/* Mesh flags for indicates addresstype */
#define MESH_ADDRESSTYPE_BM	0x0c
#define O_ADDRESSTYPE_BM	0x04
#define O_ADDRESSTYPE_16	0x04
#define O_ADDRESSTYPE_64	0x00
#define D_ADDRESSTYPE_BM	0x08
#define D_ADDRESSTYPE_16	0x08
#define D_ADDRESSTYPE_64	0x00

#define BASIC_HOP_VALUE			0x40
#define FLOODING_HOP_VALUE		0xA0
#define GENERAL_HOPLIMIT		0x16
#define PRE_MESH_HEADER	(MESH_ROUTING_TYPE | O_ADDRESSTYPE_64 | D_ADDRESSTYPE_64 )

#define FRAGMENT_OPTIONS_BM		0x04
#define FIRST_FRAGMENT			0x00
#define SUBSEQUENCY_FRAGMENTS	0x04

/* cIPv6's defined constant */

/* cIPV6 HC1 Header mask */
/* IPv6 Source address compression field: Bit 0-1*/
#define S_PIII	0x00 /* Prefix and Interface identifier carried in-line */
#define S_PIIC	0x01 /* Prefix carried in-line and Interface identifier compressed */
#define S_PCII	0x02 /* Prefix compressed and Interface identifier carried in-line */
#define S_PCIC	0x03 /* Prefix  and Interface identifier compressed*/

/* IPv6 Destination address compression field: Bit 2-3*/
#define D_PIII	0x00 /* Prefix and Interface identifier carried in-line */
#define D_PIIC	0x04 /* Prefix carried in-line and Interface identifier compressed */
#define D_PCII	0x08 /* Prefix compressed and Interface identifier carried in-line */
#define D_PCIC	0x0c /* Prefix  and Interface identifier compressed*/

/* Traffic & flow compression flag */
#define TRAFFIC_FLOW_COMPRESSED 	0x10
#define TRAFFIC_FLOW_NONCOMPRESSED 	0x00

/* Next Header compression field: bits 5 & 6 */
#define NEXTH_NOCOMPRESS	0x00 /* show whole 8-bits field */
#define NEXTH_UDP		0x20
#define NEXTH_ICMP		0x40
#define NEXTH_TCP		0x60

#define NEXT_HEADER_UDP		0x11
#define NEXT_HEADER_ICMP6	0x3A
/* HC2 encode */
#define NO_MORE_COMP_FIELD		0x00
#define HC2_FIELD_NEXT			0x80
#define HC1_NEXT_HEADER_UNCOMPRES_UDP		(S_PCIC | D_PCIC | TRAFFIC_FLOW_COMPRESSED | NEXTH_UDP )
#define HC1_NEXT_HEADER_COMPRESSED_UDP		(S_PCIC | D_PCIC | TRAFFIC_FLOW_COMPRESSED | NEXTH_UDP | HC2_FIELD_NEXT)
#define IP_HEADER_FOR_ICMP					(S_PCIC | D_PCIC | TRAFFIC_FLOW_COMPRESSED | NEXTH_ICMP)
#define PUSH_BUFFER			0x01
#define FREE_BUFFER			0x00

/* ICMP define constant */
#define ROUTER_ADVRT_TYPE			134
#define ROUTER_SOLICICATION_TYPE	133
#define ECHO_REQUEST_TYPE			128
#define ECHO_RESPONSE_TYPE			129
#define ICMP_ERROR_MESSAGE_TYPE		1
#define ICMP_CODE					0
#define ERROR_CODE_NO_ROUTE			0
#define ERROR_CODE_BROKEN_LINK		3

/** Compressed IPv6 Information base */
typedef struct
{
	uint8_t own_brodcast_sqn;			/*!< Own broadcast sequency number. */
	uint8_t short_address[2];			/*!< Short address. */
	uint8_t use_short_address;			/*!< Show which address will be used. */
	uint8_t use_full_compress;			/*!< Show which compressed mode is used. */
}cipv6_ib_t;

extern portCHAR build_lowpan_header(buffer_t *buf);
extern void parse_lowpan_packet(buffer_t *buf, uint8_t packet_type);
extern void update_ip_sqn(void);
extern portCHAR build_ipv6_header(buffer_t *buf);
extern void parse_ipv_header(buffer_t *buf);
extern void add_fcf(buffer_t *buf );

extern portCHAR compare_ori_to_own(addrtype_t type, address_t address);
extern void ip_address_setup( uint8_t short_addr, uint8_t *ptr);
#ifndef NO_FCS
extern uint16_t ipv6_fcf(buffer_t *buf, uint8_t next_protocol, uint8_t rx_case);
#endif /*NO_FCS*/
extern void ip_broken_link_notify(buffer_t *buf, uint8_t no_route_reason);
#ifdef HAVE_ROUTING
extern void route_packet(buffer_t *buf, address_t ori_address, addrtype_t o_addrtype, uint8_t mesh_header, uint8_t hops_left, uint8_t ind);
#endif
#endif /*_CIPV6_H*/
