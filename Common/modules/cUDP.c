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
 * \file     cUDP.c
 * \brief    Compressed UDP for IPv6-packets
 *
 *  
 *  Support 16-bits port number and 4-bits compressed
 */



#include <string.h>
#include <sys/inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifndef CUDP_DEBUG
#undef HAVE_DEBUG
#endif

#include "debug.h"
#include "stack.h"
#include "buffer.h"

#include "module.h"
#include "socket.h"
#include "event_timer.h"
#include "rf.h"
#include "rf_802_15_4.h"
#include "control_message.h"
#include "cipv6.h"

/*
[NAME]
CUDP

[ID]
MODULE_CUDP,

[INFO]
#ifdef HAVE_CUDP
  {cudp_init, cudp_handle, cudp_check, 0, MODULE_CUDP, 8, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR cudp_init(buffer_t *buf);
extern portCHAR cudp_handle( buffer_t *buf );
extern portCHAR cudp_check( buffer_t *buf );

/* HC_UDP defined value */
/* Source Port Compression: Bit 0 */
#define S_PORT_COMPRESSED	0x01
#define S_PORT_UNCOMPRESSED	0x00
/* Destination Port Compression: Bit 1 */
#define D_PORT_COMPRESSED	0x02
#define D_PORT_UNCOMPRESSED	0x00
/* Lenghth Compression: Bit 2 */
#define LENGTH_COMPRESSED	0x04
#define LENGTH_UNCOMPRESSED	0x00

#define LENGTH_COMPRESSED_HC_UDP	( LENGTH_COMPRESSED )
#define COMPRESSED_HC_UDP	( LENGTH_COMPRESSED | S_PORT_COMPRESSED | D_PORT_COMPRESSED )
#define UNCOMPRESSED_HC_UDP	0x00
#define PORT_DECODED		0x03
/* When using port compressed mode using TBD constant value which + 4-bits compressed value */
#define HC2_ENCODE_P_VALUE		0xF0B0
#define HC2_ENCODE_MAX_ENCODE	0xF0C0 
#define HC2_PORT_ENCODE			0xF0AF
#define PUSH_BUFFER		0x01
#define FREE_BUFFER		0x00

/* Bitmask for Port number decoding */
#define S_PORTBM		0x0F
#define D_PORTBM		0xF0
uint8_t use_compress=1;	/* Defaul also turnit on */

portCHAR cudp_init( buffer_t *buf )
{ 
	buf; 	
	return pdTRUE;
}

void cudp_compress_mode( uint8_t mode )
{  	
	use_compress = mode;
}

uint16_t udp_port, udp_data_length;

/**
 *  Main cUDP buffer handler.
 *
 *	\param buf pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR cudp_handle( buffer_t *buf )
{
  /* Process the packet */
  uint8_t hc_udp,tmp_8, header_length=0;
  uint8_t *dptr;
  uint8_t portfield=0;
  control_message_t *ptr;
  uint16_t length=0;
#ifndef NO_FCS
	uint16_t fcs;
#endif
	debug("cUDP: handler.\r\n");
	switch(buf->dir)
	{
		case BUFFER_DOWN:
			debug("down: ");
			debug("\r\n ptr: ");
			debug_int(buf->buf_ptr);
			debug(" end: ");
			debug_int(buf->buf_end);
			debug("\r\n");
			/* Check data payload length */
			udp_data_length = (buf->buf_end - buf->buf_ptr);

			/* Checksum  calculate*/
			udp_data_length +=8;
			if(stack_buffer_headroom( buf,8)==pdFALSE)
			{
				ptr = ( control_message_t*) buf->buf;
				ptr->message.ip_control.message_id = TOO_LONG_PACKET;
				push_to_app(buf);
				return pdTRUE;
			}
			
			buf->buf_ptr -= 8; /* HC2, SPORT,DPORT and Check Sum */
			dptr = buf->buf + buf->buf_ptr;
			
			*dptr++ = (buf->src_sa.port >> 8);		
			*dptr++ = (uint8_t) buf->src_sa.port;
			*dptr++ = (buf->dst_sa.port >> 8);		
			*dptr++ = (uint8_t)buf->dst_sa.port;
			*dptr++ = (udp_data_length >> 8);	
			*dptr++ = (uint8_t) udp_data_length;	
			*dptr++ = 0x00;
			*dptr++ = 0x00;
#ifndef NO_FCS
			fcs = ipv6_fcf(buf,NEXT_HEADER_UDP, 0);
#endif
			if( use_compress==0)
			{	
				dptr = buf->buf + (buf->buf_ptr + 6);
			}
			if ( use_compress )
			{
				portfield = 0;
				tmp_8=2;/* FCS field */
				hc_udp=LENGTH_COMPRESSED;
				/* Check port that these are supported 6LoWPAN ports 61616 - 61631 */
				if((buf->src_sa.port > HC2_PORT_ENCODE &&  buf->src_sa.port < HC2_ENCODE_MAX_ENCODE) && (buf->dst_sa.port > HC2_PORT_ENCODE && buf->dst_sa.port < HC2_ENCODE_MAX_ENCODE))
				{
					hc_udp |= S_PORT_COMPRESSED;
					portfield = (buf->src_sa.port - HC2_ENCODE_P_VALUE);

					hc_udp |= D_PORT_COMPRESSED;
					portfield |= ((buf->dst_sa.port - HC2_ENCODE_P_VALUE) << 4);
					tmp_8++;
				}
				else
				{
					tmp_8 +=4;
				}
				buf->buf_ptr +=(8 - tmp_8);
				dptr = buf->buf + buf->buf_ptr;
				buf->options.lowpan_compressed = hc_udp;
				if(tmp_8 == 3)
				{
					*dptr++ = portfield;	/* Encoded Source and Destination-port*/
				}
				else
				{
					*dptr++ = (buf->src_sa.port >> 8);		
					*dptr++ = (uint8_t) buf->src_sa.port;
					*dptr++ = (buf->dst_sa.port >> 8);		
					*dptr++ = (uint8_t)buf->dst_sa.port;
				}
			}
#ifndef NO_FCS
			/* FCS */
			*dptr++ = (fcs >> 8);		
			*dptr++ = (uint8_t)fcs;
#else
			*dptr++ = 0;		
			*dptr++ = 0;		
#endif
			/*Check sum will be soon real, but for beta testing it is always TRUE */
			buf->src_sa.addr_type = ADDR_NONE;
			buf->from = MODULE_CUDP;
			buf->to = MODULE_NONE;
			stack_buffer_push(buf);
			buf=0;
			break;

		case BUFFER_UP:
			/* Check data payload length */
			udp_data_length = (buf->buf_end - buf->buf_ptr);
			length=0;
			dptr = buf->buf + buf->buf_ptr;
			/* Lets check HC_UDP compress options */
			switch (buf->options.lowpan_compressed)
			{
				/*case UNCOMPRESSED_HC_UDP:
					// Read Src-port
					udp_port = *dptr++;
					udp_port <<= 8;
					udp_port += *dptr++;
					buf->src_sa.port=udp_port;

					udp_port = *dptr++;
					udp_port <<= 8;
					udp_port += *dptr++;
					buf->dst_sa.port=udp_port;

					length = *dptr++;
					length <<= 8;
					length += *dptr++;
					header_length=8;

					udp_data_length = ((buf->buf_end - buf->buf_ptr) - header_length);
					break;
*/
				case COMPRESSED_HC_UDP:
					header_length = 3;
					/* Decode Source and Destination port-number */
					portfield = *dptr++;
					/* Source port */
					buf->src_sa.port = ((portfield & S_PORTBM) + HC2_ENCODE_P_VALUE );
					/* Destination Port */
					buf->dst_sa.port = ((portfield >> 4) + HC2_ENCODE_P_VALUE);

					udp_data_length = ((buf->buf_end - buf->buf_ptr) - header_length);
					length =(udp_data_length + 8);
					break;
				case UNCOMPRESSED_HC_UDP:

				case LENGTH_COMPRESSED_HC_UDP:
					header_length = 6;
					/* Read Src-port */
					udp_port = *dptr++;
					udp_port <<= 8;
					udp_port += *dptr++;
					buf->src_sa.port=udp_port;
					
					udp_port = *dptr++;
					udp_port <<= 8;
					udp_port += *dptr++;
					buf->dst_sa.port=udp_port;
					if(buf->options.lowpan_compressed == UNCOMPRESSED_HC_UDP)
					{
						length = *dptr++;
						length <<= 8;
						length += *dptr++;
						header_length += 2;
						udp_data_length = ((buf->buf_end - buf->buf_ptr) - header_length);
					}
					else
					{
						udp_data_length = ((buf->buf_end - buf->buf_ptr) - header_length);
						length =(udp_data_length + 8);
					}
					break;
			}
#ifndef NO_FCS			
			/*fcs*/
			fcs=0;
			fcs = *dptr++;
			fcs <<= 8;
			fcs += *dptr++;
#else
				dptr += 2;		
#endif /*NO_FCS*/
			buf-> buf_ptr = (dptr - buf->buf);	
			if(buf->options.handle_type == HANDLE_NO_ROUTE_TO_DEST)
			{
				debug("UDP: no del info\r\n");
				udp_port = buf->dst_sa.port;
				buf->dst_sa.port = buf->src_sa.port;
				buf->src_sa.port = udp_port;
				buf->from = MODULE_CUDP;
				buf->to = MODULE_NONE;
				push_to_app(buf);
				buf=0;
				return pdTRUE;
			}
			else
			{
#ifndef NO_FCS			
				if(fcs!=0x0000)
#endif
				{
					/* Calculate and check fcs */
					buf->buf_ptr -= 8; /* HC2, SPORT,DPORT and Check Sum */
					dptr = buf->buf + buf->buf_ptr;
					*dptr++ = (buf->src_sa.port >> 8);		
					*dptr++ = (uint8_t) buf->src_sa.port;
					*dptr++ = (buf->dst_sa.port >> 8);		
					*dptr++ = (uint8_t)buf->dst_sa.port;
					*dptr++ = (length >> 8);	
					*dptr++ = (uint8_t) length;	
#ifndef NO_FCS			
					*dptr++ = (fcs >> 8);
					*dptr++ = (uint8_t)fcs;

					if (ipv6_fcf(buf,NEXT_HEADER_UDP, 1) != 0xFFFF)
					{
						debug("UDP:FCS error\r\n");
						stack_buffer_free(buf);
						buf=0;	
					}
#else	/*NO_FCS*/
					*dptr++ = 0;
					*dptr++ = 0;
#endif	/*NO_FCS*/
					buf->buf_ptr += 8; /* HC2, SPORT,DPORT and Check Sum */
					dptr = buf->buf + buf->buf_ptr;
				}
#ifndef NO_FCS			
				else
				{
					debug("FCS disable\r\n");
				}
#endif
#ifndef HAVE_NRP
				if (buf && (buf->dst_sa.port == NPING_PORT || buf->dst_sa.port == UDP_ECHO_PORT) )
				{ /*Ping*/
					if (   ( (buf->src_sa.port == NPING_PORT) && (buf->dst_sa.port == NPING_PORT) )
							|| ( (buf->src_sa.port == UDP_ECHO_PORT) && (buf->dst_sa.port == UDP_ECHO_PORT) ) )
					{ /*Evil loop ping*/
						stack_buffer_free(buf);
						buf=0;						
					}
					if (buf)
					{ /*respond*/
#ifndef NO_FCS
						uint16_t tmp_fcs;
#endif					

						debug("cUDP: ping ->respond.\r\n");
						/* Change Destination and Source-port */
						udp_port = buf->dst_sa.port;
						buf->dst_sa.port = buf->src_sa.port;
						buf->src_sa.port = udp_port;
						buf->dst_sa.addr_type = buf->src_sa.addr_type;
						buf->src_sa.addr_type = ADDR_NONE;
						memcpy(buf->dst_sa.address, buf->src_sa.address, 8);
#ifndef NO_FCS
						buf->buf_ptr -= 8; /* HC2, SPORT,DPORT and Check Sum */
						dptr = buf->buf + buf->buf_ptr;

						*dptr++ = (buf->src_sa.port >> 8);		
						*dptr++ = (uint8_t) buf->src_sa.port;
						*dptr++ = (buf->dst_sa.port >> 8);		
						*dptr++ = (uint8_t)buf->dst_sa.port;
						*dptr++ = (length >> 8);	
						*dptr++ = (uint8_t) length;	
						*dptr++ = 0x00;
						*dptr++ = 0x00;					
						tmp_fcs = ipv6_fcf(buf,NEXT_HEADER_UDP, 0);
						buf->buf_ptr += 8;
#endif
						/* Create header */
						if(use_compress)
						{
							hc_udp=LENGTH_COMPRESSED;
							tmp_8=2;
							//if(buf->src_sa.port > HC2_PORT_ENCODE && buf->dst_sa.port > HC2_PORT_ENCODE)
							if((buf->src_sa.port > HC2_PORT_ENCODE &&  buf->src_sa.port < HC2_ENCODE_MAX_ENCODE) && (buf->dst_sa.port > HC2_PORT_ENCODE && buf->dst_sa.port < HC2_ENCODE_MAX_ENCODE))
							{
								hc_udp |= S_PORT_COMPRESSED;
								portfield = (buf->src_sa.port - HC2_ENCODE_P_VALUE);
								hc_udp |= D_PORT_COMPRESSED;
								portfield |= ((buf->src_sa.port - HC2_ENCODE_P_VALUE) << 4);
								tmp_8++;
							}
							else
								tmp_8 +=4;
						}
						else
							tmp_8=8;

						buf->buf_ptr -= tmp_8;
						dptr = buf->buf + buf->buf_ptr;
						switch (tmp_8)
						{
							case 6:
								buf->options.lowpan_compressed = LENGTH_COMPRESSED_HC_UDP; 
								*dptr++ = (buf->src_sa.port >> 8);		
								*dptr++ = (uint8_t) buf->src_sa.port;
								*dptr++ = (buf->dst_sa.port >> 8);		
								*dptr++ = (uint8_t)buf->dst_sa.port;
								break;

							case 3:
								buf->options.lowpan_compressed = COMPRESSED_HC_UDP; 
								*dptr++ = portfield;
								break;

							case 8:
								buf->options.lowpan_compressed = 0;
								*dptr++ = (buf->src_sa.port >> 8);		
								*dptr++ = (uint8_t) buf->src_sa.port;
								*dptr++ = (buf->dst_sa.port >> 8);		
								*dptr++ = (uint8_t)buf->dst_sa.port;
								*dptr++ = (length >> 8);
								*dptr++ = (uint8_t) length;
								break;
						}
#ifndef NO_FCS
					/* Add FCS */
						*dptr++ = (tmp_fcs >> 8);		
						*dptr++ = (uint8_t)tmp_fcs;
#else
						*dptr++ = 0;		
						*dptr++ = 0;
#endif
						buf->socket = 0;
						buf->from = MODULE_CUDP;
						buf->dir = BUFFER_DOWN;
						buf->to = MODULE_NONE;
						stack_buffer_push(buf);
						buf=0;
					}
				}
#endif /*HAVE_NRP*/
			}

			if (buf)
			{ /*normal processing*/
				debug("Src port ");
				debug_hex((buf->src_sa.port));
				debug("\r\n");

				debug("Dst port ");
				debug_hex((buf->dst_sa.port));
				debug("\r\n");
				buf->from = MODULE_CUDP;
				buf->to = MODULE_NONE;
				buf->buf_ptr = (dptr - buf->buf); // Move the buffer pointer
#ifndef HAVE_NRP
				if (buf->src_sa.port == UDP_ECHO_PORT)
					parse_echo_response(buf);
				else
					stack_buffer_push(buf);
#else
				stack_buffer_push(buf);
#endif
					

				buf=0;
			}
			break;
	}
return pdTRUE;
}

/**
 *  The cUDP buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE    is cUDP
 *  \return  pdFALSE   is not cUDP or broken header
 */
portCHAR cudp_check( buffer_t *buf )
{
	uint8_t payload_length, *ind;
	uint16_t length;
	portCHAR ret_val = pdTRUE;
	switch (buf->options.lowpan_compressed)
	{
		case UNCOMPRESSED_HC_UDP:
			payload_length = (buf->buf_end - buf->buf_ptr);
			ind = buf->buf + buf->buf_ptr;
			ind += 4;
			length = *ind++;
			length <<= 8;
			length += *ind++;
			if(length !=payload_length)
			{
				ret_val = pdFALSE;
			}
			break;
			
		case LENGTH_COMPRESSED_HC_UDP:
			break;

		case COMPRESSED_HC_UDP:
			break;

		default:
				ret_val = pdFALSE;
			break;

	}
	return ret_val; 
}
