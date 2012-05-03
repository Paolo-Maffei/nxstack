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


#ifndef LIBNRP_H
#include "libnRP.h"
#endif

//extern unsigned char libnrp_addr_table[32][128];

int DEBUGLEVEL = NRP_CRITICAL;

void libnrp_debug(int debuglevel, const char *dbg, ...);
int libnrp_check_nRoute_reply(unsigned char *reply, int len);

/**	This function can be used to create a complete nRP data packet. */
int libnrp_create_data_pkt_hdr(unsigned char *buffer, unsigned char *data, unsigned char datalen, unsigned char proto, unsigned char *source_addr, unsigned char *dest_addr, unsigned char d_addr_type, unsigned short int *source_port, unsigned short int dest_port, unsigned short int *pkt_seq)
{
	unsigned char *buf;
	unsigned int i=0, j=0;
	unsigned int last_field = 1;	// This variable will contain the info about the last field, it's a simple order number

	buf = buffer;

/*	Check that *buffer is not a NULL pointer. It is users duty to make sure that there is enough room in *buffer. */
	if(buf == NULL)
	{
		libnrp_debug(NRP_CRITICAL, "NULL pointer at libnrp_create_data_pkt_hdr().\n");
		return(-1);
	}

/*	This ugly looking pile of if() statements is to find out which field is to be marked last. Note that it currently allows to have headers that don't
 *	have all the necessary fields. */	
	if(pkt_seq != NULL)
	{
		last_field=7;
	}
	else if(dest_port != PORT_UNDEFINED)
	{
		last_field=6;
	}
	else if(source_port != NULL)
	{
		last_field=5;
	}
	else if(dest_addr != NULL)
	{
		last_field=4;
	}
	else if(source_addr != NULL)
	{
		last_field=3;
	}
	else if(proto != PROTO_UNDEFINED)
	{
		last_field=2;
	}

/*	Insert the 4 byte nRP header. */
	memcpy(buf, "NRP", 3);
	buf += 3;
	i += 3;
	
	
	*buf = (NRP_VERSION << 4);
	*buf = (*buf) & 0xf0;		/*	This is a little bit redundant but to make sure that the lowest 4 bits are all zero (meaning a data packet). */

	buf++;
	i++;

	if(data != NULL)
	{
		unsigned short int dlen;
		libnrp_debug(NRP_INFO, "Inserting data field.\n");
	
		if(last_field == 1)
		{
			*buf = 0x80;
		}
		else
		{
			*buf = 0x00;
		}
		
		buf++;
		i++;
		
		dlen = datalen;
		*buf = dlen >> 8;
		buf++;
		i++;
		*buf = dlen & 0xff;
		buf++;
		i++; 
		
		memcpy(buf, data, datalen);
		
		buf += datalen;
		i += datalen;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No data field.\n");
	}

	if(proto != PROTO_UNDEFINED)
	{
		unsigned char proto_hdr[3] = { 0x01, 0x00, 0x01 };
		libnrp_debug(NRP_INFO, "Inserting protocol field.\n");
		memcpy(buf, proto_hdr, 3);
		if(last_field == 2)
		{
			*buf = 0x81;			
		}
		
		buf += 3;
		*buf = proto;
		buf++;
		i += 4;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No protocol field.\n");
	}
	
	if(source_addr != NULL)
	{
		unsigned int addr[4];
		unsigned char s_addr_hdr[4] = { 0x02, 0x00, 0x05, 0x11 };
		libnrp_debug(NRP_DEBUG, "Inserting source address.\n");
		memcpy(buf, s_addr_hdr, 4);
		
		if(last_field == 3)
		{
			*buf = 0x82;
		}
		
		buf += 4;
		i+= 4;
		sscanf((char *)source_addr, "%u.%u.%u.%u", &(addr[0]), &(addr[1]), &(addr[2]), &(addr[3])); 
		//printf("source addr:%d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);
//		sprintf((char *)buf, "%c%c%c%c", addr[0], addr[1], addr[2], addr[3]);
		*buf = addr[0];
		*(buf+1) = addr[1];
		*(buf+2) = addr[2];
		*(buf+3) = addr[3];
		buf += 4;
		i += 4;
	}
	else
	{
		libnrp_debug(NRP_DEBUG, "No source address field.\n");
	}
	
	if(dest_addr != NULL)
	{
		if(last_field == 4)
			*buf = 0x83;
		else
			*buf = 0x03;

		buf++;
		i++;

		libnrp_debug(NRP_INFO, "Address type:%s\n", (char *)libnrp_addr_table[d_addr_type]);
		switch(d_addr_type)
		{
			case 0x00:
			{
				/*	48 bit IEEE hardware identifier */
				unsigned char len[2] = { 0x00, 0x07 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 6);
				buf += 6;
				i += 6;
				break;
			}
			case 0x01:
			{
				/*	802.15.4 64 bit address */
				unsigned char len[2] = { 0x00, 0x09 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 8);
				buf += 8;
				i += 8;
				break;
			}
			case 0x02:
			{
				/*	16 bit 802.15.4 PAN ID */
				unsigned char len[2] = { 0x00, 0x03 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 2);
				buf += 2;
				i += 2;
				break;
			}
			case 0x03:
			{
				/*	16 bit 802.15.4 short address */
				unsigned char len[2] = { 0x00, 0x03 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 2);
				buf += 2;
				i += 2;
				break;
			}
			case 0x04:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x05:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x06:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x07:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x08:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x09:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0A:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0B:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0C:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0D:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0E:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0F:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x10:
			{
				unsigned int addr[4];
				libnrp_debug(NRP_DEBUG, "Inserting destination address.\n");
				sscanf((char *)dest_addr, "%u.%u.%u.%u", &(addr[0]), &(addr[1]), &(addr[2]), &(addr[3])); 
//				sprintf((char *)buf, "%c%c%c%c", addr[0], addr[1], addr[2], addr[3]);
				*buf = addr[0];
				*(buf+1) = addr[1];
				*(buf+2) = addr[2];
				*(buf+3) = addr[3];
				buf += 4;
				i += 4;
				break;
			}
			case 0x11:
				/*	To be implemented. */			
				break;
			case 0x12:
				/*	To be implemented. */			
				break;
			default:
				libnrp_debug(NRP_CRITICAL, "Unknown address type: 0x%.2x.\n", d_addr_type);
				return(-1);
				break;
		}

	}
	else
	{
		libnrp_debug(NRP_DEBUG, "No destination address field.\n");
	}

	if(source_port != NULL)
	{
		unsigned char source_port_hdr[3] = { 0x04, 0x00, 0x02 };

		libnrp_debug(NRP_INFO, "Inserting source port number field.\n");
		memcpy(buf, source_port_hdr, 3);
		if(last_field == 5)
		{
			*buf = 0x84;			
		}
		
		buf += 3;
		/*	We must make sure that the port number is in MSB byte order as required by the nRP specification. */
		*buf = *source_port >> 8;
		buf++;
		*buf = *source_port & 0xff;
		buf++;
		i += 5;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No source port number field.\n");
	}

	if(dest_port != PORT_UNDEFINED)
	{
		unsigned char dest_port_hdr[3] = { 0x05, 0x00, 0x02 };

		libnrp_debug(NRP_INFO, "Inserting destination port number field.\n");
		memcpy(buf, dest_port_hdr, 3);
		if(last_field == 6)
		{
			*buf = 0x85;			
		}
		
		buf += 3;
		/*	We must make sure that the port number is in MSB byte order as required by the nRP specification. */
		*buf = dest_port >> 8;
		buf++;
		*buf = dest_port & 0xff;
		buf++;
		i += 5;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No destination port number field.\n");
	}

	if(pkt_seq != NULL)
	{
		unsigned char dest_port_hdr[3] = { 0x07, 0x00, 0x02 };
		unsigned short int seq;
		/*	We must make sure that the packet sequence number is in MSB byte order as required by the nRP specification. */
		seq = htons(*pkt_seq);
		libnrp_debug(NRP_INFO, "Inserting packet sequence number field.\n");
		memcpy(buf, dest_port_hdr, 3);
		if(last_field == 7)
		{
			*buf = 0x87;			
		}
		
		buf += 3;
/*		*buf = seq >> 8;
		buf++;
		*buf = seq & 0xff;
		buf++;
*/
		memcpy(buf, &seq, 2);
		buf += 2;
		i += 5;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No packet sequence field.\n");
	}		
	
	for(;j<i;j++)
	{
		//printf("0x%.2x-", buffer[j]);
	}
	//printf("\n");
	
	return(i);
}

/**	A nRouted configuration packet creation function. */
int libnrp_create_conf_pkt(unsigned char *buffer, unsigned char proto, unsigned char *source_addr, unsigned char *dest_addr, unsigned char d_addr_type, unsigned short int *source_port, unsigned short int dest_port)
{
	unsigned char *buf;
	unsigned int i=0;
	unsigned int last_field = 1;	// This variable will contain the info about the last field, it's a simple order number

	buf = buffer;

/*	Check that *buffer is not a NULL pointer. It is users duty to make sure that there is enough room in *buffer. */
	if(buf == NULL)
	{
		libnrp_debug(NRP_CRITICAL, "NULL pointer at libnrp_create_conf_pkt().\n");
		return(-1);
	}

/*	This ugly looking pile of if() statements is to find out which field is to be marked last. Note that it currently allows to have headers that don't
 *	have all the necessary fields. */	
	else if(dest_port != PORT_UNDEFINED)
	{
		last_field=5;
	}
	else if(source_port != NULL)
	{
		last_field=4;
	}
	else if(dest_addr != NULL)
	{
		last_field=3;
	}
	else if(source_addr != NULL)
	{
		last_field=2;
	}

/*	Insert the 4 byte nRP header. */
	memcpy(buf, "NRP", 3);
	buf += 3;
	i += 3;
	
	
	*buf = (NRP_VERSION << 8);
	*buf = (*buf) | NROUTED_CONFIG;

	buf++;
	i++;


	if(proto != PROTO_UNDEFINED)
	{
		unsigned char proto_hdr[3] = { 0x00, 0x00, 0x01 };
		libnrp_debug(NRP_INFO, "Inserting protocol field.\n");
		memcpy(buf, proto_hdr, 3);
		if(last_field == 1)
		{
			*buf = 0x80;			
		}
		
		buf += 3;
		*buf = proto;
		if((*buf) != 0x00 && NRP_SPEC_VERSION < 1.0)
		{
			printf("nRPlib warning: this vesrion of nRPlib does not necessarily support the requested protocol in nRouted configuration packet.\n");
		} 
		
		buf++;
		i += 4;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No protocol field.\n");
	}
	
	if(source_addr != NULL)
	{
		unsigned int addr[4];
		unsigned char s_addr_hdr[4] = { 0x01, 0x00, 0x05, 0x11 };
		libnrp_debug(NRP_DEBUG, "Inserting source address.\n");
		memcpy(buf, s_addr_hdr, 4);
		
		if(last_field == 2)
		{
			*buf = 0x81;
		}
		
		buf += 4;
		i+= 4;
		sscanf((char *)source_addr, "%u.%u.%u.%u", &(addr[0]), &(addr[1]), &(addr[2]), &(addr[3])); 
		//printf("source addr:%d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);
//		sprintf((char *)buf, "%c%c%c%c", addr[0], addr[1], addr[2], addr[3]);
		*buf = addr[0];
		*(buf+1) = addr[1];
		*(buf+2) = addr[2];
		*(buf+3) = addr[3];
		buf += 4;
		i += 4;
	}
	else
	{
		libnrp_debug(NRP_DEBUG, "No source address field.\n");
	}
	
	if(dest_addr != NULL)
	{
		if(last_field == 3)
			*buf = 0x82;
		else
			*buf = 0x02;

		buf++;
		i++;

		libnrp_debug(NRP_INFO, "Address type:%s\n", (char *)libnrp_addr_table[d_addr_type]);
		switch(d_addr_type)
		{
			case 0x00:
			{
				/*	48 bit IEEE hardware identifier */
				unsigned char len[2] = { 0x00, 0x07 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 6);
				buf += 6;
				i += 6;
				break;
			}
			case 0x01:
			{
				/*	802.15.4 64 bit address */
				unsigned char len[2] = { 0x00, 0x09 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 8);
				buf += 8;
				i += 8;
				break;
			}
			case 0x02:
			{
				/*	16 bit 802.15.4 PAN ID */
				unsigned char len[2] = { 0x00, 0x03 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 2);
				buf += 2;
				i += 2;
				break;
			}
			case 0x03:
			{
				/*	16 bit 802.15.4 short address */
				unsigned char len[2] = { 0x00, 0x03 };
				memcpy(buf, len, 2);
				buf += 2;
				i += 2;
				*buf = d_addr_type;
				buf++;
				i++;
				memcpy(buf, dest_addr, 2);
				buf += 2;
				i += 2;
				break;
			}
			case 0x04:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x05:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x06:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x07:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x08:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x09:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0A:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0B:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0C:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0D:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0E:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x0F:
				/*	Placeholder for future address type definitions. */
				break;
			case 0x10:
			{
				unsigned int addr[4];
				unsigned char addr_hdr[3] = { 0x00, 0x05, 0x10 };

				memcpy(buf, addr_hdr, 3);
				buf += 3;
				i += 3;
				libnrp_debug(NRP_DEBUG, "Inserting destination address.\n");
				sscanf((char *)dest_addr, "%u.%u.%u.%u", &(addr[0]), &(addr[1]), &(addr[2]), &(addr[3])); 
//				sprintf((char *)buf, "%c%c%c%c", addr[0], addr[1], addr[2], addr[3]);
				*buf = addr[0];
				*(buf+1) = addr[1];
				*(buf+2) = addr[2];
				*(buf+3) = addr[3];
				buf += 4;
				i += 4;
				break;
			}
			case 0x11:
				/*	To be implemented. */			
				break;
			case 0x12:
				/*	To be implemented. */			
				break;
			case 0xff:
				libnrp_debug(NRP_INFO, "Address type 0xff (unknown).\n");
				break;
			default:
				libnrp_debug(NRP_CRITICAL, "Unknown address type: 0x%.2x.\n", d_addr_type);
				return(-1);
				break;
		}

	}
	else
	{
		libnrp_debug(NRP_DEBUG, "No destination address field.\n");
	}

	if(source_port != NULL)
	{
		unsigned char source_port_hdr[3] = { 0x03, 0x00, 0x02 };

		libnrp_debug(NRP_INFO, "Inserting source port number field.\n");
		memcpy(buf, source_port_hdr, 3);
		if(last_field == 4)
		{
			*buf = 0x83;			
		}
		
		/*	We must make sure that the port number is in MSB byte order as required by the nRP specification. */
		buf += 3;
		*buf = *source_port >> 8;
		buf++;
		*buf = *source_port & 0xff;
		buf++;
		i += 5;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No source port number field.\n");
	}

	if(dest_port != PORT_UNDEFINED)
	{
		unsigned char dest_port_hdr[3] = { 0x04, 0x00, 0x02 };

		libnrp_debug(NRP_INFO, "Inserting destination port number field.\n");
		memcpy(buf, dest_port_hdr, 3);
		if(last_field == 5)
		{
			*buf = 0x84;			
		}
		
		/*	We must make sure that the port number is in MSB byte order as required by the nRP specification. */
		buf += 3;
		*buf = dest_port >> 8;
		buf++;
		*buf = dest_port & 0xff;
		buf++;
		i += 5;
	}
	else
	{
		libnrp_debug(NRP_INFO, "No destination port number field.\n");
	}
	
	return(i);
}

int libnrp_snd_conf(unsigned char *buffer, int pkt_len, char *addr, unsigned int port)
{
	int sockfd;
	struct sockaddr_in srvaddr;
	
	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		libnrp_debug(NRP_CRITICAL, "Failed create TCP socket!\n");
		return(sockfd);
	}
	
//	printf("libnRP: sockfd:%d\n", sockfd);
	
	bzero(&srvaddr, sizeof(srvaddr));
	
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_port = htons(port);
	inet_pton(AF_INET, addr, &srvaddr.sin_addr);

	if((connect(sockfd, (struct sockaddr *)&srvaddr, sizeof(srvaddr))) != 0)
	{
		libnrp_debug(NRP_CRITICAL, "Connect to nRouted failed...\n");
		return(-1);
	}
	else
	{
		unsigned char rbuf[128];
		int i;
		int rval;
		
		write(sockfd, buffer, pkt_len);

		i = read(sockfd, rbuf, 128);

		if((rval = libnrp_check_nRoute_reply(rbuf, i)) != 1)
		{
			libnrp_debug(NRP_CRITICAL, "nRouted configuration error.\n");
			return(-1);
		}
		else
			return(sockfd);
	}
}

void libnrp_debug(int debuglevel, const char *dbg, ...)
{
	va_list ap;

	//	Print debug info only if the argument debuglevel is smaller or equal to the debuglevel defined in headerfile
	if(debuglevel <= DEBUGLEVEL)
	{
		//	Process the variable argument list
		va_start(ap, dbg);
		vprintf(dbg, ap);
		va_end(ap);
		fflush(stdout);
	}
	return;	
}

int libnrp_check_nRoute_reply(unsigned char *reply, int len)
{
	unsigned char conf_reply_ok[9] = { 0x4E, 0x52, 0x50, 0x03, 0x05, 0x00, 0x02, 0x4F, 0x4B };
	unsigned char conf_reply_fail[11] = { 0x4E, 0x52, 0x50, 0x03, 0x05, 0x00, 0x02, 0x46, 0x41, 0x49, 0x4C };

	libnrp_debug(NRP_DEBUG, "Checking nRoute configuration reply packet.\n");
	
	if(len == 9)
	{
		if(memcmp(conf_reply_ok, reply, 9) == 0)
		{
			libnrp_debug(NRP_DEBUG, "Reply indicated successful configuration.\n");
			return(1);
		}
		else
		{
			libnrp_debug(NRP_DEBUG, "Malformed configuration reply packet.\n");
			return(-1);
		}
	}
	else if(len == 11)
	{
				if(memcmp(conf_reply_fail, reply, 9) == 0)
		{
			libnrp_debug(NRP_DEBUG, "Reply indicated failed configuration.\n");
			return(-1);
		}
		else
		{
			libnrp_debug(NRP_DEBUG, "Malformed configuration reply packet.\n");
			return(-1);
		}
	}
	else
	{
		libnrp_debug(NRP_DEBUG, "Malformed configuration reply packet.\n");
		return(-1);
	}
}	

int libnrp_get_source_address(unsigned char *buffer, unsigned int buflen, unsigned char *addr_type, unsigned char *address_bytes, char *address_string)
{
	int libnrp_index = 0;
	unsigned char libnrp_field_len;
	unsigned char addrlen;
	unsigned char dual_addr = 0;
	unsigned char done = 0;

	if(((buffer[3]) & 0xf) == 0x00)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP data buffer.\n");

		for(libnrp_index=4 ; libnrp_index < buflen && done == 0;)
		{
			
			if(buffer[libnrp_index] & 0x80)
				done = 1;
			
			switch (buffer[libnrp_index] & 0x7f)
			{
				case 0x00:
					libnrp_field_len = ((uint16_t)buffer[libnrp_index+1]) << 8;
					libnrp_field_len += ((uint16_t)buffer[libnrp_index+2]);
					
					if(buflen-libnrp_index < libnrp_field_len)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					libnrp_index += (libnrp_field_len + 3);
					break;
			
				case 0x01:
					libnrp_index += 4;					
					break;
			
				case 0x02:
					libnrp_debug(NRP_DEBUG, "Found source address tag (0x02).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);

					if(dual_addr != 0)
						*addr_type = ADDR_15_4_PAN_AND_SHORT;
					else
						*addr_type = buffer[libnrp_index+3];

					if(buflen-libnrp_index < addrlen)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}

					if(*addr_type == 0x10)
					{
						libnrp_debug(NRP_DEBUG, "Source address in string format 0x%.2x.\n", *addr_type);
						memcpy(address_string, &(buffer[libnrp_index+4]), addrlen);
						libnrp_debug(NRP_DEBUG, "Source address:%s\n", address_string);
					}
					else if(dual_addr != 0)
					{
						libnrp_debug(NRP_DEBUG, "Second source address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						if(buffer[libnrp_index+3] == ADDR_IEEE_802_15_4_PAN_ID)
						{
							memcpy(&(address_bytes[dual_addr]), &(buffer[libnrp_index+4]), addrlen);
						}
						else if(buffer[libnrp_index+3] == ADDR_IEEE_802_15_4_SHORT)
						{
							memmove(&(address_bytes[dual_addr]), address_bytes, dual_addr);
							memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
						}
						
						*addr_type = 0x80;
					}
					else if(*addr_type == ADDR_IEEE_802_15_4_PAN_ID || *addr_type == ADDR_IEEE_802_15_4_SHORT)
					{
						libnrp_debug(NRP_DEBUG, "Source address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
						dual_addr = addrlen;
					}
					else
					{
						libnrp_debug(NRP_DEBUG, "Source address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
					}
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x03:
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x04:
					libnrp_index += 5;
					break;

				case 0x05:
					libnrp_index += 5;
					break;

				case 0x06:
					libnrp_index += 5;
					break;

				case 0x07:
					libnrp_index += 5;
					break;
			}
		}
		return(1);
	}
	else if(((buffer[3]) & 0xf) == 0x01)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration request packet.\n");
	}
	else if(((buffer[3]) & 0xf) == 0x02)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration reply packet.\n");
	}
	
	return(-1);
}


int libnrp_get_destination_address(unsigned char *buffer, unsigned int buflen, unsigned char *addr_type, unsigned char *address_bytes, char *address_string)
{
	int libnrp_index = 0;
	unsigned char libnrp_field_len;
	unsigned char addrlen;
	unsigned char dual_addr = 0;
	unsigned char done = 0;
	
	if(((buffer[3]) & 0xf) == 0x00)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP data buffer.\n");

		for(libnrp_index=4 ; libnrp_index < buflen ;)
		{
			switch (buffer[libnrp_index] & 0x7f)
			{
				case 0x00:
					libnrp_field_len = ((uint16_t)buffer[libnrp_index+1]) << 8;
					libnrp_field_len += ((uint16_t)buffer[libnrp_index+2]);
					
					if(buflen-libnrp_index < libnrp_field_len)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					libnrp_index += (libnrp_field_len + 3);
					break;
			
				case 0x01:
					libnrp_index += 4;					
					break;
			
				case 0x03:
					libnrp_debug(NRP_DEBUG, "Found destination address tag (0x02).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);

					if(dual_addr != 0)
						*addr_type = ADDR_15_4_PAN_AND_SHORT;
					else
						*addr_type = buffer[libnrp_index+3];

					if(buflen-libnrp_index < addrlen)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}

					if(*addr_type == 0x10)
					{
						libnrp_debug(NRP_DEBUG, "Destination address in string format 0x%.2x.\n", *addr_type);
						memcpy(address_string, &(buffer[libnrp_index+4]), addrlen);
						libnrp_debug(NRP_DEBUG, "Destination address:%s\n", address_string);
					}
					else if(dual_addr != 0)
					{
						libnrp_debug(NRP_DEBUG, "Second destination address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						if(buffer[libnrp_index+3] == ADDR_IEEE_802_15_4_PAN_ID)
						{
							memcpy(&(address_bytes[dual_addr]), &(buffer[libnrp_index+4]), addrlen);
						}
						else if(buffer[libnrp_index+3] == ADDR_IEEE_802_15_4_SHORT)
						{
							memmove(&(address_bytes[dual_addr]), address_bytes, dual_addr);
							memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
						}
						
						*addr_type = 0x80;
					}
					else if(*addr_type == ADDR_IEEE_802_15_4_PAN_ID || *addr_type == ADDR_IEEE_802_15_4_SHORT)
					{
						libnrp_debug(NRP_DEBUG, "Destination address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
						dual_addr = addrlen;
					}
					else
					{
						libnrp_debug(NRP_DEBUG, "Destination address is in format 0x%.2x.\n", *addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", *addr_type, (char *)libnrp_addr_table[*addr_type]);

						memcpy(address_bytes, &(buffer[libnrp_index+4]), addrlen);
					}
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x02:
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x04:
					libnrp_index += 5;
					break;

				case 0x05:
					libnrp_index += 5;
					break;

				case 0x06:
					libnrp_index += 5;
					break;

				case 0x07:
					libnrp_index += 5;
					break;
			}
		}
	}
	else if(((buffer[3]) & 0xf) == 0x01)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration request packet.\n");
	}
	else if(((buffer[3]) & 0xf) == 0x02)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration reply packet.\n");
	}
	
	return(-1);
}



int libnrp_get_data(unsigned char *buffer, unsigned int buflen, unsigned char *data, unsigned int *datalen)
{
	int libnrp_index = 0;
	unsigned char libnrp_field_len = 0x00;
	unsigned char addrlen;
	
	if(((buffer[3]) & 0xf) == 0x00)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP data buffer.\n");

		for(libnrp_index=4 ; libnrp_index < buflen ;)
		{
			switch (buffer[libnrp_index] & 0x7f)
			{
				case 0x00:
					libnrp_debug(NRP_DEBUG, "Tag:0x00\n");
					libnrp_field_len = ((uint16_t)buffer[libnrp_index+1]) << 8;
					libnrp_field_len += ((uint16_t)buffer[libnrp_index+2]);
					
					if(buflen-libnrp_index < libnrp_field_len)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					memcpy(data, &buffer[libnrp_index+3], libnrp_field_len);
					*datalen = libnrp_field_len;

					libnrp_index += (libnrp_field_len + 3);
					return(1);
			
				case 0x01:
					libnrp_debug(NRP_DEBUG, "Tag:0x01\n");
					libnrp_index += 4;					
					break;
			
				case 0x02:
					libnrp_debug(NRP_DEBUG, "Tag:0x02\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x03:
					libnrp_debug(NRP_DEBUG, "Tag:0x03\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x04:
					libnrp_debug(NRP_DEBUG, "Tag:0x04\n");
					libnrp_index += 5;
					break;

				case 0x05:
					libnrp_debug(NRP_DEBUG, "Tag:0x05\n");
					libnrp_index += 5;
					break;

				case 0x06:
					libnrp_debug(NRP_DEBUG, "Tag:0x06\n");
					libnrp_index += 5;
					break;

				case 0x07:
					libnrp_debug(NRP_DEBUG, "Tag:0x07\n");
					libnrp_index += 5;
					break;
			}
		}
	}
	else if(((buffer[3]) & 0xf) == 0x01)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration request packet.\n");
	}
	else if(((buffer[3]) & 0xf) == 0x02)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP configuration reply packet.\n");
	}
	else
	{
		libnrp_debug(NRP_DEBUG, "libnRP could not recognize the packet - trouble!.\n");
	}

	return(-1);
}

/*
int libnrp_get_RANDOM(unsigned char *buffer, unsigned int buflen, unsigned char addr_type, unsigned char *address_bytes, char *address_string)
{
	int libnrp_index = 0;
	unsigned char libnrp_field_tag = 0x00;
	
	if(((buffer[3]) & 0xf) == 0x00)
	{
		printf(""Looks like a nRP data buffer.\n");

		for(libnrp_index=4 ; libnrp_index < buflen ;)
		{
			logger(2, "Going to switch() statement with idx:%d/%d and buffer[idx]:0x%.2x\n", libnrp_index, buflen, buffer[libnrp_index]);
			switch (buffer[libnrp_index] & 0x7f)
			{
				case 0x00:
					logger(2, "Found data tag (0x00).\n");
					field_len = ((uint16_t)buffer[libnrp_index+1]) << 8;
					field_len += ((uint16_t)buffer[libnrp_index+2]);
					
					if(buflen-libnrp_index < field_len)
					{
						logger(0, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					logger(2, "Data length:%u.\n", field_len);
					memcpy(data, &(buffer[libnrp_index+3]), field_len);
					libnrp_index += (field_len + 3);
					break;
			
				case 0x01:
					logger(2, "Found protocol tag (value:%s) in nRP buffer from serial.\n", (char *)nrp_proto_table[buffer[libnrp_index+3]]);
					memcpy(&proto, &(buffer[libnrp_index+3]), 1);
					
					libnrp_index += 4;					
					break;
			
				case 0x02:
					logger(2, "Found source address tag (0x02).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					addr_type = buffer[libnrp_index+3];

					if(buflen-libnrp_index < addrlen)
					{
						logger(0, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}

					if(addr_type == 0x10)
					{
						logger(2, "Source address in string format 0x%.2x.\n", addr_type);
//						strncpy(source_addr_str, &(buffer[libnrp_index+4]), addrlen);
						memcpy(source_addr, &(buffer[libnrp_index+4]), addrlen);
						logger(2, "Source address:%s\n", source_addr);
					}
					else
					{
						logger(2, "Source address is in format 0x%.2x.\n", addr_type);
						logger(2, "Address type:(0x%.2x) -> %s\n", addr_type, (char *)nrp_addr_table[addr_type]);

						memcpy(source_addr, &(buffer[libnrp_index+4]), addrlen);
					}
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x03:
					logger(2, "Found destination address tag (0x03).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					addr_type = buffer[libnrp_index+3];

					if(addr_type == 0x10)
					{
						logger(2, "Destination address in string format 0x%.2x.\n", addr_type);
//						strncpy(dest_addr_str, &(buffer[libnrp_index+4]), addrlen);
						memcpy(dest_addr, &(buffer[libnrp_index+4]), addrlen);
						logger(2, "Destination address:%s\n", dest_addr_str);
					}
					else
					{
						logger(2, "Destination address is in format 0x%.2x.\n", addr_type);
						memcpy(dest_addr, &(buffer[libnrp_index+4]), addrlen);
					}	 
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x04:
					logger(2, "Found source port tag (0x04).\n");
					
					sport = buffer[libnrp_index+3] << 8;
					sport += buffer[libnrp_index+4];
					logger(2, "Source port:%d\n" , sport);
					
					libnrp_index += 5;
					break;

				case 0x05:
					logger(2, "Found destination port tag (0x05).\n");
					
					dport = buffer[libnrp_index+3] << 8;
					dport += buffer[libnrp_index+4];
					logger(2, "Destination port:%d\n" , dport);
					
					libnrp_index += 5;
					break;

				case 0x06:
					logger(2, "Found signal level tag (0x06).\n");
					
					memcpy(&signal_lvl, &(buffer[libnrp_index+3]), 2);
					signal_lvl = ntohs(signal_lvl);
//					signal_lvl = buffer[libnrp_index+3] << 8;
//					signal_lvl += buffer[libnrp_index+4];
					
					logger(2, "Signal level:0x%.4x\n", signal_lvl);
					logger(2, "Signal level:%d\n", signal_lvl);
					
					libnrp_index += 5;
					break;

				case 0x07:
					logger(2, "Found packet sequence ID tag (0x07).\n");
					
					buffer_id = buffer[libnrp_index+3] << 8;
					buffer_id += buffer[libnrp_index+4];
					
					logger(2, "Packet ID:%d\n", buffer_id);
					
					libnrp_index += 5;
					break;
			}
		}
	}
	else if(((buffer[3]) & 0xf) == 0x01)
	{
		logger(2, "Looks like a nRP configuration request packet.\n");
	}
	else if(((buffer[3]) & 0xf) == 0x02)
	{
		logger(2, "Looks like a nRP configuration reply packet.\n");
	}
}
*/

int libnrp_parse_nrp_hdr(unsigned char *buffer, unsigned char buflen, unsigned char *data, unsigned char *datalen, unsigned char *proto, unsigned char *source_addr, unsigned char *s_addr_type, unsigned char *dest_addr, unsigned char *d_addr_type, unsigned short int *source_port, unsigned short int *dest_port, unsigned short int *signal, unsigned short int *pkt_seq)
{
	int libnrp_index = 0;
	unsigned short int field_len;
	
	unsigned char addrlen;
	unsigned char addr_type;
	
	if(((buffer[3]) & 0xf) == 0x00)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a nRP data buffer.\n");

		for(libnrp_index=4 ; libnrp_index < buflen ;)
		{
			switch (buffer[libnrp_index] & 0x7f)
			{
				case 0x00:
					libnrp_debug(NRP_DEBUG, "Found data tag (0x00).\n");
					field_len = ((uint16_t)buffer[libnrp_index+1]) << 8;
					field_len += ((uint16_t)buffer[libnrp_index+2]);
					
					if(buflen-libnrp_index < field_len)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					if(data == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for data tag.\n");
						break;
					}
					
					libnrp_debug(NRP_DEBUG, "Data length:%u.\n", field_len);
					memcpy(data, &(buffer[libnrp_index+3]), field_len);
					*datalen = (unsigned char)field_len;
					libnrp_index += (field_len + 3);
					break;
			
				case 0x01:
					libnrp_debug(NRP_DEBUG, "Found protocol tag (value:%s) in nRP buffer from serial.\n", (char *)libnrp_proto_table[buffer[libnrp_index+3]]);

					if(proto == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for protocol tag.\n");
						break;
					}

					*proto = buffer[libnrp_index+3];
					
					libnrp_index += 4;					
					break;
			
				case 0x02:
					libnrp_debug(NRP_DEBUG, "Found source address tag (0x02).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					addr_type = buffer[libnrp_index+3];

					if(buflen-libnrp_index < addrlen)
					{
						libnrp_debug(NRP_DEBUG, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}

					if(source_addr == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for source adddress tag.\n");
						break;
					}

					if(s_addr_type == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for source adddress type tag.\n");
						break;
					}

					*s_addr_type = addr_type;

					if(addr_type == 0x10)
					{
						libnrp_debug(NRP_DEBUG, "Source address in string format 0x%.2x.\n", addr_type);
						memcpy(source_addr, &(buffer[libnrp_index+4]), addrlen);
						libnrp_debug(NRP_DEBUG, "Source address:%s\n", source_addr);
					}
					else
					{
						libnrp_debug(NRP_DEBUG, "Source address is in format 0x%.2x.\n", addr_type);
						libnrp_debug(NRP_DEBUG, "Address type:(0x%.2x) -> %s\n", addr_type, (char *)libnrp_proto_table[addr_type]);

						memcpy(source_addr, &(buffer[libnrp_index+4]), addrlen);
					}
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x03:
					libnrp_debug(NRP_DEBUG, "Found destination address tag (0x03).\n");
					addrlen = buffer[libnrp_index+1] << 8;
					addrlen += (buffer[libnrp_index+2] - 1);
					addr_type = buffer[libnrp_index+3];

					if(dest_addr == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for destination adddress tag.\n");
						break;
					}

					if(d_addr_type == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for destination adddress type tag.\n");
						break;
					}

					*d_addr_type = addr_type;

					if(addr_type == 0x10)
					{
						libnrp_debug(NRP_DEBUG, "Destination address in string format 0x%.2x.\n", addr_type);
						memcpy(dest_addr, &(buffer[libnrp_index+4]), addrlen);
						libnrp_debug(NRP_DEBUG, "Destination address:%s\n", dest_addr);
					}
					else
					{
						libnrp_debug(NRP_DEBUG, "Destination address is in format 0x%.2x.\n", addr_type);
						memcpy(dest_addr, &(buffer[libnrp_index+4]), addrlen);
					}	 
					
					libnrp_index += (addrlen + 4);
					break;

				case 0x04:
					libnrp_debug(NRP_DEBUG, "Found source port tag (0x04).\n");
					
					if(source_port == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for source port number tag.\n");
						break;
					}
					
					*source_port = buffer[libnrp_index+3] << 8;
					*source_port += buffer[libnrp_index+4];
					libnrp_debug(NRP_DEBUG, "Source port:%d\n" , source_port);
					
					libnrp_index += 5;
					break;

				case 0x05:
					libnrp_debug(NRP_DEBUG, "Found destination port tag (0x05).\n");
					
					if(dest_port == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for destination port number tag.\n");
						break;
					}
					
					*dest_port = buffer[libnrp_index+3] << 8;
					dest_port += buffer[libnrp_index+4];
					libnrp_debug(NRP_DEBUG, "Destination port:%d\n" , dest_port);
					
					libnrp_index += 5;
					break;

				case 0x06:
					libnrp_debug(NRP_DEBUG, "Found signal level tag (0x06).\n");
					
					if(signal == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for signal level tag.\n");
						break;
					}
					
					memcpy(signal, &(buffer[libnrp_index+3]), 2);
					*signal = ntohs(*signal);
					
					libnrp_debug(NRP_DEBUG, "Signal level:0x%.4x\n", signal);
					libnrp_debug(NRP_DEBUG, "Signal level:%d\n", signal);
					
					libnrp_index += 5;
					break;

				case 0x07:
					libnrp_debug(NRP_DEBUG, "Found packet sequence ID tag (0x07).\n");
					
					if(pkt_seq == NULL)
					{
						libnrp_debug(NRP_DEBUG, "No memory allocated for packet sequence number tag.\n");
						break;
					}
					
					memcpy(pkt_seq, &(buffer[libnrp_index+3]), 2);
					
					*pkt_seq = ntohs(*pkt_seq);
					
					libnrp_debug(NRP_DEBUG, "Packet ID:%d\n", pkt_seq);
					
					libnrp_index += 5;
					break;
			}
		}
	}
	else if(((buffer[3]) & 0xf) == 0x01)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a RF configuration request packet.\n");
		return(0);
	}
	else if(((buffer[3]) & 0xf) == 0x02)
	{
		libnrp_debug(NRP_DEBUG, "Looks like a RF configuration reply packet.\n");
		return(0);
	}
	
	return(0);
}
