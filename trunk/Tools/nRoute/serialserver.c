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


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#define __USE_GNU
#include <sys/ipc.h>
#include <sys/msg.h>


#ifdef __APPLE__
#include <sys/select.h>
typedef struct msgbuf
{
	long mtype;
	char mtext[1];
}msgbuf;
#else
#include <sys/poll.h>
#endif /* APPLE */

#ifdef __CYGWIN__
typedef struct msgbuf
{
	long mtype;
	char mtext[1];
}msgbuf;
#endif

#define _GNU_SOURCE
#include <string.h>

#ifndef _SERIALSERVER_H
#include "serialserver.h"
#endif

#ifndef NRP_H
#include "nrp.h"
#endif

#ifndef _PORT_H
#include "port.h"
#endif


/** @file serialserver.c
 *	Serial interface server functionality.
 *
 *	All the serial port server functionality is implmented in this file. The file also contains the buffer manipulation functions.
 *
 */

extern struct nRdconfig_t nRdconf;
extern struct port_t *nRdport;
extern FILE *serialfp;
extern int haltsystem;

extern unsigned char nrp_proto_table[32][64];
extern unsigned char nrp_addr_table[32][128];

extern void logger(int loglevel, const char *tolog, ...);
extern void rawlogger(int loglevel, const char *tolog, ...);


extern char nRouted_lockfile[64];

int conf_state = 0;

//extern int init_serial_buffers(void);

#define MAX_SERIAL_READ 512

/** The serial server.
 *
 *	This function implements the serial port server which listens to data arriving from radio module.
 *
 */
void serial_server(void)
{
	unsigned char packet_tmp[MAX_SERIAL_READ];
	unsigned packet_len = 0;
	unsigned char packet[MAX_SERIAL_READ];
	unsigned int config = 0;
	unsigned int tag_len = 0;
	unsigned int tag_info = 0;
	unsigned int last = 0;
	unsigned int header = 0;
	
#ifdef __APPLE__
	fd_set fds;
	struct timeval tv;
#else
	struct pollfd fds;
#endif /* APPLE */
	unsigned int nfds = 1;
	int timeout = 100;
	int rval = 0;
		
	unsigned char NRP_data_hdr[4] = { 0x4e, 0x52, 0x50, 0x00 };
	unsigned char NRP_config_query_hdr[4] = { 0x4e, 0x52, 0x50, 0x01 };
	unsigned char NRP_config_hdr[4] = { 0x4e, 0x52, 0x50, 0x02 };
	unsigned char NRP_MCU_hdr[4] = { 0x4e, 0x52, 0x50, 0x00 };
	
	unsigned char register_nrp[64];

	conf_state = 0;
	
	NRP_data_hdr[3] = NRP_VERSION << 4;
	NRP_config_hdr[3] += NRP_VERSION << 4;
	NRP_MCU_hdr[3] = NRP_VERSION << 4;
	NRP_MCU_hdr[3] += 0x0f;
	
	logger(1, "Serial server started, looks good.\n");

	/** @todo Parts of the serial server is yet to be implemented. -mjs */
	/** @todo Dongle initialization has not been implemented. Need a dongle for this. -mjs */

	if((rval = init_serial_buffers()) != 0)
	{
		logger(0, "Failed to initialize serial buffer(s). Exiting.\n");
		kill(0, SIGINT);
		pthread_exit(NULL);
	}

	if(port_open(nRdconf.serialport) == -1)
	{
		kill(0, SIGINT);
		pthread_exit(NULL);
	}

	set_port_params();

	bzero(packet_tmp, MAX_SERIAL_READ);

#ifdef __APPLE__
	FD_ZERO(&fds);
	FD_SET((int)(nRdport->handle), &fds);

	tv.tv_sec = 0;
	tv.tv_usec = timeout;
	
	while((rval = select(1, &fds, NULL, NULL, &tv)) >= 0 && haltsystem != 1)
	{
#else
	fds.fd = (int)(nRdport->handle);
	fds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

	bzero(packet_tmp, MAX_SERIAL_READ);

	sleep(1);

	while((rval = poll(&fds, nfds, timeout)) >= 0 && haltsystem != 1)
	{

#endif /* APPLE */
		if(rval == 0)
		{
			bzero(packet, MAX_SERIAL_READ);

			if (conf_state < 10)
			{
				if ((conf_state++ & 3) == 0)
				{
					/* nRP module reset. */
					memcpy(register_nrp, NRP_config_query_hdr, 4);
					register_nrp[4] = 0x87;
					register_nrp[5] = 0x00;
					register_nrp[6] = 0x00;

					if (monitor_mode)
					{
						printf("Writing reset.\n");
					}
					port_write(register_nrp, 7);
				}

				if (conf_state > 9)
				{
					logger(0,"No module found.\n");
					if (monitor_mode)
					{
						printf("No module found.\n");
					}
					haltsystem = 1;
					kill(0, SIGINT);
				}
			}
			else if (conf_state < 30)
			{
				if ((conf_state++ & 3) == 0)
				{
					/* Register with nRP module to receive protocol packets. */
					register_nrp[4] = 0x03;
					register_nrp[5] = 0x00;
					register_nrp[6] = 0x00;
					register_nrp[7] = 0x81;
					register_nrp[8] = 0x00;
					register_nrp[9] = 0x01;
					register_nrp[10] = nRdconf.protocol;
					
					if (monitor_mode)
					{
						printf("Writing config with protocol %d.\n", nRdconf.protocol);
					}
					
					usleep(100000);
					
					port_write(register_nrp, 11);
				}
				if (conf_state > 20)
				{
					logger(0,"Module config failed.\n");
					if (monitor_mode)
					{
						printf("Module config failed.\n");
					}
					haltsystem = 1;
					kill(0, SIGINT);
				}
			}
			else if (conf_state < 40)
			{
				if ((conf_state++ & 3) == 0)
				{
					/* Get MAC. */
					register_nrp[4] = 0x91;
					register_nrp[5] = 0x00;
					register_nrp[6] = 0x00;

					if (monitor_mode)
					{
						printf("Getting MAC.\n");
					}
					
					usleep(100000);
					
					port_write(register_nrp, 11);
				}
				if (conf_state > 38)
				{
					logger(0,"MAC fetch failed.\n");
					if (monitor_mode)
					{
						printf("MAC fetch failed.\n");
					}
					conf_state = 40;
				}
			}
		}
		else if(fds.revents == POLLIN || fds.revents == POLLPRI)
		{
#ifndef __APPLE__
			switch (fds.revents)
			{
				case POLLIN:
#endif /* !APPLE */
					
					if (port_read(&(packet_tmp[packet_len]), 1) == 0)
					{
						break;
					}
					else
					{					
						packet_len++;
						if (header < 4)
						{
							header++;
							if (memcmp(packet_tmp, NRP_config_hdr, header) == 0)
							{
								if (header == 4)
								{ logger(2, "nRP config reply header found.\n");
									config = 1;
									tag_len = 0;
									tag_info = 0;
									last = 0;
								}
							}
							else if (memcmp(packet_tmp, NRP_data_hdr, header) == 0)
							{
								if (header == 4)
								{
									logger(2, "nRP data packet header found.\n");
									config = 0;
									tag_len = 0;
									tag_info = 0;
									last = 0;
								}
							}
							else if(memcmp(packet_tmp, NRP_MCU_hdr, header) == 0)
							{
								if (header == 4)
								{
									logger(2, "Recognized MCU debug header.\n");
									tag_info = 3;
									tag_len = 1;
									config = 2;
									last = 1;
								}
							}
							else
							{
									logger(2, "invalid packet header found, %d bytes.\n", header);
									printf("Header: %2.2X %2.2X %2.2X %2.2X\n", 
											packet_tmp[0], packet_tmp[1], packet_tmp[2], packet_tmp[3]);
									header = 0;
									packet_len = 0;
									last = 0;
									bzero(packet_tmp, MAX_SERIAL_READ);
							}
						}
						else if (tag_info < 3)
						{
							tag_info++;
							if (tag_info == 3)
							{
								tag_len = packet_tmp[packet_len-2]*256 + packet_tmp[packet_len-1];
								logger(2, "Tag 0x%2.2X with length %d.\n", packet_tmp[packet_len-3], tag_len);
								if (packet_tmp[packet_len-3] & 0x80)
								{
									logger(2, " Last tag.\n");
									last = 1;
								}
								else
								{
									logger(2, "\n");
									last = 0;
								}
							}
						}
						else
						{
							tag_len--;
							if (config == 2)
							{	/*read debug string until zero*/
								tag_len = packet_tmp[packet_len-1];
							}
							if (tag_len == 0)
							{
								tag_info = 0;
								if (last)
								{
									logger(2, "Packet length %d.\n", packet_len);
									memcpy(packet, packet_tmp, packet_len);
									nrp_from_serial(packet, packet_len);
									bzero(packet_tmp, MAX_SERIAL_READ);
									packet_len = 0;
									header = 0;
								}
							}
						}
					}
#ifndef __APPLE__
					break;
				case POLLPRI:
					logger(2, "Serial port poll: POLLPRI\n");
					break;
				case POLLERR:
					logger(1, "Serial port poll: POLLERR\n");
					break;
				case POLLHUP:
					logger(1, "Serial port poll: POLLHUP\n");
					break;
				case POLLNVAL:
					logger(1, "Serial port poll: POLLNVAL\n");
					perror("SERIALSERVER (POLLINVAL):");
					break;
				default:
					logger(3, "Default poll...\n");

			}
#endif /* !APPLE */
		}
	}
	pthread_exit(NULL);
}







int nrp_from_serial(unsigned char packet[MAX_NRP_PACKET_SIZE], int len)
{
	int sockfd;

	unsigned char data[128];
	unsigned char proto;
	
	unsigned char addr_type;
	unsigned char source_addr[15];
	unsigned char dest_addr[15];
	char dest_addr_str[15];
	unsigned int addrlen;
	unsigned int sport = 0;
	unsigned int dport = 0;
	short int signal_lvl;
	unsigned int packet_id;
	unsigned int i;
	int idx;
	unsigned short int field_len=0;

/*
 *	First check if we can find all the mandatory Tags & fields from the nRP packet and if this actually is a nRP packet. Open sockets etc. only after that.
 */
/*	if(CHECK_NRP_HDR(packet) == 1)
	{
		logger(2, "Received other than nRP packet from serial device.\n");
		return(-1);
	}

	if(((packet[3])>>4) != NRP_VERSION)
	{
		logger(2, "Received a nRP packet, but wrong version (0x%u)\n", packet[3]);
		return(-1);
	}

	logger(2, "nRP version check passed.\n");
*/
	/** @todo Implement the Type ID 1 & 2 handling! -mjs */
	i = 0;
	while(i<len)
	{
		char line[(16*3) + 12];
		char tmp_string[4];
		
		if (i == 0)
		{
			sprintf(line, "data[%d]: ", i);
		}
		else if ((i & 0x0F) == 0)
		{
			logger(2, "%s\n", line);
			sprintf(line, "data[%d]: ", i);
		}

		sprintf(tmp_string, "%2.2X", packet[i]);
		strcat(line, tmp_string);
		i++;
		
		if (i < len)
		{ 
			strcat(line, ":");
		}
		else
		{	/* last byte, write out last line */
			logger(2, "%s\n", line);
		}		
	}


	if(((packet[3]) & 0xf) == 0x00)
	{
		unsigned char end = 0;
		
		logger(2, "Looks like a nRP data packet.\n");

		for(idx=4 ; idx < len ;)
		{
			logger(2, "Going to switch() statement with idx:%d/%d and packet[idx]:0x%.2x\n", idx, len, packet[idx]);
			if (packet[idx] & 0x80)
			{
				logger(2, "Last tag.\n"); 
				end = 1;
			}
			switch (packet[idx] & 0x7f)
			{
				case 0x00:
					logger(2, "Found data tag (0x00).\n");
					field_len = ((uint16_t)packet[idx+1]) << 8;
					field_len += ((uint16_t)packet[idx+2]);
					
					if(len-idx < field_len)
					{
						logger(0, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}
					
					logger(2, "Data length:%u.\n", field_len);
					memcpy(data, &(packet[idx+3]), field_len);
					if (monitor_mode)
					{
						printf("Data field: %d bytes.\n", field_len);
						hex_dump(data, field_len);
					}
					idx += (field_len + 3);
					break;
			
				case 0x01:
					logger(2, "Found protocol tag (value:%s) in nRP packet from serial.\n", (char *)nrp_proto_table[packet[idx+3]]);
					memcpy(&proto, &(packet[idx+3]), 1);
					
					idx += 4;					
					if (monitor_mode)
					{
						printf("Protocol: %s.\n", (char *)nrp_proto_table[packet[idx-1]]);
						hex_dump(data, field_len);
					}
					break;
			
				case 0x02:
					logger(2, "Found source address tag (0x02).\n");
					addrlen = packet[idx+1] << 8;
					addrlen += (packet[idx+2] - 1);
					addr_type = packet[idx+3];

					if(len-idx < addrlen)
					{
						logger(0, "Invalid length field in the nRP packet (excessive length). Discarding packet.\n");
						return(-1);
					}

					if(addr_type == 0x10)
					{
						logger(2, "Source address in string format 0x%.2x.\n", addr_type);
//						strncpy(source_addr_str, &(packet[idx+4]), addrlen);
						memcpy(source_addr, &(packet[idx+4]), addrlen);
						logger(2, "Source address:%s\n", source_addr);
						if (monitor_mode)
						{
							printf("Source address in string format 0x%.2x=%s.\n", addr_type, source_addr);
						}
					}
					else
					{
						logger(2, "Source address is in format 0x%.2x.\n", addr_type);
						logger(2, "Address type:(0x%.2x) -> %s\n", addr_type, (char *)nrp_addr_table[addr_type]);

						memcpy(source_addr, &(packet[idx+4]), addrlen);
						if (monitor_mode)
						{
							printf("Source type:(0x%.2x) ", addr_type);
							hex_dump(source_addr, addrlen);
						}
					}
					
					idx += (addrlen + 4);
					break;

				case 0x03:
					logger(2, "Found destination address tag (0x03).\n");
					addrlen = packet[idx+1] << 8;
					addrlen += (packet[idx+2] - 1);
					addr_type = packet[idx+3];

					if(addr_type == 0x10)
					{
						logger(2, "Destination address in string format 0x%.2x.\n", addr_type);
//						strncpy(dest_addr_str, &(packet[idx+4]), addrlen);
						memcpy(dest_addr, &(packet[idx+4]), addrlen);
						logger(2, "Destination address:%s\n", dest_addr_str);
						if (monitor_mode)
						{
							printf("Destination address in string format 0x%.2x=%s.\n", addr_type, dest_addr);
						}
					}
					else
					{
						logger(2, "Destination address is in format 0x%.2x.\n", addr_type);
						memcpy(dest_addr, &(packet[idx+4]), addrlen);
						if (monitor_mode)
						{
							printf("Destination type:(0x%.2x) ", addr_type);
							hex_dump(dest_addr, addrlen);
						}
					}	 
					
					idx += (addrlen + 4);
					break;

				case 0x04:
					logger(2, "Found source port tag (0x04).\n");
					
					sport = packet[idx+3] << 8;
					sport += packet[idx+4];
					logger(2, "Source port:%d\n" , sport);
					if (monitor_mode)
					{
						printf("Source port:%d\n" , sport);
					}
					
					idx += 5;
					break;

				case 0x05:
					logger(2, "Found destination port tag (0x05).\n");
					
					dport = packet[idx+3] << 8;
					dport += packet[idx+4];
					logger(2, "Destination port:%d\n" , dport);
					if (monitor_mode)
					{
						printf("Dest port:%d\n" , dport);
					}
					
					idx += 5;
					break;

				case 0x06:
					logger(2, "Found signal level tag (0x06).\n");
					
					memcpy(&signal_lvl, &(packet[idx+3]), 2);
					signal_lvl = ntohs(signal_lvl);
//					signal_lvl = packet[idx+3] << 8;
//					signal_lvl += packet[idx+4];
					
					logger(2, "Signal level:0x%.4x\n", signal_lvl);
					logger(2, "Signal level:%d\n", signal_lvl);
					
					if (monitor_mode)
					{
						printf("Signal level: %d\n" , signal_lvl);
					}
					
					idx += 5;
					break;

				case 0x07:
					logger(2, "Found packet sequence ID tag (0x07).\n");
					
					packet_id = packet[idx+3] << 8;
					packet_id += packet[idx+4];
					
					logger(2, "Packet ID:%d\n", packet_id);
					if (monitor_mode)
					{
						printf("Packet ID:%d\n", packet_id);
					}
					idx += 5;
					break;
			}
			if (end) break;
		}
	}
	else if(((packet[3]) & 0xf) == 0x01)
	{
		logger(2, "Looks like a RF configuration request packet.\n");
		if (monitor_mode)
		{
			printf("Config request: %d\n", packet[4]);
			hex_dump(packet, len);
		}
	}
	else if(((packet[3]) & 0xf) == 0x02)
	{
		logger(2, "Looks like a RF configuration reply packet.\n");
		if (monitor_mode)
		{
			printf("Config reply: %d\n", packet[4]);
			hex_dump(packet, len);
		}
		if ((packet[4] & 0x7F) == 0x07)
		{
			conf_state = 10;
			if (monitor_mode)
			{
				printf("Module reset received: module present.\n");
			}
		}
		if ((packet[4] & 0x7F) == 0x03)
		{
			conf_state = 30;
			if (monitor_mode)
			{
				printf("Module config received: module configured.\n");
			}
		}
		if ((packet[4] & 0x7F) == 0x11)
		{
			conf_state = 40;
			if (monitor_mode)
			{
				printf("Module MAC received.\n");
			}
		}
	}

	for(i=0;i<nRdconf.tcpmaxconn;i++)
	{
		logger(2, "Trying nRd_conn_table[%d]\n", i);
		if(nRd_conn_table[i].active == 1)
		{
			logger(2, "This was active.\n");
			if (monitor_mode)
			{
				printf("Protocol %d vs %d.\n", proto, nRd_conn_table[i].proto);
			}
			if(nRd_conn_table[i].proto == 0xFF || nRd_conn_table[i].proto == proto)
			{
				int source_match = 0;
				int dest_match = 0;				
				int saddr_match = 0;
				
				logger(2, "No protocol filter set or match found.\n");
				if( nRd_conn_table[i].proto != 0xFF && 
				   nRd_conn_table[i].proto != PROTO_IEEE_802_15_4_RAW && 
					 nRd_conn_table[i].proto != PROTO_NUDP && 
					 nRd_conn_table[i].proto != PROTO_6LOWPAN && 
					 nRd_conn_table[i].proto != PROTO_6LOWPAN_ICMP)
				{
					logger(2, "WARNING: UNSUPPORTED PROTOCOL TYPE 0x%2x RECEIVED!\n", proto);
				}
				if (monitor_mode)
				{
					printf("Source port %d vs %d.\n", sport, nRd_conn_table[i].source_port);
				}
				if(nRd_conn_table[i].source_port == 0xFFFF || nRd_conn_table[i].source_port == sport)
				{
					logger(2, "No source port filter set or a match found.\n");
					source_match = 1;
				}
				if (monitor_mode)
				{
					printf("Destination port %d vs %d.\n", dport, nRd_conn_table[i].destination_port);
				}
				if(nRd_conn_table[i].destination_port == 0xFFFF || nRd_conn_table[i].destination_port == dport)
				{
					logger(2, "No destination port filter set or a match found.\n");
					dest_match = 1;
				}
				/** @todo Add support for other address types. The only supported at the moment is 802.15.4 long address 8 bytes. -mjs */
				if(nRd_conn_table[i].address_type == 0xFE || memcmp(source_addr, nRd_conn_table[i].addr.addr_15_4_long, 8) == 0 || CHECK_8_BYTE_BCAST(nRd_conn_table[i].addr.addr_15_4_long))
				{
					logger(2, "No address type filter set or source address match found.\n");
					saddr_match = 1;
				}
				if (saddr_match && dest_match && source_match)
				{
					// Packet matched all filters! Relay it!!!
					logger(2, "Trying to send the packet to remote host.\n");
					sockfd = nRd_conn_table[i].fd;

					/*	If the protocol type is 0x00 (802.15.4 MAC raw),
					   0x01 (nUDP) or 0x02 (6lowpan)
						 then send the whole packet, in other cases send just the data */
					if(proto == PROTO_IEEE_802_15_4_RAW || 
					   proto == PROTO_NUDP || 
						 proto == PROTO_6LOWPAN)
						write(sockfd, packet, len);
					else
						write(sockfd, data, field_len);
				}
			}
		}	
	}

	logger(2, "Returning to serial_server()\n");

	return(1);
}

void serial_write(void)
{
	struct msgbuf *serial_msg;
	unsigned char rcvbuf[MAX_NRP_PACKET_SIZE];
	int msgsz;
	
	serial_queue_id = msgget(ftok(nRouted_lockfile, SERIAL_ID), IPC_CREAT | 0666);
	
	serial_msg = (struct msgbuf *)malloc((sizeof(struct msgbuf) + MAX_NRP_PACKET_SIZE));

	/*	Waiting untill the nRP module is configured properly. */
	while(conf_state < 30 && haltsystem != 1)
	{
		usleep(500000);
	}

	logger(2, "serial_write:ready for data.\n");

	while(haltsystem != 1)
	{
		usleep(1);
		
		/* The msgrcv() function call is made blocking by not having the last argument IPC_NOWAIT. As the fourth argument
		 *	is zero, the function call reads the first message in queue.
		 */
		if((msgsz = msgrcv(serial_queue_id, serial_msg, MAX_NRP_PACKET_SIZE, 0, 0)) > 0)
		{
			memcpy(rcvbuf, &(serial_msg->mtext[0]), MAX_NRP_PACKET_SIZE);

			port_write((unsigned char *)rcvbuf, msgsz);
			
			bzero(serial_msg, sizeof(struct msgbuf) + MAX_NRP_PACKET_SIZE);
		}
		else
		{
			// logger(2, "serial_write:WARNING - Failed to receive a message from queue!\n");
		}
	}
	
	free(serial_msg);
	logger(2, "serial_write: exit.\n");
	
	return;
}

int init_serial_buffers(void)
{
	int i;

	/*	Allocate memory for the serial send buffer. */
	serial_send_buffer.fifo_ring = (struct packet_entry_t *)malloc(nRdconf.serialbufsize * sizeof(struct packet_entry_t));
	if(serial_send_buffer.fifo_ring == NULL)
	{
		logger(0, "Failed to allocate memory for serial send buffer(s).\n");
		return(-1);
	}
	/*	Now set the bottom and top buffers to point to the same element in the buffer.
	 *	It should be noted that the only situation when top and bottom pointers point to the same memory address is when the
	 *	buffer is empty. When the buffer is full, the bottom->next should point to the same element as top pointer.
	 */
	serial_send_buffer.bottom = &(serial_send_buffer.fifo_ring[0]);
	serial_send_buffer.top = &(serial_send_buffer.fifo_ring[0]);

	/*	We set the pointers of the "first" packet_entry_t manually. Because of this we can avoid having two extra if() 
	 *	statements inside the for() loop below. That's why the for-loop looks a bit weird/ugly.
	 *	We will also set the "last" entrys pointers manually below. We will also bzero() all the packet data arrays. -mjs
	 */
	serial_send_buffer.fifo_ring[0].next = &(serial_send_buffer.fifo_ring[1]);
	serial_send_buffer.fifo_ring[0].prev = &(serial_send_buffer.fifo_ring[nRdconf.serialbufsize-1]);

	bzero(serial_send_buffer.fifo_ring[0].packet_data, MAX_NRP_PACKET_SIZE);

	for(i=1;i<nRdconf.serialbufsize-2;i++)
	{
		serial_send_buffer.fifo_ring[i].next = &(serial_send_buffer.fifo_ring[i+1]);
		serial_send_buffer.fifo_ring[i].prev = &(serial_send_buffer.fifo_ring[i-1]);

		bzero(serial_send_buffer.fifo_ring[i].packet_data, MAX_NRP_PACKET_SIZE);
	}

	serial_send_buffer.fifo_ring[nRdconf.serialbufsize-1].next = &(serial_send_buffer.fifo_ring[0]);
	serial_send_buffer.fifo_ring[nRdconf.serialbufsize-1].prev = &(serial_send_buffer.fifo_ring[nRdconf.serialbufsize-2]);

	bzero(serial_send_buffer.fifo_ring[nRdconf.serialbufsize-1].packet_data, MAX_NRP_PACKET_SIZE);

	logger(1, "Serial send buffer(s) initialized succesfully.\n");

	return(0);
}


int add_packet(struct fifo_buffer_t *bufaddr, char packet[MAX_NRP_PACKET_SIZE])
{
	/*	First check if the buffer is not full. */
	if(bufaddr->top->next != bufaddr->bottom)
	{
		/*	Copy data to the buffer and adjust top pointer. */
		memcpy(bufaddr->top->next->packet_data, packet, MAX_NRP_PACKET_SIZE);
		bufaddr->top = bufaddr->top->next;

		/*	If the packet is the only packet in the buffer we must also adjust the bottom pointer. */
		if(bufaddr->top->prev == bufaddr->bottom)
			bufaddr->bottom = bufaddr->top;
		
		logger(2, "New packet succesfully added to serial send buffer.\n");

		return(0);
	}
	else
	{
		logger(1, "The serial send buffer is full! Dropping packet, sorry.\n"); 
		logger(1, "If you have at times heavy and very peaky nRP traffic to the serial interface, try increasing SERIALBUFSIZE.\n");
		
		return(-1);
	}
};

int get_bottom_packet(struct fifo_buffer_t *bufaddr, char *packet)
{

	/*	Check if there is any packets in the buffer. */
	if(bufaddr->top != bufaddr->bottom)
	{
		/*	Copy data. */
		memcpy(packet, bufaddr->bottom->packet_data, MAX_NRP_PACKET_SIZE);

		logger(2, "Retrieved the bottom packet from serial send buffer.\n");
		return(0);
	}
	
	logger(2, "Serial send buffer was empty, failed to retrieve packet.\n");

	return(1);
};

int purge_bottom_packet(struct fifo_buffer_t *bufaddr)
{
	if(bufaddr->top != bufaddr->bottom)
	{
		/*	bzero() slot and adjust bottom pointer. */
		bzero(bufaddr->bottom->packet_data, MAX_NRP_PACKET_SIZE);
		bufaddr->bottom = bufaddr->bottom->next;

		logger(2, "Purged the bottom packet from serial send buffer.\n");
		return(0);
	}

	logger(2, "Serial send buffer was empty, failed to purge packet.\n");

	return(1);
};

int purge_packet(struct fifo_buffer_t *bufaddr, char packet_header[25])
{

	/** @todo This function is yet to be implmented -mjs. */

	return(0);
};

int clear_buffer(struct fifo_buffer_t *bufaddr)
{
	int i = 0;

	/*	Check if the buffer is already empty. */	
	if(bufaddr->top != bufaddr->bottom)
	{
		/*	Clear the buffer entries from bottom to top. */
		while(bufaddr->bottom != bufaddr->top)
		{
			bzero(bufaddr->bottom->packet_data, MAX_NRP_PACKET_SIZE);
			bufaddr->bottom = bufaddr->bottom->next;

			i++;
		}
		return(i);
	}

	/*	If the buffer was already empty, return 0. */
	return(0);
};

int send_to_serial(struct fifo_buffer_t *bufaddr, char packet[MAX_NRP_PACKET_SIZE])
{
	return(0);
};
