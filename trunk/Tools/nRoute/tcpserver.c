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
#include <arpa/inet.h>

#include <sys/poll.h>

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#define __USE_GNU
#include <sys/ipc.h>

#include <sys/msg.h>

#ifdef __APPLE__
typedef struct msgbuf
{
	long mtype;
	char mtext[1];
}msgbuf;
#endif

#ifdef __CYGWIN__
typedef struct msgbuf
{
	long mtype;
	char mtext[1];
}msgbuf;
#endif

#include "nRouted.h"

#ifndef _PORT_H

#include "port.h"

#endif /* _PORT_H */

#include "tcpserver.h"
#include "nrp.h"

/** @file tcpserver.c
 *	TCP server functionality.
 *
 *	All the TCP server functionality is implmented in this file.
 *
 */

extern struct nRdconfig_t nRdconf;
extern unsigned char nrp_proto_table[32][64];
extern unsigned char nrp_addr_table[32][128];
extern int haltsystem;

extern char nRouted_lockfile[64];

extern void logger(int loglevel, const char *tolog, ...);

void *process_tcp_connection(void *fd);

int tcpconns=0;

/** The TCP server.
 *
 *	This function creates a simple TCP server which listens to specified port at address 127.0.0.1.
 *
 */
int tcp_server(void)
{
	int serverfd;
	int *fdptr = NULL;
	
	socklen_t len;
	struct sockaddr_in serveraddr;
	
	pthread_t handler_t;
	
	int rval;

	logger(1, "TCP server started, looks good.\n");

	if((serverfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		logger(0, "Failed to create a socket for the TCP server.\n");
		return(-1);
	}

	bzero(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	inet_pton(AF_INET, nRdconf.tcpaddr, &serveraddr.sin_addr.s_addr);
	serveraddr.sin_port = htons(nRdconf.tcpport);

	if((rval = bind(serverfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr))) != 0)
	{
		logger(0, "Failed to bind the socket to an address.\n");
		return(-1);
	}

/*	Here we change the socket in to listen state and define the backlog to be the value defined in the configuration file. */
	if((rval =listen(serverfd, nRdconf.tcpmaxconn)) != 0)
	{
		logger(0, "Failed to change the socket in to listen state.\n");
		return(-1);
	}

	/** @todo Must make sure that the server notices when the peer hangs/drops the connection and acts properly. -mjs */
	while(haltsystem != 1)
	{
		len = sizeof(serveraddr);
		fdptr = (int *)malloc(sizeof(int));
		*fdptr = accept(serverfd, (struct sockaddr *)&serveraddr, &len);
		logger(2, "New fdptr:%d\n", fdptr);
		if(tcpconns < nRdconf.tcpmaxconn && haltsystem != 1)
		{
			tcpconns++;
			logger(2, "Creating new server thread...\n");
			pthread_create(&handler_t, NULL, &process_tcp_connection, fdptr);
		}
		else
		{
			logger(0, "No more TCP connection slots available (try adjusting TCPMAXCONN) or nRoute stopping.\n");
		}
	}

/*	Close the server socket before exiting. */
	close(serverfd);

	free(fdptr);

	return(1);
}

/** The TCP connection handler function.
 *
 *	This function handles the connection with the peer/client in the network.
 *
 *	@param file pointer to the TCP socket
 *	@fn static void *process_tcp_connection(void *fd)
 */
void *process_tcp_connection(void *fd)
{
	int connfd;
	unsigned char readbuf[MAX_NRP_PACKET_SIZE];
	
	unsigned char conf_reply_ok[9] = { 0x4E, 0x52, 0x50, 0x03, 0x05, 0x00, 0x02, 0x4F, 0x4B };
	unsigned char conf_reply_fail[11] = { 0x4E, 0x52, 0x50, 0x03, 0x05, 0x00, 0x02, 0x46, 0x41, 0x49, 0x4C };

	struct pollfd fds;
	unsigned int nfds = 1;

	int i;
	int rval=0;
	int conn_index=0;
	int idx=4;
	int got_last_tag=0;
	
	struct msgbuf *ser_buf;
	int msgid;
	
	unsigned int field_len;
	
	logger(2, "New TCP conn. handler created successfully!\n");
	
	connfd = *((int *) fd);
	free(fd);

	pthread_detach(pthread_self());

	/*	Set the poll() parameters. */
	fds.fd = connfd;
	fds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

	/*	Allocate memory for the message buffer.	*/
	ser_buf = (struct msgbuf *)malloc(sizeof(struct msgbuf) + MAX_NRP_PACKET_SIZE);

	/*	Set the mtype. It really doesn't matter what the mtype is as the serial_write() function reads
	 *	all the messages from the queue.
	 */
	ser_buf->mtype = 1;

	/*	Get the message queue ID.	*/
	msgid = msgget(ftok(nRouted_lockfile, SERIAL_ID), IPC_CREAT | 0666);

/* Here we reserve and fill in initial information to a connection array. */

	logger(2, "Looking for free slot in connection table.\n");
	for(conn_index=0;conn_index<nRdconf.tcpmaxconn;conn_index++)
	{
		if(nRd_conn_table[conn_index].fd == 0)
		{
			logger(2, "Found free slot with index %d.\n", conn_index);
			nRd_conn_table[conn_index].fd = connfd;
			break;
		}
	}

	if(conn_index == nRdconf.tcpmaxconn)
	{
		logger(0, "Could not find a free slot from connection table even if the internal variable (int tcpconns) indicated that there is room left!\n");
		logger(0, "The previous error message might indicate a bug that really needs to be fixed.\n");
		logger(0, "Please report the bug (with as much as info as possible) to developers.\n");
		logger(0, "Must drop this connection, sorry.\n");

		tcpconns--;
		
		return(NULL);
	}

/** @todo The connection configuration packet timeout needs to be implemented. -mjs */

/*	We first set the socket into non-blocking state so that we can do the timeout for the configuration packet really easy. We'll find a more elagant
	way to do this in future releases. */

	fcntl(connfd, O_NONBLOCK);
	
	bzero(readbuf, MAX_NRP_PACKET_SIZE);

/*	Now we must wait for the configuration packet from the remote application. We have a 3 second timeout here. If the configuration packet is not 
 *	received within that time period, the connection is dropped. 
 *	The configuration packet is used to fill the nRd_conn_table element associated to this socket/file descriptor. The packet format is reused 
 *	from the nRP packet format. See more details from
 *	the nanoStack manual. -mjs 
 */

	rval = poll(&fds, nfds, 2000);
	if(rval>0)
	{
		/*	We received something, lets read it and check what it is... */
		rval = read(connfd, readbuf, MAX_NRP_PACKET_SIZE);

		for(i=0;i<rval;i++)
		{
			logger(2, "readbuf[%d]:0x%.2x - u:%u - c:%c\n", i, readbuf[i], readbuf[i], readbuf[i]);
		}
	
		logger(2, "Received %d bytes from TCP socket\n", rval);
		i=0;
		/*	Check if the received packet really is the nRoute conf. packet... (check version and Type ID) If it is not a configuration packet, just go
		 *	on and forward the possible packet to its destination. This might happen when some client application just wants to send e.g. control data
		 *	to a sensor node. */
		if(((readbuf[3]) >> 4) == NRP_VERSION && ((readbuf[3]) & 0xf) == 0x03)
		{
			logger(2, "Packet seems to be in nRP format (version and Type ID match)\n");
			/* The packet has Type ID 3 so we activate the connection. */
			nRd_conn_table[conn_index].active = 1;

			 /* Go through the buffer and look for field tags. */
			while(idx<rval && got_last_tag == 0)
			{
				logger(2, "Going to switch() statement with idx:%d and readbuf[%d]:0x%.2x.\n", idx, idx, readbuf[idx]);
				/** @todo The conf. packet parsing below supports only RAW packets with certainty. Needs more work. -mjs */
				/** @todo Must implement logic to decide if all the necessary information has been given to activate the connection. At the moment it is enough to receive a packet which has Type ID 3. -mjs */
				switch ((readbuf[idx]) & 0x7f)
				{
					case 0x00:
						logger(2, "Found protocol tag (value:%s) from nRP connection configuration packet.\n", nrp_proto_table[readbuf[idx+3]]);
						nRd_conn_table[conn_index].proto = readbuf[idx+3];
						
						if((readbuf[idx] >> 7) == 0x01) 
						{
							logger(2, "Last triplet (highest bit of the Tag field was set).\n");
							got_last_tag = 1;
						}

						idx += 4;
						break;
					case 0x01:
						logger(2, "Found source address tag from nRP connection configuration packet.\n");
						field_len = readbuf[idx+1] << 8;
						field_len += readbuf[idx+2];
						
						if((readbuf[idx] >> 7) == 0x01) 
						{
							logger(2, "Last triplet (highest bit of the Tag field was set).\n");
							got_last_tag = 1;
						}
						
						nRd_conn_table[conn_index].address_type = readbuf[idx+3];
						logger(2, "Address type:(0x%.2x) -> %s\n", nRd_conn_table[conn_index].address_type, (char *)nrp_addr_table[nRd_conn_table[conn_index].address_type]);
						
						switch (nRd_conn_table[conn_index].address_type)
						{
							case 0x00:
								if(field_len - 1 == 6)
								{
									memcpy(nRd_conn_table[conn_index].addr.addr_hw, &(readbuf[idx+4]), field_len - 1);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;
							
							case 0x01:
								if(field_len - 1 == 8)
								{
									memcpy(nRd_conn_table[conn_index].addr.addr_15_4_long, &(readbuf[idx+4]), field_len - 1);
									logger(2, "\n\nAddress: 0x%.2x:0x%.2x:0x%.2x:0x%.2x:0x%.2x:0x%.2x:0x%.2x:0x%.2x\n", nRd_conn_table[conn_index].addr.addr_15_4_long[0], nRd_conn_table[conn_index].addr.addr_15_4_long[1], nRd_conn_table[conn_index].addr.addr_15_4_long[2], nRd_conn_table[conn_index].addr.addr_15_4_long[3], nRd_conn_table[conn_index].addr.addr_15_4_long[4], nRd_conn_table[conn_index].addr.addr_15_4_long[5], nRd_conn_table[conn_index].addr.addr_15_4_long[6], nRd_conn_table[conn_index].addr.addr_15_4_long[7]);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;
							
							case 0x02:
								if(field_len - 1 == 2)
								{
									memcpy(nRd_conn_table[conn_index].addr.addr_15_4_short, &(readbuf[idx+4]), field_len - 1);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;
							
							case 0x03:
								if(field_len - 1 == 2)
								{
									memcpy(nRd_conn_table[conn_index].addr.addr_15_4_short, &(readbuf[idx+4]), field_len - 1);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;
								
							case 0x10:
								if(field_len - 1 >= 7 && field_len - 1 <= 15)
								{
									memcpy(nRd_conn_table[conn_index].addr.addr_str, &(readbuf[idx+4]), field_len - 1);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;

							case 0x11:
								if(field_len - 1 == 4)
								{
									memcpy(&(nRd_conn_table[conn_index].addr.addr_32b), &(readbuf[idx+4]), field_len - 1);
								}
								else
								{
									logger(0, "Invalid length field in the nRP packet. Discarding triplet.\n");
									/* Remove the address type entry that was already stored. */
									nRd_conn_table[conn_index].address_type = 0xFE;
								}
								break;
						}

						idx += (field_len+3);
						break;
					case 0x02:
						logger(2, "Found Destination address tag from nRP connection configuration packet.\n");
						field_len = readbuf[idx+1] << 8;
						field_len += readbuf[idx+2];
						
						if((readbuf[idx] >> 7) == 0x01) 
						{
							logger(2, "Last triplet (highest bit of the Tag field was set).\n");
							got_last_tag = 1;
						}
						
						/** @todo Destination address filter needs to be implmented into process_tcp_connection()... -mjs */
						idx += (field_len+3);
						break;
					case 0x03:
						logger(2, "Found source port tag from nRP connection configuration packet.\n");
						field_len = readbuf[idx+1] << 8;
						field_len += readbuf[idx+2];
						nRd_conn_table[conn_index].source_port = readbuf[idx+3] << 8;
						nRd_conn_table[conn_index].source_port += readbuf[idx+4];
						
						if((readbuf[idx] >> 7) == 0x01) 
						{
							logger(2, "Last triplet (highest bit of the Tag field was set).\n");
							got_last_tag = 1;
						}
						
						idx += (field_len+3);
						break;
					case 0x04:
						field_len = readbuf[idx+1] << 8;
						field_len += readbuf[idx+2];
						nRd_conn_table[conn_index].destination_port = readbuf[idx+3] << 8;
						nRd_conn_table[conn_index].destination_port += readbuf[idx+4];
						logger(2, "Found destination port tag (value:0x%.2x) from nRP connection configuration packet.\n", nRd_conn_table[conn_index].destination_port);

						if((readbuf[idx] >> 7) == 0x01) 
						{
							logger(2, "Last triplet (highest bit of the Tag field was set).\n");
							got_last_tag = 1;
						}

						idx += (field_len+3);
						break;
				}
			}
		}
	}
	else if(rval == 0)
	{
		logger(2, "tcpserver:Configuration message from a new client timed out. Dropping connection.\n");
		close(connfd);
		return(NULL);
	}
	else if(rval < 0)
	{
		logger(2, "tcpserver:poll() failed with error:%s\nDropping connection.", strerror(errno));
		close(connfd);
		return(NULL);
	}
	
	
	if(nRd_conn_table[conn_index].active == 1)
	{
		logger(2, "Got enough information from the remote host to activate the connection, sending OK.\n");
		write(connfd, conf_reply_ok, 9);
	}
	else
	{
		logger(2, "Not enough information to activate the connection or parsing failure, sending FAIL.\n");
		write(connfd, conf_reply_fail, 11);
	}

	/** \todo TCP server must use poll()!!! -mjs */

//	if(
	while((rval = read(connfd, readbuf, MAX_NRP_PACKET_SIZE)) > 0 && haltsystem != 1)
	{
		logger(2, "Read %d bytes of data (%s) from TCP socket.\n", rval, readbuf);
		
		/*	Copy the received data into the message buffer. */
		memcpy(ser_buf->mtext, readbuf, rval);

		/*	Send the message to queue. */		
		msgsnd(msgid, ser_buf, rval, IPC_NOWAIT);
	}
	if(rval < 0)
	{
		logger(1, "read() from TCP socket failed!\n");
		perror("TCPSERVER (read):");
	}
	else
	{
		logger(2, "EOF from TCP socket.\n");
	}

	close(connfd);

	logger(2, "TCP connection handler exiting!\n");

	nRd_conn_table[conn_index].active = 0;
	nRd_conn_table[conn_index].fd = 0;
	nRd_conn_table[conn_index].proto = 0xFF;
	nRd_conn_table[conn_index].source_port = 0xFFFF;
	nRd_conn_table[conn_index].destination_port = 0xFFFF;
	nRd_conn_table[conn_index].address_type = 0xFE;
	tcpconns--;

	free(ser_buf);

	return(NULL);
} 
