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


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <strings.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

#include <sys/poll.h>

#include "nPing.h"

#ifndef LIBNRP_H
#include "../libnRP/libnRP.h"
#endif


#define NPING_PORT 0xfd

typedef void (*sighandler_t)(int);

int sockfd;

int sent = 0, received = 0, timeouts = 0;
struct nPing_opts_t nPing_opts = { "127.0.0.1", 21870, {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}, 0, 0, -1, -1, 2};

void usage(void)
{
		printf("nPing version 0.9\n\n");
		printf("USAGE: ./nPING [OPTIONS] [ADDR]\n");
		printf("	Options:\n");
		printf("	-S [nRouted_address:port]	- The address and port number where the nRouted TCP server is located.\n");
		printf("	-c [count]			- The number of nPing packets to send.\n");
		printf("	-i [time]			- NOT IMPLEMENTED! Time interval between pings (in usec, counted from previous receive or timeout which ever comes first).\n");
		printf("	-v					- Verbose output.\n");
		printf("	-f					- Flood ping.\n");
		printf("	-l [len]			- Length of the data payload of nPing packets. NOTE: 1 < len < 65.\n");
		printf("						  The two first bytes of the payload are always used  as a 16 bit sequence number.\n");
		printf("	   [ADDR] 			- the 802.15.4 MAC address of the target.\n\n");
}	

void sig_interrupt(void)
{
	printf("User interrupt.\n");
	close(sockfd);

	printf("--- %.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X ping statistics ---\n", nPing_opts.target[0], nPing_opts.target[1], nPing_opts.target[2], nPing_opts.target[3], nPing_opts.target[4], nPing_opts.target[5], nPing_opts.target[6], nPing_opts.target[7]);
	printf("%d packets transmitted, %d received, %d%% packet loss\n", sent, received, (int)((sent-received)*100/sent));
	
	exit(1);
}
/* 	A small helper function which simply checks the nPing header. The things that are checked are: data field length
 *	and the destination port number (msut be 254, see nanoIP IETF draft for more details).
 */
int check_nping_hdr(unsigned char *data, unsigned int len)
{
	if(data == NULL)
	{
		printf("NULL pointer as supplied for check_nping_hdr().\n");
		fflush(stdout);
		return(-1);
	}

	/*	Check the nanoIP header for version and flags. */
	if((data[0] & 0xf0) == 0x00)
	{
		int pkt_len=0;
		//	nUDP protocol field ok
		pkt_len = (int)(data[1]<<8);
		pkt_len += data[2];

		/*	Check the payllaod length field. */
		if(pkt_len != len - 5)
		{
			printf("nPing (data) header had wrong length.\n");
			fflush(stdout);
			return(-1);
		}

		/*	Check the port number. */
		if(data[3] != 0xfe)
		{
			printf("Source port number != 254 -> not a nPing packet.\n");
			fflush(stdout);
			return(-1);
		}
		
		return(1);
	}
	
	return(-1);
}

int main(int argc, char **argv)
{
	struct sockaddr_in srvaddr;
	unsigned char sndbuf[128];
	int packet_len = 0;
	unsigned char sndbuf_conf[128];
	unsigned char rbuf[512];
	int i = 1;
	
	int counter = 0;
	unsigned short int seq = 0;

	unsigned short int source_port = 253;
	unsigned short int dst_port = 254;
	
	unsigned char *nPing_packet;
/*	And the length of the packet. */
	unsigned short int plen;

	
	char argbuf[64];
	int hits = 0;
	int addr_bytes;
	
	
	int rval;
	struct pollfd pfds;
	unsigned int nfds = 1;
	
	struct timeval tv_snd;
	struct timeval tv_rcv;
	long unsigned int single_rtt;
	long unsigned int total_rtt = 0;
	int timeout;
	int got_reply = 0;
	
	unsigned char source_addr_bytes[8];
	char source_addr_string[32];
	unsigned char source_addr_type;

/*	Install a signal handler for user keyboard interrupt. */
	signal(SIGINT, (sighandler_t)sig_interrupt);

/*	Check the number of command line arguments. */
	if(argc < 2 || argc > 7 || strncmp(argv[1], "--help", 6) == 0)
	{
		usage();
		return(-1);
	}

/*	A very simple command line parsing. This functionality will most probably be placed in to a separate function at some point 
 *	(i.e. when there's more functionality and parameters).
 */
	while(i < argc)
	{
		hits = sscanf(argv[i], "%s ", argbuf);
		if(strncmp(argbuf, "-S", 2) == 0)
		{
			/*	The nRouted sserver address:portnumber is set here. */
			i++;
			hits = sscanf(argv[i], "%[^:]:%u", nPing_opts.nRd_address, &(nPing_opts.nRd_port));
			if(hits == 2)
				printf("Trying to connect nRouted at %s:%u\n", nPing_opts.nRd_address, nPing_opts.nRd_port);
			else
			{
				printf("Error parsing the nRouted server address/port number!\n");
				usage();
				return(-1);
			}

			bzero(argbuf, 64);
		}
		else if(strncmp(argbuf, "-c", 2) == 0)
		{
			/*	The number of ping packets to be sent. */
			i++;
			hits = sscanf(argv[i], "%u", &(nPing_opts.count));
			if(hits == 1)
				printf("Sending %u nPing packets.\n", nPing_opts.count);
			else
			{
				printf("Error parsing number of ping packet to send!\n");
				usage();
				return(-1);
			}
			bzero(argbuf, 64);
		}
		else if(strncmp(argbuf, "-i", 2) == 0)
		{
			/*	Packet interval in microseconds. */
			i++;
			hits = sscanf(argv[i], "%d", &(nPing_opts.interval));
			if(hits == 1)
				printf("Sending nPing with %u usec interval.\n", nPing_opts.interval);
			else
			{
				printf("Error parsing ping interval!\n");
				usage();
				return(-1);
			}
			bzero(argbuf, 64);
		}
		else if(strncmp(argbuf, "-v", 2) == 0)
		{
			/*	Extra verbosity. */
			nPing_opts.verbose = 1;
			bzero(argbuf, 64);
		}
		else if(strncmp(argbuf, "-f", 2) == 0)
		{
			/*	Flood ping. NOTE: this is not very useful in the context that nRouted is normally used. */
			nPing_opts.flood = 1;
			bzero(argbuf, 64);
		}
		else if(strncmp(argbuf, "-l", 2) == 0)
		{
			/*	Payload length (in bytes). */
			i++;
			hits = sscanf(argv[i], "%hu", &(nPing_opts.payload));
			if(hits == 1)
				printf("Using payload of %d bytes.\n", nPing_opts.payload);
			else
			{
				printf("Error parsing payload length!\n");
				usage();
				return(-1);
			}
			
			if(nPing_opts.payload > 64)
			{
				printf("\nTried to set payload length of %d when MAX is 64!\n", nPing_opts.payload);
				printf("Setting payload to 64.\n\n");
				sleep(2);
				
				nPing_opts.payload = 64;
			}
			else if(nPing_opts.payload < 2)
			{
				printf("Tried to set invalid payload length, minimum is 2 bytes.\n");
				printf("Setting payload to 2 bytes.\n");
				sleep(2);
				
				nPing_opts.payload = 2;
			}
			
			bzero(argbuf, 64);
		}
		else
		{
			/*	Target address. */
			addr_bytes = sscanf(argv[i], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &(nPing_opts.target[0]), &(nPing_opts.target[1]), &(nPing_opts.target[2]), &(nPing_opts.target[3]), &(nPing_opts.target[4]), &(nPing_opts.target[5]), &(nPing_opts.target[6]), &(nPing_opts.target[7]));
			if(nPing_opts.verbose == 1)
			{
				if(addr_bytes == 2)
				{
					printf("Target: %.2X:%.2X (Short address)\n", nPing_opts.target[0], nPing_opts.target[1]);
				}
				else if(addr_bytes == 8)
				{
					printf("Target: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X (Long address)\n", nPing_opts.target[0], nPing_opts.target[1], nPing_opts.target[2], nPing_opts.target[3], nPing_opts.target[4], nPing_opts.target[5], nPing_opts.target[6], nPing_opts.target[7]);
				}
				else if(addr_bytes == 10)
				{
					printf("nPing doesn't quite yet support the address in [64bit 802.15.4 addr][16bit PAN ID] format - sorry.\n");
					return(-1);
				}
			}
		}
		
		bzero(argbuf, 64);
		i++;
	}


/*	Now that we know if the user wants any extra payload in the nPing packets, we'll allocate the memory for the nPing packet. */

/*	Allocate the memory for the packet. The five bytes is for the nPing header and the rest is for the payload. */
	nPing_packet = (unsigned char *)malloc(nPing_opts.payload);

	printf("nPing_opts.payload:%d\n", nPing_opts.payload);


/*	Open a TCP socket. */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
/*	Prepare the struct sockaddr with the nRouted address:port information. */
	bzero(&srvaddr, sizeof(srvaddr));
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_port = htons(nPing_opts.nRd_port);
	inet_pton(AF_INET, nPing_opts.nRd_address, &srvaddr.sin_addr);

	if(nPing_opts.verbose == 1)
		printf("port: %d\n", nPing_opts.nRd_port);

/*	Connect to nRouted. */
	if((connect(sockfd, (struct sockaddr *)&srvaddr, sizeof(srvaddr))) != 0)
	{
		printf("Connect to nRouted failed...\n");
		return(-1);
	}

/*
 *	The full configuration packet is inserted to sndbuf_conf at its declaration. -mjs
 */
	if((rval = libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, 253)) == -1)
	{
		printf("Failed to create nRouted configuration packet.\n");
		return(-1);
	}

/*	Print the configuration packet if extra verbosity is set. */
	if(nPing_opts.verbose == 1)
	{
		for(i=0;i<rval;i++)
		{
			printf("0x%.2x ", sndbuf_conf[i]);
		}
		printf("\n");
	}

	if(nPing_opts.verbose == 1)
		printf("Sending config packet to nRouted...\n");

/*	Send the configuration packet to nRouted. */
	write(sockfd, sndbuf_conf, 13);

/*	Read the configuration reply. */
	i = read(sockfd, rbuf, 128);

/*	Check the configuration reply. */
	if((rval = libnrp_check_nRoute_reply(rbuf, i)) != 1)
	{
		printf("nRouted configuration error.\n");
		return(-1);
	}

/*	These are needed for the TCP socket polling. */
	pfds.fd = (int)(sockfd);
	pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;


/*	We start sending the nanoPing packets and at the moment wait for reply after each packet. */

/*	And now lets fill in the "random data" to the payload. */
	for(plen=0;plen<nPing_opts.payload;plen++)
	{
		nPing_packet[plen] = 0xED;
	}

	while(counter < nPing_opts.count || nPing_opts.count == -1)
	{
		/*	Two first bytes are the sequence number. */
		nPing_packet[0] = (seq >> 8);
		nPing_packet[1] = (seq & 0xff);

		/*	Create the nRP data packet using the nPing_packet[] as the data field. */	
		if(addr_bytes == 2)
		{
			packet_len = libnrp_create_data_pkt_hdr(sndbuf, nPing_packet, plen, PROTO_6LOWPAN, NULL, nPing_opts.target, ADDR_IEEE_802_15_4_SHORT, &source_port, dst_port, NULL);
		}
		else if(addr_bytes == 8)
		{
			packet_len = libnrp_create_data_pkt_hdr(sndbuf, nPing_packet, plen, PROTO_6LOWPAN, NULL, nPing_opts.target, ADDR_IEEE_802_15_4_DEV_LONG, &source_port, dst_port, NULL);
		}
		else
		{
			printf("Can't create ping packet for this address type - sorry.\n");
			return(-1);
		}
		
		if(packet_len == -1)
		{
			printf("Failed to create nPing packet.\n");
			return(-1);
		}	

		write(sockfd, sndbuf, packet_len);
		timeout = 1000;
		gettimeofday(&tv_snd, NULL);
		sent++;
		seq++;
		
		/*	Lets bzero the send buffer just in case... */
		bzero(sndbuf, 128);
		
		if (nPing_opts.count > 0)
		  counter++;

re_read:
		if((rval = poll(&pfds, nfds, timeout)) == 0)
		{
			if(got_reply == 0)
			{
				printf("Timed out...\n");
				fflush(stdout);
			}
			else
				received++;
			got_reply = 0;
			timeout = -1;

			timeouts++;
		}
		else
		{
			gettimeofday(&tv_rcv, NULL);
			
			single_rtt = (tv_rcv.tv_sec-tv_snd.tv_sec)*1000000;
			single_rtt += tv_rcv.tv_usec-tv_snd.tv_usec;
			
			timeout -= (single_rtt/1000);
			
			total_rtt += single_rtt;
			
			i = read(sockfd, rbuf, 512);
			if(i>0)
			{
					unsigned int dlen;
					unsigned int index;
					unsigned char *tmp_ptr;
					unsigned char data[256];
					unsigned int cnt_tmp;
					
					unsigned short int rcv_seq;

/*					printf("DATA: %u bytes\n", i);
					for(cnt_tmp=0;cnt_tmp<i;cnt_tmp++)
					{
						if(!(cnt_tmp%16))
							printf("Block [%d-%d]: ", cnt_tmp, cnt_tmp+16>i ? i : cnt_tmp+16);
						printf("%2.2x:", rbuf[cnt_tmp]);
						if(!((cnt_tmp+1)%16))
							printf("\n");
					}
					printf("\n");
*/
re_process:
					libnrp_get_data(rbuf, (unsigned int)i, data, &dlen);

					rcv_seq = (data[0] << 8);
					rcv_seq += data[1];

					if(rcv_seq == seq - 1)
					{
						if(libnrp_get_source_address(rbuf, i, &source_addr_type, source_addr_bytes, source_addr_string) == 1)
						{
							if(addr_bytes == 2)
							{
								if(memcmp(source_addr_bytes, nPing_opts.target, 2) == 0 || LIBNRP_CHECK_2_BYTE_BCAST(nPing_opts.target) == 1)
									got_reply = 1;
							}
							else if(addr_bytes == 8)
							{
								if(memcmp(source_addr_bytes, nPing_opts.target, 8) == 0 || LIBNRP_CHECK_8_BYTE_BCAST(nPing_opts.target) == 1)
									got_reply = 1;
							}
						
							if(source_addr_type == 0x10)
							{
								printf("%u bytes from %s: seq=%hu rtt=", dlen, source_addr_string, rcv_seq);
								if(single_rtt < 100)
									printf("%ld usec\n", single_rtt);
								else if(single_rtt >= 100 && single_rtt < 1000000)
									printf("%.2Lf msec\n", ((long double)single_rtt)/1000.0);
								else if(single_rtt >= 1000000) 
									printf("%.2Lf sec\n", ((long double)single_rtt)/1000000.0);

								fflush(stdout);
							}
							else if(source_addr_type == 0x80)
							{
								printf("%u bytes from %2.2x:%2.2x PAN ID:%2.2x:%2.2x seq=%hu rtt=", dlen, source_addr_bytes[0], source_addr_bytes[1], source_addr_bytes[2], source_addr_bytes[3], rcv_seq);
								if(single_rtt < 100)
									printf("%ld usec\n", single_rtt);
								else if(single_rtt >= 100 && single_rtt < 1000000)
									printf("%.2Lf msec\n", ((long double)single_rtt)/1000.0);
								else if(single_rtt >= 1000000) 
									printf("%.2Lf sec\n", ((long double)single_rtt)/1000000.0);

								fflush(stdout);
							}
							else
							{
								printf("%u bytes from %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x seq=%hu rtt=", dlen, source_addr_bytes[0], source_addr_bytes[1], source_addr_bytes[2], source_addr_bytes[3], source_addr_bytes[4], source_addr_bytes[5], source_addr_bytes[6], source_addr_bytes[7], rcv_seq);
								if(single_rtt < 100)
									printf("%ld usec\n", single_rtt);
								else if(single_rtt >= 100 && single_rtt < 1000000)
									printf("%.2Lf msec\n", ((long double)single_rtt)/1000.0);
								else if(single_rtt >= 1000000) 
									printf("%.2Lf sec\n", ((long double)single_rtt)/1000000.0);

								fflush(stdout);
							}

//							received++;
						}
						else if(libnrp_get_destination_address(rbuf, i, &source_addr_type, source_addr_bytes, source_addr_string) == 1)
						{
							printf("Destination host unreachable...\n");
							fflush(stdout);
						}
						else
						{
							printf("%u bytes from unknown address: %hu rtt=", dlen, rcv_seq);
							if(single_rtt < 100)
								printf("%ld usec\n", single_rtt);
							else if(single_rtt >= 100 && single_rtt < 1000000)
								printf("%.2Lf msec\n", ((long double)single_rtt)/1000.0);
							else if(single_rtt >= 1000000) 
								printf("%.2Lf sec\n", ((long double)single_rtt)/1000000.0);

							fflush(stdout);
						}
					}
					else
					{
						goto re_read;
					}

					index=4;
					tmp_ptr = &(rbuf[0]);
					while((tmp_ptr[index] & 0x80) != 0x80)
					{
						index += ((tmp_ptr[index+1] << 8) + tmp_ptr[index+2] + 3);
					}
					index += ((tmp_ptr[index+1] << 8) + tmp_ptr[index+2] + 3);
					
					if((strncmp((char *)(&tmp_ptr[index]), "NRP", 3)) == 0 && tmp_ptr[index+3] == (NRP_VERSION || DATA) && i > index)
					{
						memmove(rbuf, &(rbuf[index]), i-index);
						i -= index;
						goto re_process;
					}
						

			}
		}
		/*	Currently we have a static 0.8 second delay between nPing reply receive and the following send. */
		if(nPing_opts.flood != 0)
			timeout = -1;

		if(timeout > 0)
		{
			goto re_read;
		}

	}

	if (sent)
	{
		if(addr_bytes == 2)
		  printf("--- %.2X:%.2X ping statistics ---\n", nPing_opts.target[0], nPing_opts.target[1]);
		else if(addr_bytes == 8)
		  printf("--- %.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X:%.2X ping statistics ---\n", nPing_opts.target[0], nPing_opts.target[1], nPing_opts.target[2], nPing_opts.target[3], nPing_opts.target[4], nPing_opts.target[5], nPing_opts.target[6], nPing_opts.target[7]);
	  printf("%d packets transmitted, %d received, %d%% packet loss\n", sent, received, sent<received ? 0 : (int)((sent-received)*100/sent));
	}
	else
	{
	  printf("Unable to print statistics. No sent packages\n");
	}

	close(sockfd);
		
	return(1);
}
