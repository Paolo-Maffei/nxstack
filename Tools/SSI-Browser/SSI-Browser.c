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
#include <arpa/inet.h>
#include <netinet/in.h>

#include <math.h>

#include <unistd.h>

#include <strings.h>
#include <string.h>

#include <sys/poll.h>

#ifndef LIBNRP_H
#include "../libnRP/libnRP.h"
#endif

#ifndef LIBSSI_H
#include "../libSSI/libSSI.h"
#endif

void usage(void)
{
		printf("SSI-Browser version 0.9\n\n");
		printf("USAGE: ./SSI-Browser [ADDR]\n");
		printf("	Options:\n");
		printf("	-S [nRouted_address:port]	- The address and port number where the nRouted TCP server is located.\n");
		printf("								  (for example  -S 192.168.0.145:13022 )\n");
		printf("	[ADDR] 			- the 802.15.4 MAC address of the target.\n\n");
}

int main(int argc, char **argv)
{
	int sockfd;

	unsigned char sndbuf[128];
	unsigned char *SSI_sensor_discovery;
	int packet_len = 0;
	unsigned char SSI_request[128];
	unsigned char sndbuf_conf[128];
	unsigned char buffer[512];
	unsigned char rbuf[512];
	
	int recvbytes;
	unsigned int bytes;

	unsigned short int portn;
	
	int i = 1;

	char nRd_address[32] = "127.0.0.1";
	unsigned int nRd_port = 21870;
	unsigned char target[8];
	char argbuf[64];
	int hits = 0;

	unsigned char sensor_count = 0;
	struct sensor_description_t sensor_desc[10];
	unsigned char sensor_addr=0;

	struct pollfd pfds;
	unsigned int nfds = 1;
	int timed_out = 0;
	int full_description=0;
	int rval = 0;
	int loop = 1;
	

/*	First check the command line parameters and parse them. */
	if(argc < 2 || argc > 4 || strncmp(argv[1], "--help", 6) == 0)
	{
		usage();
		return(-1);
	}

	while(i < argc)
	{
		sscanf(argv[i], "%s ", argbuf);
		if(strncmp(argbuf, "-S", 2) == 0)
		{
			i++;
			hits = sscanf(argv[i], "%[^:]:%d", nRd_address, &nRd_port);
			if(hits == 2)
				printf("Trying to connect nRouted at %s port %d\n", nRd_address, nRd_port);

			bzero(argbuf, 64);
		}
		else if (strncmp(argbuf, "-l", 2) == 0)
		{
			printf("Loop mode.\n");
			loop = 10;
		}
		else
		{
			hits = sscanf(argv[i], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &(target[0]), &(target[1]), &(target[2]), &(target[3]), &(target[4]), &(target[5]), &(target[6]), &(target[7]));

			printf("Requesting SSI data from: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n", target[0], target[1], target[2], target[3], target[4], target[5], target[6], target[7]);
		}
		
		bzero(argbuf, 64);
		i++;
	}

	printf("Trying to connect nRouted at %s port %d\n", nRd_address, nRd_port);

/*	Copying the Target address to the end of the nRouted Configuration packet, discovery packet and SSI request packet. */
	for(i=0;i<8;i++)
	{
		sndbuf_conf[i+12] = target[i];
		sndbuf[i+18] = target[i];
		SSI_request[i+12] = target[i];
	}

//	if((rval=libnrp_create_conf_pkt(sndbuf_conf, PROTO_UNDEFINED, target, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) < 0)
	if((rval=libnrp_create_conf_pkt(sndbuf_conf, PROTO_6LOWPAN, NULL, NULL, ADDR_UNDEFINED, NULL, PORT_UNDEFINED)) < 0)
	{
		printf("Failed to create an nRouted configuration packet.\n");
		return(-1);
	}
	
	if((sockfd = libnrp_snd_conf(sndbuf_conf, rval, nRd_address, nRd_port)) < 0)
	{
		printf("Failed to send the configuration packet to nRouted.\n");
		return(-1);
	}
	else
	{
		printf("nRouted configuration successful.\n");
	}




/*	Create the SSI sensor discovery packet using the wildcard address */
	SSI_sensor_discovery = libSSI_sensor_discovery(NULL);
	printf("SSI_sensor_discovery:0x%hhx 0x%hhx\n", SSI_sensor_discovery[0], SSI_sensor_discovery[1]);


/*	Create the SSI sensor query packet inside a nRP data packet. */
	portn = 122;
	packet_len = libnrp_create_data_pkt_hdr(sndbuf, SSI_sensor_discovery, 2, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 40, NULL);



/*	Here we send the SSI discovery packet to the sensor device. */
	write(sockfd, sndbuf, packet_len);

	pfds.fd = (int)(sockfd);
	pfds.events = POLLIN | POLLPRI | POLLERR | POLLHUP | POLLNVAL;

/*	We try to read the answer and then process it. */
	i=0;
	timed_out = 0;
	while(!timed_out && full_description != 1)
	{
		if((rval = poll(&pfds, nfds, 1000)) == 0)
		{
			printf("Timed out SSI discovery reply.\n");
			timed_out = -1;
		}
		else
		{
			recvbytes = read(sockfd, rbuf, 512);
			if(recvbytes>0)
			{
				printf("Received %d bytes ", recvbytes);

				libnrp_get_data(rbuf, recvbytes, buffer, &bytes);

				if(GET_PACKET_TYPE(buffer) == SSI_DISC_REPLY)
				{
					printf("Got an SSI sensor discovery reply - good.\n");
					
					libSSI_parse_disc_reply(buffer, &sensor_addr, bytes, &sensor_count, sensor_desc);

					printf("Got %u sensor descriptions.\n", sensor_count);
					
					if(buffer[bytes-2] == 0xff && buffer[bytes-1] == 0xff)
					{
						full_description = 1;
						printf("Got full set of sensor descriptions.\n");
					}
				}
				else
				{
					printf("Wrong type: 0x%hhx\n", (unsigned char)GET_PACKET_TYPE(buffer));
				}
			}
		}
	}
	
	if(full_description == 0)
	{
		printf("Some sensor descriptions were left missing as I timed out.\n");
	}
	

	usleep(100000);

	if(sensor_count > 0)
	{
		int k;
		uint16_t sensor_id_list[12];

		for(k=0;k<sensor_count;k++)
		{
			memcpy(&(sensor_id_list[k]), &(sensor_desc[k].sensor_id), 2);
		}

		bytes = libSSI_data_request(SSI_request, &sensor_addr, sensor_id_list, &sensor_count);

		packet_len = libnrp_create_data_pkt_hdr(sndbuf, SSI_request, bytes, PROTO_6LOWPAN, NULL, target, ADDR_IEEE_802_15_4_DEV_LONG, &portn, 40, NULL);

		printf("Write %d bytes\n", packet_len);
		write(sockfd, sndbuf, packet_len);

		timed_out = 0;
		while(!timed_out )
		{
			if((rval = poll(&pfds, nfds, 2000)) == 0)
			{
				printf("Timed out SSI discovery reply.\n");
				timed_out = -1;
			}
			else
			{
				printf("Read\n");
				recvbytes = read(sockfd, buffer, 512);
		
				libnrp_get_data(buffer, recvbytes, rbuf, &bytes);
		
				if(GET_PACKET_TYPE(rbuf) == SSI_DATA_REPLY)
				{
					if(bytes>0)
					{
						uint16_t sensor_ids[32];
						uint32_t sensor_values[32];
		
						printf("Received SSI_DATA_REPLY (%d bytes).\n", bytes);
						i=0;
		
						i = libSSI_parse_data_response(rbuf, NULL, sensor_ids, sensor_values, bytes);
		
		
						for(k=0;k<i;k++)
						{
							int j;
							for(j=0;j<i;j++)
							{
								if(sensor_ids[k] == sensor_desc[j].sensor_id)
								{
									if(sensor_desc[j].type == 0x00)
										printf("%s:%lf %s\n", sensor_desc[j].description, (double)sensor_values[k],  sensor_desc[j].unit);
									else if(sensor_desc[j].type == 0x01)
										printf("%s:%d %s\n", sensor_desc[j].description, (int)sensor_values[k],  sensor_desc[j].unit);
								}
							}
						}
					}
				}
				timed_out = -1;
			}
		}
	}
	else
	{
		printf("No sensors found.\n");
		close(sockfd);
		return(-1);
	}
	
	close(sockfd);

	return(1);
}
