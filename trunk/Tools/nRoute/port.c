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


#include "port.h"

#include "nRouted.h"

extern struct port_t *nRdport;
extern struct nRdconfig_t nRdconf;

extern void logger(int loglevel, const char *tolog, ...);
extern void rawlogger(int loglevel, const char *tolog, ...);


int port_open(char *device)
{
	nRdport = (struct port_t *) malloc(sizeof(struct port_t));
	
	nRdport->device = 0;
	nRdport->handle = 0;

	nRdport->handle = open(device, O_RDWR | O_NOCTTY | O_NDELAY /*| O_NONBLOCK */);

	if (nRdport->handle <= 0)
	{
		logger(0, "Serial port open failed with error message: %s.\n", strerror(errno));
		return(-1);
	}
	else
	{
		tcgetattr(nRdport->handle, &(nRdport->old_params));

		fcntl(nRdport->handle, F_SETFL, FASYNC);
		logger(1, "Serial port %s opened succesfully.\n", device);

		return(0);
	}	
}

int port_close(void)
{
	struct port_t *port_tmp = nRdport;
	
	if (!nRdport)
		return(-1);
	
	nRdport = 0;

	tcflush(port_tmp->handle, TCIFLUSH);
	tcsetattr(port_tmp->handle,TCSANOW,&(port_tmp->old_params));
	
	close(port_tmp->handle);

	if (port_tmp->device)
		free(port_tmp->device);
	port_tmp->device = 0;
	free(port_tmp);
	
	return(1);

}


/** @todo port_write() function probably needs mutexes -mjs */

int port_write(unsigned char *buffer, size_t buflen)
{
	int i=0;

	logger(2, "Writing %s to serial port.\n", buffer);

	if(nRdconf.loglevel >= 2)
	{
		logger(2, "Writing to UART:");
		for(i=0;i<buflen;i++)
		{
			rawlogger(2, "0x%.2x ", buffer[i]);
		}
		rawlogger(2, "\n");
	}

	i=0;
	
#define WR_SIZE buflen
	while(i<buflen)
	{
		if(buflen > i + WR_SIZE)
		{
			logger(16, "WRITING %d BYTES!\n", WR_SIZE);
			write(nRdport->handle, &(buffer[i]), WR_SIZE);
			tcdrain(nRdport->handle);
			i+=WR_SIZE;
			usleep(400);
		}
		else
		{
			write(nRdport->handle, &(buffer[i]), buflen-i);
			tcdrain(nRdport->handle);
			logger(2, "WRITE %d BYTES!\n", buflen-i);
			i = buflen;
		}
	}

	logger(2, "Back from write_port()\n");

	usleep(1000);
	
	return(0);
}

int port_read(unsigned char *buffer, size_t buflen)
{
	unsigned int l = 0;
	l = read(nRdport->handle, buffer, buflen);
	return(l);
}

int set_port_params(void)
{
	int rate = B115200;
	int rtscts = 1;
	int parity = 0;
	struct termios newtio;
	
	bzero(&newtio, sizeof(newtio));
	
	newtio.c_cflag |= CS8 | CREAD;
	newtio.c_oflag &= ~OPOST;
	if (parity)
	{
		newtio.c_cflag |= PARENB;
		if (parity & 1)	newtio.c_cflag |= PARODD;
	}
	else
	{
		newtio.c_iflag = IGNPAR;
	}
	if (rtscts)
	{
		newtio.c_cflag |= CRTSCTS;
	}
	
	cfsetispeed(&newtio, rate);
	cfsetospeed(&newtio, rate);
	
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;

	tcflush(nRdport->handle, TCIFLUSH);
	tcsetattr(nRdport->handle,TCSANOW,&newtio);

	return(0);
}

