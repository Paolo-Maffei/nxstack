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


#ifndef _PORT_H
#define _PORT_H

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define SERIALSPEED B115200


struct port_t
{
	int handle;
	char *device;
	struct termios old_params;
};

struct port_t *nRdport;


int port_open(char *device);
int port_close(void);
int port_write(unsigned char *buffer, size_t buflen);
int port_read(unsigned char *buffer, size_t buflen);
int set_port_params(void);


#include "nRouted.h"

extern struct port_t *nRdport;

extern void logger(int loglevel, const char *tolog, ...);

#include "port.h"
#endif
