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


/**	\file libSSI.c
 *	\brief The libSSI.c source code file
 *
 *	This file contains code for the SSI library.
 *
 *	Copyright: Sensinode Ltd.
 *
 */

#ifndef LIBSSI_H
#include "libSSI.h"
#endif

unsigned char *libSSI_create_query(unsigned char *addr)
{
	unsigned char *query_buffer;

	/*	allocate memory for the buffer */
	query_buffer = (unsigned char *)malloc(2*sizeof(unsigned char));
	
	if(addr != NULL)
	{
		/*	addr -parameter was given so use it */
		query_buffer[0] = addr[0];
	}
	else
	{
		/*	no addr -parameter so use the wildcard '?' */
		query_buffer[0] = 0x3f;
	}
	
	/*	now set the hexvalue 0x51 ('A'), this _could_ be also 0x71 */
	query_buffer[1] = 0x51;
	
	return(query_buffer);
}

int libSSI_parse_answer(unsigned char *buffer, unsigned char *address, unsigned char *version, unsigned char *buffer_sz, unsigned char *msg_delay, unsigned char *reserved)
{
	/*	escape if the packet is not an Answer packet. */
	if(buffer[1] != 0x41 && buffer[1] != 0x61)
	{
		return(-1);
	}

	/*	reserve the memory... */
	address = (unsigned char *)malloc(sizeof(unsigned char));
	version = (unsigned char *)malloc(2*sizeof(unsigned char));
	buffer_sz = (unsigned char *)malloc(2*sizeof(unsigned char));
	msg_delay = (unsigned char *)malloc(2*sizeof(unsigned char));
	reserved = (unsigned char *)malloc(2*sizeof(unsigned char));

	*address = buffer[0];
	memcpy(version, &(buffer[2]), 2);
	memcpy(buffer_sz, &(buffer[4]), 2);
	memcpy(msg_delay, &(buffer[6]), 2);
	memcpy(reserved, &(buffer[8]), 2);
	
	return(1);
}

unsigned char *libSSI_sensor_discovery(unsigned char *addr)
{
	unsigned char *discovery_buffer;

	/*	allocate memory for the buffer */
	discovery_buffer = (unsigned char *)malloc(2*sizeof(unsigned char));
	
	if(addr != NULL)
	{
		/*	addr -parameter was given so use it */
		discovery_buffer[0] = addr[0];
	}
	else
	{
		/*	no addr -parameter so use the wildcard '?' */
		discovery_buffer[0] = 0x3f;
	}
	
	/*	now set the hexvalue 0x43 ('C'), this _could_ be also 0x63 */
	discovery_buffer[1] = 0x43;
	
	return(discovery_buffer);
}

int libSSI_data_request(unsigned char *req_buffer, unsigned char *addr, uint16_t *sensor_ids, unsigned char *sensor_count)
{
	int i;
	
	/*	check if the number of sensors is greater than zero */
	if(*sensor_count == 0)
	{
		/*	equals zero, can't really do anything so exit with an error */
		return(-1);
	}
	
	req_buffer[0] = *addr;
	req_buffer[1] = 0x52;
	
	for(i=0;i<*sensor_count;i++)
	{
		memcpy(&(req_buffer[2+2*i]), &(sensor_ids[i]), 2);
	}
	
	return(2+2*i);
}

int libSSI_parse_data_response(unsigned char *buffer, unsigned char *address, uint16_t *sensor_ids, uint32_t *sensor_values, int buffer_len)
{
	int i;

	/*	check that the buffer is not a NULL pointer */
	if(buffer == NULL)
	{
		/*	nothing we can do */
		return(-1);
	}
	
	/*	check that the packet type really is 'V' (or 'v') */
	if(buffer[1] != 0x56 && buffer[1] != 0x76)
	{
		/*	wrong packet type, return with error */
		return(-1);
	}

	if(address != NULL)
		memcpy(address, &(buffer[0]), 1);

	for(i=0;i<(buffer_len-2)/6;i++)
	{
		memcpy(&(sensor_ids[i]), &(buffer[2+6*i]), 2);
//		memcpy(&(sensor_values[i]), &(buffer[4+6*i]), 4);
		sensor_values[i] = (buffer[4+6*i] << 24);
		sensor_values[i] += (buffer[5+6*i] << 16);
		sensor_values[i] += (buffer[6+6*i] << 8);
		sensor_values[i] += (buffer[7+6*i]);
	}

	return(i);
}

uint32_t *libSSI_get_response_data(unsigned char *buffer, unsigned char *address, uint16_t *sensor_id, int value_sz)
{
	int i;
	uint32_t *data_buffer;
	
	/*	check that the buffer is not a NULL pointer */
	if(buffer == NULL)
	{
		/*	nothing we can do, so return(NULL) is appropriate */
		return(NULL);
	}

	/*	check that the packet type really is 'M' (or 'm') */
	if(buffer[1] != 0x4d && buffer[1] != 0x6d)
	{
		/*	wrong packet type, return with error */
		return(NULL);
	}

	data_buffer = (uint32_t *)malloc(value_sz*sizeof(uint32_t));	

	for(i=0;i<value_sz;i++)
	{
		memcpy(&(data_buffer[i]), &(buffer[4+i*4]), 4);
	}
	
	return(data_buffer);
}

unsigned char *libSSI_create_observer(unsigned char *address, uint16_t interval, int8_t multiplier, uint8_t count, uint8_t length, uint32_t threshold, uint16_t *sensor_ids, uint8_t sensors)
{
	int i;
	unsigned char *observer_buffer;
	
	observer_buffer = (unsigned char *)malloc(11+sensors*sizeof(uint16_t));
	address = (unsigned char *)malloc(sizeof(unsigned char));
	
	if(address != NULL)
		observer_buffer[0] = *address;
	else
		observer_buffer[0] = 0x3f;

	observer_buffer[1] = 0x4f;
	memcpy(&(observer_buffer[2]), &interval, 2);
	memcpy(&(observer_buffer[4]), &multiplier, 1);
	memcpy(&(observer_buffer[5]), &count, 1);
	memcpy(&(observer_buffer[6]), &length, 1);
	memcpy(&(observer_buffer[7]), &threshold, 4);

	for(i=0;i<sensors;i++)
	{
		memcpy(&(observer_buffer[11+2*i]), &(sensor_ids[i]), 2);
	}
	
	return(observer_buffer);
}

int libSSI_parse_obs_created(unsigned char *buffer, unsigned char *address, unsigned char *observer_id)
{
	/*	check if buffer != NULL */
	if(buffer == NULL)
	{
		/*	buffer is a NULL pointer so there's nothing we can do */
		return(-1);
	}
	
	address = (unsigned char *)malloc(sizeof(unsigned char));
	observer_id = (unsigned char *)malloc(sizeof(unsigned char));

	memcpy(address, &(buffer[0]), 1);
	memcpy(observer_id, &(buffer[2]), 1);

	return(1);
}

int libSSI_parse_obs_finished(unsigned char *buffer, unsigned char *address, unsigned char *observer_id)
{
	/*	check if buffer != NULL */
	if(buffer == NULL)
	{
		/*	buffer is a NULL pointer so there's nothing we can do */
		return(-1);
	}
	
	address = (unsigned char *)malloc(sizeof(unsigned char));
	observer_id = (unsigned char *)malloc(sizeof(unsigned char));

	memcpy(address, &(buffer[0]), 1);
	memcpy(observer_id, &(buffer[2]), 1);
	
	return(1);
}

unsigned char *libSSI_create_short_error(unsigned char *address, unsigned char errorcode)
{
	unsigned char *error_buffer = (unsigned char *)malloc(3*sizeof(unsigned char));
	
	if(address == NULL)
		error_buffer[0] = 0x3f;
	else
		error_buffer[0] = *address;
	
	error_buffer[1] = 0x45;
	error_buffer[2] = errorcode;
	
	return(error_buffer);
}

uint16_t *libSSI_parse_error_packet(unsigned char *buffer, unsigned char *address, unsigned char *errorcode, unsigned char packet_size)
{
	uint16_t *sensor_ids;

	if(buffer[1] != 0x45)
		return(NULL);

	address = (unsigned char *)malloc(sizeof(unsigned char));
	errorcode = (unsigned char *)malloc(sizeof(unsigned char));
	
	memcpy(address, &(buffer[0]), 1);
	memcpy(errorcode, &(buffer[2]), 1);

	if(packet_size > 3)
	{
		int i;
		
		sensor_ids = (uint16_t *)malloc((packet_size-3)*sizeof(uint16_t)/2);
		
		for(i=0;i<(packet_size-3)/2;i++)
		{
			memcpy(&(sensor_ids[i]), &(buffer[3+2*i]), 2);
		}
	}

	return(sensor_ids);
}

int libSSI_parse_disc_reply(unsigned char *buffer, unsigned char *address, uint16_t packet_size, unsigned char *sensors, struct sensor_description_t *sensor_desc)
{
	int i=0;

	if(buffer[1] != 0x4e)
		return(-1	);
	
	for(;i < (packet_size-2)/36;i++)
	{
		printf("Sensors:%u\n", *sensors);
	
		memcpy(&(sensor_desc[*sensors].sensor_id), &(buffer[2+i*36]), 2);
		memcpy(&(sensor_desc[*sensors].description), &(buffer[4+i*36]), 16);
		sensor_desc[*sensors].description[17] = '\0';
		memcpy(&(sensor_desc[*sensors].unit), &(buffer[20+i*36]), 8);
		sensor_desc[*sensors].unit[9] = '\0';
		memcpy(&(sensor_desc[*sensors].type), &(buffer[28+i*36]), 1);
		memcpy(&(sensor_desc[*sensors].scaler), &(buffer[29+i*36]), 1);
		memcpy(&(sensor_desc[*sensors].min_value), &(buffer[30+i*36]), 4);
		memcpy(&(sensor_desc[*sensors].max_value), &(buffer[34+i*36]), 4);

		(*sensors)++;
	}
	
	return(1);
}
