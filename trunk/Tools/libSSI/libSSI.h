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


/**	\file libSSI.h
 *	\brief The libSSI.h header file
 *
 *	This header file contains definitions, macros and function declarations for the SSI library.
 *
 *	Copyright: Sensinode Ltd.
 *
 */

#define LIBSSI_H
#define SSI_MAJOR_VERSION 1
#define SSI_MINOR_VERSION 2

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef struct sensor_description_t {
	uint16_t sensor_id;
	char description[17];
	char unit[9];
	unsigned char type;
	uint8_t scaler;
	uint32_t min_value;
	uint32_t max_value;
} sensor_description_t;


/*	Return the number of sensor values in a response packet. If the packet is not of correct length, return -1.
 *	The only parameter is the _total_ length of the packet. */
#define CALCULATE_RESP_SZ_V(x) ((x-2)%6 == 0) : (x-2)/6 ? -1
#define CALCULATE_RESP_SZ_M(x) ((x-4)%4 == 0) : (x-4)/4 ? -1

/** GET_PACKET_TYPE Get the type of the SSI packet.
 *
 *	\param buffer The SSI packet.
 *	\return The packet type as enumerated value.
 *
 *	NOTE: To be precise the buffer argument does not have to be the whole packet the first two bytes are enough (although it is
 *	usually far more convenient to pass a pointer to the whole packet.
 *	NOTE: This macro is just a simple wrapper which enables one to mask the two different ways of marking the packet types
 *	(small caps or capital letters).
 */
#define GET_PACKET_TYPE(x) (x[1] & 0xDF)

#define DETERMINE_CRC(x) ((x[1] & 0x20) == 0x20) : YES ? NO

enum SSI_packet_types { SSI_QUERY = 0x51, SSI_QUERY_REPLY = 0x41, SSI_DISC_SENSORS = 0x43, SSI_DISC_REPLY = 0x4e, SSI_RESET_W_DEVICE = 0x5a, SSI_GET_CONFIG = 0x47, SSI_CONFIG_REPLY = 0x58, SSI_SET_CONFIG = 0x53, SSI_REQUEST_DATA = 0x52, SSI_DATA_REPLY = 0x56, SSI_DATA_REPLY_STATUS = 0x44, SSI_DATA_REPLY_MULTI = 0x4d, SSI_CREATE_OBSERVER = 0x4f, SSI_OBSERVER_CREATED = 0x59, SSI_DELETE_OBSERVER = 0x4b, SSI_OBSERVER_FINISHED = 0x55, SSI_REQUEST_LISTENER = 0x4c, SSI_LISTENER_CREATED = 0x4a, SSI_ERROR = 0x45, SSI_FREE_DATA = 0x46 };

enum truthvalue { NO = 0, YES = 1 };

/**	libSSI_create_query Create an SSI query.
 *
 *	\param addr The address (single byte) of the sensor (OPTIONAL)
 *	\return pointer to a unsigned char array that contains the packet.
 *
 *	NOTE: The user _must_ take care of the freeing of the memory address
 *	that this function returns.
 */
unsigned char *libSSI_create_query(unsigned char *addr);

/**	libSSI_parse_answer Parse a buffer containing an SSI Answer. 
 *
 *	
 *	\param buffer The buffer that contains the answer
 *	\param address A pointer to an unsigned char (a single byte)where the function will store the sensor address
 *	\param version A pointer to an unsigned char (two bytes) where the function will store the protocol version number from the received packet
 *	\param buffer_sz A pointer to an unsigned char (two bytes) where the function will store the size that must be reserved for data packets (input buffer)
 *	\param msg_delay A pointer to an unsigned char (two bytes) where the function will store the delay that has to be kept between messges in milliseconds
 *	\param reserved A pointer to an unsigned char (two bytes) where the function will store possible data that may be defined in the future
 *	\return 1 on success, -1 on error (errno is not set)
 *
 *	NOTE: The user _must_ take care of the freeing of the memory addresses
 *	that this function allocates for variables *address, *version, *buffer_sz, *msg_delay and *reserved.
 */
int libSSI_parse_answer(unsigned char *buffer, unsigned char *address, unsigned char *version, unsigned char *buffer_sz, unsigned char *msg_delay, unsigned char *reserved);

/**	libSSI_create_discovery Create an SSI sensor discovery packet.
 *
 *	\param addr The address (single byte) of the sensor (OPTIONAL)
 *	\return pointer to an unsigned char array that contains the discovery packet
 *
 *	NOTE: The user _must_ take care of the freeing of the memory address
 *	that this function returns.
 */
unsigned char *libSSI_sensor_discovery(unsigned char *addr);

/**	libSSI_data_request Create an SSI data request packet.
 *
 *	\param req_buffer A pointer to an array of unsigned chars where the request packet will be stored
 *	\param addr The address (single byte) of the sensor (OPTIONAL)
 *	\param sensor_ids The array of sensor ids to request the data from.
 *	\param sensor_count The number of sensors in the sensor_ids array.
 *	\return 1 if ok, -1 if failure
 *
 *	NOTE: The user _must_ take care of the freeing of the memory address
 *	that this function returns.
 */
int libSSI_data_request(unsigned char *req_buffer, unsigned char *addr, uint16_t *sensor_ids, unsigned char *sensor_count);

/**	libSSI_parse_data_response Parse an SSI data response type 'V' packet.
 *
 *	\param buffer An unsigned char buffer that contains the received packet
 *	\param address An unsigned char pointer where the address of the sensor device will be stored
 *	\param sensor_ids A pointer to 16b unsigned intergers where sensor ID's will be stored
 *	\param sensor_values A pointer to 32b unsigned integers where the actual sensor values will be stored 
 *	\param buffer_len Length of the buffer.
 *	\return Number of sensor values parsed from the buffer.
 *
 *	NOTE: This function can be used to parse ONLY Sensor Data Response packet of type 'V' !
 *	NOTE: The ith element of sensor_values contains the data associated to the sensor at the ith element of the sensor_ids
 */
int libSSI_parse_data_response(unsigned char *buffer, unsigned char *address, uint16_t *sensor_ids, uint32_t *sensor_values, int buffer_len);

/**	libSSI_get_response_data Parse an SSI data response type 'M' packet.
 *
 *	\param buffer An unsigned char buffer that contains the received packet
 *	\param address An unsigned char pointer where the address of the sensor device will be stored
 *	\param sensor_id A pointer to 16b unsigned interger where sensor ID will be stored
 *	\param value_sz The number of sensor values in the packet.
 *	\return a pointer to an array of uint32_t elements where the sensor values will be stored.
 *
 *	NOTE: This function can be used to parse ONLY Sensor Data Response packet of type 'M' !
 *	NOTE: The user _must_ take care of the freeing of the memory address
 *	that this function returns and the memory range that is allocated for the variablee *address
 *	and *sensor_id.
 */
uint32_t *libSSI_get_response_data(unsigned char *buffer, unsigned char *address, uint16_t *sensor_id, int value_sz);

/**	libSSI_create_observer Create an SSI Create Sensor Observer type 'O' packet (or type 'o')
 *
 *	\param address A pointer to an unsigned char which contains the device where the observer is created. If the pointer is NULL, wildcard is used.
 *	\param interval An uint16_t which contains the refresh time interval for the observer in milliseconds
 *	\param multiplier An int8_t which contains a signed 10's exponent multiplier for the refresh interval
 *	\param count An uint8_t which contains the number of sensor data responses ('V') to send. A value of 0xff means continuous sending. The continuous mode can be stopped with a 'K' command.
 *	\param length An uint8_t representing the number of samples for a specific sensor to be included in a single response. If length > 1 then the response will be of type 'M' instead of default 'V'.
 *	\param threshold An 4 byte floating point or integer value casted into an uint32_t which represents the absolute value of minimum change between two consecutive sensor value reads as a condition to send the data.
 *	\param sensor_ids A pointer to array of uint16_t elements that contain the sensor id's
 *	\param sensors An uint8_t that contains the number of sensors in the sensor_ids array
 *	\return a pointer to array of unsigned chars that contains the create observer packet
 *
 *	NOTE: The user _must_ take care of the freeing of the memory address
 *	that this function returns.
 */
unsigned char *libSSI_create_observer(unsigned char *address, uint16_t interval, int8_t multiplier, uint8_t count, uint8_t length, uint32_t threshold, uint16_t *sensor_ids, uint8_t sensors);

/**	libSSI_parse_obs_created Parse an SSI Observer Created packet (type 'Y').
 *
 *	\param buffer a pointer to an unsigned char array which contains the packet
 *	\param address a pointer to an unsigned char where the sensor address will be stored
 *	\param observer_id a pointer to an unsigned char where the Observer ID will be stored
 *	\return 1 on success, -1 on failure
 */
int libSSI_parse_obs_created(unsigned char *buffer, unsigned char *address, unsigned char *observer_id);

/**	libSSI_parse_obs_finished Parse an SSI Observer Finished packet (type 'U').
 *
 *	\param buffer a pointer to an unsigned char array which contains the packet
 *	\param address a pointer to an unsigned char where the sensor address will be stored
 *	\param observer_id a pointer to an unsigned char where the sensor ID will be stored
 *	\return 1 on success, -1 on failure
 */
int libSSI_parse_obs_finished(unsigned char *buffer, unsigned char *address, unsigned char *observer_id);

/**	libSSI_create_delete_observer Create an SSI Delete Observer packet (type 'K').
 *
 *	\param address a pointer to an unsigned char containing the sensor address (address == NULL ->'?' for wildcard as usual)
 *	\param observer_id a pointer to an unsigned char where the sensor ID is stored
 *	\return The function returns a pointer to a unsigned char array which will contain the three byte packet
 *
 *	NOTE: The user must free the memory allocated to the pointer that this function returns.
 */
unsigned char *libSSI_create_delete_observer(unsigned char *address, unsigned char *observer_id);

/**	libSSI_create_short_error Create a short SSI error packet (type 'E')
 *
 *	\param address a pointer to an unsigned char which contains the address of the device (OPTIONAL, if set to NULL wildcard '?' will be used.
 *	\param errorcode an unsigned char containing the specific error code
 *	\return a buffer containing the three byte error message
 */
unsigned char *libSSI_create_short_error(unsigned char *address, unsigned char errorcode);

/**	libSSI_parse_error_packet Parse an SSI error packet (type 'E')
 *
 *	\param buffer a pointer to unsigned char array containing the packet
 *	\param address a pointer to and unsigned char where the address of the device will be stored (a single byte)
 *	\param errorcode a pointer to an unsigned char where the errorcode will be stored
 *	\param packet_size an unsigned char that contains the length of the packet to be parsed
 *	\return If the error packet is of the longer type (so that it contains the sensor ID's of the sensors that produced the error) 
 *	this function returns a pointer to the array of uint16_t intergers containing the specific Sensor ID numbers. Otherwise NULL is returned
 */
uint16_t *libSSI_parse_error_packet(unsigned char *buffer, unsigned char *address, unsigned char *errorcode, unsigned char packet_size);

/**	libSSI_parse_disc_reply Parse an SSI discovery reply packet (type 'N')
 *
 *	\param buffer a pointer to an unsigned char array containing the packet
 *	\param address a pointer to an unsigned char where the address of the sensor device will be stored
 *	\param packet_size the total length of the packet
 *	\param sensors a pointer to an unsigned char where the total number of sensors found from the packet will be stored
 *	\return The function returns a struct sensor_description * pointer which contains the sensor descriptions.
 *
 *	NOTE: The user must take care of the freeing of the memory allocated for the returned struct. This memory is allocated within this
 *		  function so the calling function does not have to do that.
 */
int libSSI_parse_disc_reply(unsigned char *buffer, unsigned char *address, uint16_t packet_size, unsigned char *sensors, struct sensor_description_t *sensor_desc);
