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


#define _SERIALSERVER_H

#ifndef _NROUTED_H
#include "nRouted.h"
#endif /*_NROUTED_H*/

#ifndef _PORT_H

#include "port.h"

#endif /* _PORT_H */



/** @file serialserver.h
 *
 *	This header file contains those declarations that are essential part of the serial server functionality.
 *
 */

/** @struct packet_entry_t. 
 *	Basic packet item in the FIFO buffer. 
 */
struct packet_entry_t {
	struct packet_entry_t *next;	/**< Pointer to next item in the FIFO buffer. */
	struct packet_entry_t *prev;	/**< Pointer to previous item in the FIFO buffer. */

	char packet_data[MAX_NRP_PACKET_SIZE];			/**< This array contains the actual packet that should be sent. */
};

/** @struct fifo_buffer_t.
 *	
 *	This struct defines the actual FIFO buffer. 
 */
struct fifo_buffer_t {
	struct packet_entry_t *top;			/**< This pointer points at all times to the top of the stack (=FIFO) i.e. it's the last item that has been added to the stack. */
	struct packet_entry_t *bottom;		/**< This pointer points at all times to the bottom of the stack (=FIFO) i.e. it's the first item to be purged from the stack. */
	
	struct packet_entry_t *fifo_ring;	/**< The actual buffer. The size of the buffer is set at initialization. Variable nRdconf.serialbufsize is used as the size. */
};

/**	This function processes the data that has been received by the serial_server().
 *
 *	@param packet A unsigned char array containing the data received by the serial_server()
 *	@param len Length of the data array
 *	@return 0 on success, -1 on error
 */
int nrp_from_serial(unsigned char packet[MAX_NRP_PACKET_SIZE], int len);


/** serial_send_buffer	The actual serial port send buffer.
 *
 *	This buffer is introduced here but e.g. memory for some of its internals is allocated at init_serial_buffers().
 */
struct fifo_buffer_t serial_send_buffer;

/**	The buffer initialization function.
 *
 *	The serial servers send buffers are initialized in this function. Memory is allocated and the buffers internal pointers are set.
 *	If a separate priority send buffer will be implemented in the future, it must also be initialized here.
 *
 *	@return 0 on success, -1 on failure to allocate memory for buffers.
 */
int init_serial_buffers(void);

/*
 *	These functions implement the actual buffer manipulation functionality. They are intended to be used through wrapper functions such as
 *	send_to_serial(). There _should_ be no reason why they could not be called directly but as some of the statistics are updated in the 
 *	wrapper functions, the problems are obvious.
 */
 
/** Adds a packet to specified buffer.
 *
 *	This function adds the packet given as parameter to the specified buffer and adjusts the buffers top pointer.
 *
 *	@param *bufaddr Pointer to the buffer where the packet is to be added.
 *	@param packet A nRP packet.
 *	@return 0 (zero) if the packet was succesfully added to the buffer, -1 if buffer was full and the packet was dropped.
 */
int add_packet(struct fifo_buffer_t *bufaddr, char packet[MAX_NRP_PACKET_SIZE]);

/** Retrieves the bottom packet.
 *
 *	This function is used to retrieve the bottom packet which is to be sent next.
 *
 *	@param bufaddr Pointer to the buffer where the packet is to be retrieved.
 *	@param packet Pointer to a char array where the retrieved packet is copied. At least MAX_NRP_PACKET_SIZE octets must be reserved to the array
 *	before the function is called.
 *	@return 0 (zero) if packet was succesfully retrieved, 1 if the buffer was empty, -1 if packet is a NULL pointer, -2 if an unspecified error occured.
 *
 */
int get_bottom_packet(struct fifo_buffer_t *bufaddr, char *packet);

/** Remove the bottom packet. 
 *
 *	This function removes the packet pointed to by the bottom pointer from the buffer specified by the buffaddr. After this the bottom pointer 
 *	will be readjusted to point to the new bottom packet.
 *
 *	@param bufaddr Pointer to the buffer where the packet is to be purged from.
 *	@return 0 (zero) if the packet was succesfully purged, 1 if the buffer was already empty or -1 if an unspecified error occured.
 *
 */
int purge_bottom_packet(struct fifo_buffer_t *bufaddr);

/** Purge a specific packet.
 *
 *	This function can be used to remove all packets from a buffer that match the nRP header given as parameter. The header consists of
 *	Tag-Value pairs as ordinary nRP data packets. There are only two differences: 1) the valid Tags are 0x01, 0x02, 0x03 and 0x05 all or any subset 
 *	of these Tags can be used. The sizes of corresponding value fields are defined in nRP-Manual. 2) If all the Tags are used the header
 *	will be exactly 25 octets long, if only a subset of Tags are used, the remainig space (of the 25 octets) must be filled with 0xff.
 *	Wildcards are not supported at the moment.
 *
 *	@todo Implement support for wildcards in to the packet_purge() function -mjs.
 *	@param bufaddr Pointer to the buffer where the packet(s) is/are to be purged from.
 *	@param packet_header A 25 octets long char array consisting of valid Tag-Value pairs, with remainig space filled with 0xff's.
 *	@return Number of packets purged, 0 (zero) if no packets were purged (no match), -1 if buffer was already empty or -2 if an unspecified
 *	error occured.
 *
 */
int purge_packet(struct fifo_buffer_t *bufaddr, char packet_header[25]);


/** Remove all packets from buffer.
 *
 *	This function removes all the packets from a buffer pointed to by buffaddr and adjusts the bottom and top pointers accordingly.
 *
 *	@param bufaddr A pointer to the buffer which is to be cleared.
 *	@return Number of packets removed, 0 (zero) if no packets were removed, -1 if an unspecified error occured.
 *
 */
int clear_buffer(struct fifo_buffer_t *bufaddr);


/*
 *	These are the wrapper functions 
 */
/**	This function is used to send nRP packets to serial interface.
 *
 *	This function does not actually send the packet to serial interface but simply adds the packet into serial send buffer.
 *	As the buffer is a FIFO the packet will be placed as the last entry in the buffer. The address of the buffer is required so that we 
 *	can have more than one send buffer in the future e.g. normal and priority buffers.
 *	@param bufaddr The address of the buffer where the packet is placed.
 *	@param packet The packet which is to placed in to the buffer.
 *	@return 0 (zero) is returned if the packet was succesfully placed in to the buffer, -1 if buffer was full.
 */
int send_to_serial(struct fifo_buffer_t *bufaddr, char packet[MAX_NRP_PACKET_SIZE]);
