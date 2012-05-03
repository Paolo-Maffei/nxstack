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


/**	@file nRouted.h
 *	The nRouted.h header file
 *
 *	This header file contains some structure and global variable definitions for the nRouted daemon.
 *
 */ 
#ifndef _NROUTED_H

#define _NROUTED_H

#define MAX_NRP_PACKET_SIZE 180

#define SERIAL_ID 1234

#define NROUTED_VERSION "1.0"

int monitor_mode;
void hex_dump(unsigned char *data, int length);

/**	Struct for configuration data.
 *
 *	The nRdconfig_t struct contains fields for all the relevant configuration information for nRouted.
 *
 */ 
struct nRdconfig_t {
	char wdir[128];			/**< Character array containing the working directory path. Default: "/tmp/nRouted/" */
	char serialport[128];	/**< Character array containing the path to serial port device to use. Default: "/dev/ttyS0"  */
	int uselogging;			/**< Whether to log events in to log file. 0=NO, 1=YES. Default: 1 */
	int loglevel;			/**< The amount of information that is logged. 0=low (only starting, exiting and errors are logged), 1=medium (some additional information is also logged), 2=high (extensive logging, almost every action is logged. Be careful with logfile size). Default: 0 */
	char logfile[128];		/**< Character array containing the path to the nRouted logfile. Default: "/var/log/nRouted.log" */
	int tcpport;			/**< The TCP port number to listen for connections. Default: 21780 */
	char tcpaddr[32];		/**< The address of the interface where to bind the TCP server. */
	int tcpallowmulti;		/**< Whether to allow multiple TCP connections from the same IP address. 0=no 1=yes. Default: 1 */
	int tcpmaxconn;			/**< The maximum number of simultaneous TCP connections. Default: 1 */

	int serialbufsize;		/**< How many packets should the serial server sendbuffer have space for. Default: 20 */
	int protocol;					/**< Protocol to register */
};

/** Struct for statistics.
 *
 *	All the statistics from the nRouted are stored in nRdstat_t struct.
 *
 *	@todo The macros for manipulating the statistics struct are implmented but not used anywhere -mjs 
 */
struct nRdstat_t {
	int tcp_connections;			/**< Total number of TCP connections that the server has accepted. */
	int tcp_conns_open;				/**< Number of open TCP connections. */ 
	int connections_dropped;		/**< Total number of TCP connections that were dropped. */
	
	int packets_from_tcp;			/**< Number of packets that have been received by the TCP server. */
	int packets_from_serial;		/**< Number of packets that have been received by the serial server. */
	int set_config_packets_sent;	/**< Number of set_config packets that have been sent to the serial device. */
	int get_config_packets_sent;	/**< Number of get_config packets that have been sent to the serial device. */
	int malformed_packets_tcp;		/**< Number of malformed packets that have been received by the TCP server. */
	int malformed_packets_serial;	/**< Number of malformed packets that have been received by the serial server. */
	
	long int bytes_sent;			/**< Total bytes sent (TCP and serial). */
	long int bytes_received;		/**< Total bytes received (TCP and serial). */
};

/** The statistics struct.
 *
 *	This is the actual declaration for the statistics struct.
 *
 */
struct nRdstat_t nRdstat;

/**	Macros used to manipulate the statistics.
 *
 */
#define INC_TCP_CONNS nRdstat.tcp_connections++							/**< Increase the total number of TCP connections. */
#define INC_TCP_OPEN nRdstat.tcp_conns_open++							/**< Increase the number of open TCP connections. */
#define DEC_TCP_OPEN nRdstat.tcp_conns_open--							/**< Decrease the number of open TCP connections. */
#define INC_TCP_DROPPED nRdstat.connections_dropped++					/**< Increase the number of dropped TCP connections. */
#define INC_PKTS_FROM_TCP nRdstat.packets_from_tcp++					/**< Increase the number of packets received from TCP connections. */
#define INC_PKTS_FROM_SERIAL nRdstat.packets_from_serial++				/**< Increase the number of packets received from serial connections. */
#define INC_SET_CONF_PKTS_SENT nRdstat.set_config_packets_sent++		/**< Increase the number of set_conf packets sent. */
#define INC_GET_CONF_PKTS_SENT nRdstat.get_config_packets_sent++		/**< Increase the number of get_conf packets sent. */
#define INC_MALFORMED_TCP_PKTS nRdstat.malformed_packets_tcp++			/**< Increase the number of malformed packets received from TCP connections. */
#define INC_MALFORMED_SERIAL_PKTS nRdstat.malformed_packets_serial++	/**< Increase the number of malformed packets received from serial connections. */
#define INC_BYTES_SENT nRdstat.bytes_sent++								/**< Increase the number of total bytes sent. */
#define INC_BYTES_RECEIVED nRdstat.bytes_received++						/**< Increase the number of total bytes received. */


/** Configuration struct.
 *	
 *	The global variable that contains the configuration data. 
 *	The default values of some parameters are assigned to the variable in parse_config().
 *
 */
struct nRdconfig_t nRdconf;


/**	Definition for the structure that is used as the entries for the conncetion  table.
 *
 *	An array constructed from this structure is used to store the connection information table i.e. the table which the nRouted checks 
 *	where to relay data coming from the GW dongle. One nRd_conn_table_t structure contains the relevant information for one TCP connection.
 *	The different fields (proto, sadd etc.) are filled in by the process_tcp_connection() function as the application that initiated the TCP
 *	connection sends the necessary information. The format of the packet containing this information is defined in the nanostack-PC manual.
 *	The connection is not active (i.e. no data is relayed to the application) before the application has sent this information. The fields
 *	act as a filters which the nRouted uses to decide where (one or several) to relay a packet after it has been received from the GW dongle.
 *	Not all the fields are mandatory meaning that a) not all the fields are applicable to all possible protocols (e.g. for nanoUDP only port
 *	number is used. See the nanoDUP spcification for more information) and b) limited wildcard functionality is available. See the 
 *	nanoStack-PC manual for more details. More complex filters would be possible to implement (e.g. accepting only packets which contain
 *	signal level information) but usefullness of such extensions is highly questionable.
 *
 *	@todo Add support for IPv6 addresses -mjs
 */
struct nRd_conn_table_t {
	unsigned short int active;				/**< This variable indicates whether the connection is already active and properly configured by the remote application. 0=inactive, 1=active. */

	unsigned int fd;						/**< The file descriptor of the TCP connection. */
	
	unsigned char proto;					/**< The protocol that the application that created this TCP connection wants to receive. Basic types: 0x00=raw (default), 0x01=nanoUDP, 0x02=nanoTCP.See nanoStack PC manual for more details on use. */

	unsigned short int source_port;			/**< The source port number. 0xFFFF means no port set.*/
	unsigned short int destination_port;	/**< The destination port number. 0xFFFF means no port set. */

	unsigned char address_type;				/**< The type of address which is stored into the address union addr. This uses the protocol definitions from the nRP specification. A value of 0xFE means no address has been set.*/
	union {
		unsigned char addr_hw[6];			/**< The IEEE HW address of the origin of the packet. */
		unsigned char addr_15_4_long[8];	/**< The 802.15.4 device long address. */
		unsigned char addr_15_4_short[2];	/**< The 802.15.4 device PAN ID or 802.15.4 device short address. */
		unsigned char addr_str[16];			/**< The IPv4 address in string format. */
		unsigned int addr_32b;				/**< The IPv4 source address where the packet has originated (if applicable) in hardware byte order (little endian). */
	} addr;									/**< This union encapsulates the different address types inside one variable. */
};

/** The connection table array.
 *
 *	This is the actual declaration for the connection array. Memory for this array is reserved in the nRouted.c file. The number of elements
 *	in this array is defined by the TCPMAXCONN configuration parameter.
 *
 */
struct nRd_conn_table_t *nRd_conn_table;

/**	System communications.
 *
 *	If an unrecoverable error occures in some of the daemons threads, the thread must set this system wide variable into 1.
 *	All threads must check this variable from time to time (interval < 3 seconds) to see if they must stop execution and return
 *	to the main function which will then exit the whole daemon.
 */
int haltsystem;

/**	Log file file descriptor.
 *
 *	The global file descriptor variable that is associated to the log file in init_conf() 
 */
FILE *logfp;


/**	Serial port file descriptor.
 *
 *	The global file descriptor variable that is associated to the serial port device file in init_conf() 
 */
FILE *serialfp;

/** Serial port write queue identifier.
 *
 *	This ID is used to send messages to the serial_write() function.
 */
int serial_queue_id;

#endif /* _NROUTED_H */
