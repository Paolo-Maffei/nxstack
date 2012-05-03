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


/**
 *
 * \file ssi.h
 * \brief SSI protocol sensor structure.
 *
 *  SSI protocol headers and structures.
 *   
 * SSI Protocol Structure
 *
 * ADDR[1] | CMD[1] - Always present
 * 
 * ID[2] - Typical sensor ID
 * ID[2] & 0xC000 == 0xC000 - Sensor description tuple 0x3F00 = GROUP_MASK
 *
 * TYPE[1] | INS[1] | DATA[4] | STAT[1] - For data response
 *
 * TYPE[1] | INS[1] | ATTR[1] | DATA[4] or ASCII[16] - For configuration
 *
 * SSI Protocol commands
 *
 * Capital = no CRC,
 * lower case = CRC
 *
 * q - Query presence on SSI device (respond with q)
 * c - Discover sensors (respond with n)
 * n - Discovery reply
 * z - Reset sensor device
 * g - Get configuration data from sensor
 * s - Set configuration data of a sensor
 * w - Like s but for ascii data
 * x - Configuration response to g
 * i - Configuration response to s
 * r - Request sensor data
 * d - Data response from sensor
 * p - Read/write free-form text (for protocol extensions)
 *
 */


#ifndef _SSI_H
#define _SSI_H

/*We are version 1.0*/
#define SSI_VERSION_MAJOR 1
#define SSI_VERSION_MINOR 00

/** SSI command enumeration */
typedef enum ssi_cmd_t
{
	SSI_CMD_QUERY = 'Q',
	SSI_CMD_QUERY_REPLY = 'A',
	SSI_CMD_DISCOVERY = 'C',
	SSI_CMD_DISCOVERY_REPLY = 'N',
	SSI_CMD_DATA_REQUEST = 'R',
	SSI_CMD_DATA_RESPONSE = 'D',
	SSI_CMD_DATA_RESPONSE_NO_STATUS = 'V',
	SSI_CMD_GET_CONFIGURATION_DATA = 'G',
	SSI_CMD_SET_CONFIGURATION_DATA = 'S',
	SSI_CMD_CONFIGURATION_DATA_RESPONSE = 'X',
	SSI_CMD_BUFFERED_RESPONSE = 'B',
	SSI_CMD_OBSERVER_START = 'O',
	SSI_CMD_OBSERVER_RESPONSE = 'Y',
	SSI_CMD_OBSERVER_STOP = 'K',
	SSI_CMD_FREE_DATA = 'F',
	SSI_CMD_RESET = 'Z'
}ssi_cmd_t;

#define SSI_CRC_MASK 0x20
/* Returns TRUE if a command is the CRC version */
#define SSI_CRC_CHECK(x)  (x & SSI_CRC_MASK)

#define SSI_ADDRESS_ALL '?'		/* Broadcast SSI address for discovery */

#define SSI_TYPE_DEVICE 0x00    /* Identifies the sensor device in general */
#define SSI_TYPE_ALL    0xFF    /* All types of sensors */
#define SSI_TYPE_END SSI_TYPE_ALL /* End of discovery reply */

/*SSI data types*/
#define SSI_DATA_TYPE_FLOAT 0x00
#define SSI_DATA_TYPE_INT   0x01
#define SSI_DATA_TYPE_CONF  0x02 /*non-displayable configuration sensor*/
#define SSI_DATA_TYPE_INT16   0x04 /*buffering sensor*/
#define SSI_DATA_TYPE_BUFFER  0x08 /*buffering sensor*/

/* Default Instances */

#define SSI_INS_ALL 0xFF        /* All instances of sensors */
#define SSI_INS_END SSI_INS_ALL /* End of discovery reply */
/* Attribute Definitions */
/* Attributes are sensor specific */

#define SSI_ATTR_ALL  0xff    /* All attributes */

#define SSI_ATTR_OFFSET 100   
#define SSI_ATTR_SCALE  101

/*Attribute field types*/
#define SSI_ATTR_NULL 0
#define SSI_ATTR_ASCII_1   1
#define SSI_ATTR_ASCII_2   2
#define SSI_ATTR_ASCII_4   3
#define SSI_ATTR_ASCII_8   4
#define SSI_ATTR_ASCII_16  5
#define SSI_ATTR_ASCII_32  6
#define SSI_ATTR_ASCII_N   7
#define SSI_ATTR_INTD_1    8
#define SSI_ATTR_INTD_10   9
#define SSI_ATTR_INTD_100  10 
#define SSI_ATTR_INTD_1000 11
#define SSI_ATTR_INTD_10000 12
#define SSI_ATTR_INTD_100000 13
#define SSI_ATTR_INTD_1000000 14
#define SSI_ATTR_FLOAT 15

/** SSI sensor structure*/
typedef struct 
{
  uint16_t id;    /*!< The sensor's ID */
  uint8_t unit;   /*!< Sensor's unit type, float or int*/
  int8_t scaler; 	/*!< Unit scaling factor 10's exp for ints, precision for floats*/
  union {               
    uint32_t i; 	/*!< Value: signed 32-bit integer */
    float   f;		/*!< Value: IEEE 32-bit floating point */
  } sdata;         /*!< Current data reading, updated by application, type defined by unit field */
  uint8_t status; /*!< Sensor status flags: enabled, data updated/requested flags */
} ssi_sensor_t;
/*  uint8_t n_attr;*/       /* Number of attributes */
/*  attr_t  *attr;*/        /* Attribute storage */

#define SSI_DATA_TYPE_FLOAT 0x00 /*!< SSI unit type: IEEE 32-bit floating point */
#define SSI_DATA_TYPE_INT   0x01 /*!< SSI unit type: signed 32-bit integer */
#define SSI_DATA_TYPE_CONF  0x02 /*!<non-displayable configuration sensor*/


#define SSI_STATUS_ENABLED  	0x01 /*!< SSI sensor status flag: enabled */
#define SSI_STATUS_REQUESTED	0x40 /*!< SSI sensor status flag: data request pending*/
#define SSI_STATUS_UPDATED  	0x80 /*!< SSI sensor status flag: data updated since last request */

#define SSI_GROUP_MASK 0xC000 /*!< SSI sensor ID grouping mask */

#ifdef SSI_HAVE_CONFIG
/** SSI attribute structure*/
typedef struct 
{
  uint8_t attr_type;   /*!< Attribute ID type*/
  uint8_t value_type;   /*!< Attribute value type*/
  union {               
    uint8_t a[4]; 	/*!< Attribute: ascii data */
    int16_t i; 	/*!< Attribute: signed 16-bit integer */
    float   f;		/*!< Attribute: IEEE 32-bit floating point */
  } attr;         /*!< Current data reading, updated by application, type defined by unit field */
  union {               
    uint8_t a[4]; 	/*!< Value: signed 32-bit integer */
    int16_t i; 	/*!< Value: signed 16-bit integer */
    float   f;		/*!< Value: IEEE 32-bit floating point */
  } value;         /*!< Current data reading, updated by application, type defined by unit field */
} ssi_attr_t;
#endif

/**
 *  SSI sensor value update.
 *
 *	\param   ind       nr of sensor (in array, not ID)
 *  \param   value     new value
 *
 */
extern void ssi_sensor_update(uint8_t ind, uint32_t value);

/**
 *  SSI sensor value update from ISR.
 *
 *	\param   ind       nr of sensor (in array, not ID)
 *  \param   value     new value
 *
 */
extern void ssi_sensor_update_from_ISR(uint8_t ind, uint32_t value);

#endif
