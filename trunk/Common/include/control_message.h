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
 * \file control_message.h
 * \brief control_message type defined
 *
 *  nanoStack: control-message structure including config messages
 *   
 *	
 */

#ifndef _CONTROL_MESSAGE_H
#define _CONTROL_MESSAGE_H

#include "address.h"
#include "rf_802_15_4.h"

extern void enable_router_features( void );
/** Mac-layers control messages */
typedef struct
{
	mac_control_id_t message_id;									/*!< message id. */
	union
	{
			rf_802_15_4_beacon_notify_t 	beacon_notify;			/*!< Beacon notification used in superframe-state. */
			rf_802_15_4_assoc_req_t		assoc_req;				/*!< Association request. */
			uint8_t						reset_pib;				/*!< Reset Mac-PIB. */
			rf_802_15_4_rx_enable_req_t	rx_enable_req;			/*!< RX enable request. */		
			rf_802_15_4_scan_req_t		scan_req;				/*!< Scan request. */
			rf_802_15_4_scan_confirm_t	scan_confirm;			/*!< Result of scan request. */
			rf_802_15_4_comm_status_ind_t	comm_status_ind;		/*!< Communications status.  */
			rf_802_15_4_disassocite_req_t	diss_assoc_req;			/*!< Disassociation request. */
			rf_802_15_4_synch_req_t		synch_req;				/*!< Synchronize request by next beacon. */
			rf_802_15_4_synch_lost_t	synch_loss_reason;		/*!< Synchronize-loss reason. */
			rf_802_15_4_poll_req_t		poll_req;				/*!< Poll data from coordinator request. */				
			assoc_status_t			assoc_confirm;			/*!< Confirm for assocation request.  */
#ifdef MAC_FFD
			rf_802_15_4_assoc_ind_t		assoc_ind;				/*!< Assocation request indication.  */
			rf_802_15_4_assoc_response_t	assoc_response;			/*!< Response for assocation request.  */
			rf_802_15_4_gts_req_t		gts_req;				/*!< GTS request. */
			rf_802_15_4_gts_confirm_t	gts_confirm;			/*!< Confirm for GTS request. */
			rf_802_15_4_gts_ind_t		gts_ind;				/*!< GTS request indication.  */
			rf_802_15_4_orphan_ind_t	orphan_ind;				/*!< Orphan indication.  */
			rf_802_15_4_orphan_response_t	orphan_response;		/*!< Response for Orphan device.  */
			rf_802_15_4_start_req_t		start_req;				/*!< Start PAN-network request.  */
			rf_802_15_4_start_router_t  router_start;
#endif
			
	}message;													/*!< Name of the control-message union  */
} MACconf_t;

/** ICMP and IP control messages */
typedef enum
{
	IP_ADDRESS__MODE_SETUP,				/*!< . */		
	ROUTER_DISCOVER ,				/*!< . */	
	ROUTER_DISCOVER_RESPONSE,		/*!< . */	
	ROUTER_ADVER_SEND,				/*!< . */
	ECHO_REQ,
	ECHO_RES,
	BROKEN_LINK,
	NO_ROUTE_TO_DESTINATION	,
	PENDING_TIMEOUT,
	TOO_LONG_PACKET,
	ASSOCIATION_WITH_COORDINATOR,
	ASSOCIATION_LOST,
	COORDINATOR_STARTED,
	GATEWAY_STARTED,
	NOT_ASSOCIATED,
	DATA_BACK_NO_ROUTE,
	MAC_QUEUE_FULL,
	MAC_CCA_ERR,
	CONTROL_NONE_RX,
}ip_control_id_t;

/** ICMP error type*/
typedef enum
{
	NO_ROUTE_TO_DESTINATION_TYPE,
	ADMIN_DISCARD_TYPE,
	PORT_UNREACHED_TYPE,
	BROKEN_LINK_TYPE	
}icmp_error_status;





typedef struct stack_info_t
{
	ip_control_id_t id;
	buffer_t 	*buf;
}stack_info_t;


typedef struct
{
	icmp_error_status 	reason;
	uint8_t 	address[8];				/*!< .*/
	uint8_t		type;		/*!< . */
}ip_broken_link_notify_t;

typedef struct
{
	icmp_error_status 	reason;
	uint8_t 			address[8];				/*!< .*/
	uint8_t				type;		/*!< . */
}ip_error_status_t;


/** IP-layers control message */
typedef struct
{
	ip_control_id_t 							message_id;									/*!< message id. */
	union
	{
		ip_broken_link_notify_t				broken_link_detect;
		ip_error_status_t					error_status;
	}message;
} IPconf_t;


/** control-messages structure */
typedef struct
{
	union
	{
		MACconf_t mac_control;		/*!< Mac-layers control messages. */
		IPconf_t	ip_control;		/*!< ICMP & IP layers control messages. */
	}message;						/*!< Name of the message union  */
} control_message_t;

/** network manager modules state enumeration */
typedef enum
{
	CORD_STATE		,		/*!< Coordinator state which handles received mac-command like assocation req etc. */		
	CLIENT_STATE  		,	/*!< Client state: functions for superframe-state. */
    ROUTER_STATE,			/*!< Router state:not ready with this release. */	
	NWK_DISCOVER_STATE	,	/*!< Discover state: module waiting response for active-scan. */	
	NWK_FORMATION_STATE	,	/*!< Formation state: module waiting response for ed-scan. */	
	NWK_ASSOC_STATE			/*!< Association state: module waiting response for association request. */	
}nwk_manager_state_t;

/** child list */
typedef struct
{
	uint8_t 	short_address[2];		/*!< Child allocated nwk_address. */
	uint8_t		long_address[8];		/*!< Child long address. */
	uint8_t		cap_info;				/*!< MAC-capability info. */
}child_list_t;

/** Parent info */
typedef struct
{
	uint8_t 	short_address[2];		/*!< Coordinators short address. */
	uint8_t		long_address[8];		/*!< Coordinators long address. */
	uint8_t		address_type;			/*!< Coordinators address type. */ 
	uint8_t		pan_id[2];				/*!< PAN-id      */
	uint8_t		channel;				/*!< PAN's logical channel. */ 
	uint8_t		beacon_order;			/*!< PAN Beacon order. */ 
	uint8_t		superframe_order;		/*!< PAN Superframe order. */ 
	uint8_t		assoc_status;			/*!< Assoc status with coordinators. */
}parent_info_t;

/** network manager modules information base */
typedef struct
{
#ifdef MAC_FFD
	uint8_t 	child_count;		/*!< Child count. */
	uint8_t     pan_id[2];
#else
	parent_info_t 	parent_info;		/*!< Parent Info */
#endif
}nwk_manager_pib_t;

extern ip_control_id_t parse_event_message(buffer_t *buf);

#endif /*CONTROL_MESSAGE_H*/
