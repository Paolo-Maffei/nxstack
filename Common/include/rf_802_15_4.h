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
 * \file rf_802_15_4.h
 * \brief RF 802.15.4 modules constant and stuctures.
 *
 */



#ifndef RF_802_15_4_H
#define RF_802_15_4_H
#include "buffer.h"
#include "rf.h"
/** Frame control field, just for 802_15_4_RAW */
#define TRUE					1
#define FALSE					0
#define MAX_RE_TX				3
#define DELTA_BEACON_RX			4

#define MAX_PENDING_BUFFER_COUNT 3
#define MIN_PENDING_BUFFER_COUNT 3

#define PING_RESPONSE_MAX		10

/** MAC Header masks */
#define FC_DST_MODE			0x0C    
#define FC_DST_ADDR_NONE	0x00
#define FC_DST_16_BITS		0x08
#define FC_DST_64_BITS		0x0C
#define FC_SRC_MODE			0xC0   
#define FC_SRC_ADDR_NONE	0x00
#define FC_SRC_16_BITS		0x80
#define FC_SRC_64_BITS		0xC0 
#define FC_INTRA_PAN		0x40     
#define FC_ACK				0x20     
#define FC_SEC				0x08 
#define FC_PENDING			0x10    
#define FC_FRAME_TYPE_MASK 	0x07    
#define FC_BEACON_FRAME		0x00     
#define FC_DATA_FRAME		0x01     
#define FC_ACK_FRAME		0x02     
#define FC_COMMAND_FRAME	0x03
#define FC_NOT_DEFINED_FRAME 0x04

#define FC_ACK_PENDING		(FC_ACK_FRAME+FC_PENDING)
#define FC_ACK_NO_PENDING	(FC_ACK_FRAME)

#define ADDR_MODE_64		0x03
#define ADDR_MODE_16		0x02
//#define ADDR_NONE			0x00

/** Mac Capability constant */
#define MC_ALTER_COORD		0x01
#define MC_FFD_DEVICE		0x02
#define MC_POWER_SOURCE		0x04
#define MC_RX_ON_WHEN_IDLE	0x08
#define MC_SECURITY			0x40
#define MC_ALLOCATED_ADDR	0x80

/** Mac-scan constant */
#define ACTIVE_SCAN			0x01
#define PASSIVE_SCAN		0x02
#define ED_SCAN				0x00
#define ORPHAN_SCAN			0x03
#define SCAN_ALL			0xffff

/** Mac-command-id */
#define	CMD_ASSOC_REQ 				1
#define	CMD_ASSOC_RESPONSE			2
#define	CMD_DISSASSOC_NOTIFY		3
#define	CMD_DATA_REQ				4
#define	CMD_PAN_ID_CONFLIGT_NOTIFY 	5
#define	CMD_ORPHAN_NOTIFY			6
#define	CMD_BEACON_REQ				7
#define	CMD_CORDINATOR_REALIGN		8
#define	CMD_GTS_REQ					9

/** Superframe timing value */
#define BI_12	40673
#define BI_11	20336
#define BI_10	10166
#define BI_9	5082
#define BI_8	2541
#define BI_7	1270
#define BI_6	634
#define BI_5	316
#define BI_4	157

#if 1
extern void rf_802_15_4_slot_timing(uint8_t slot, uint16_t *synch_value, uint16_t *last_slot);
#else
/** Timeslot synch value for superframe */
#define SLOT_12	40672
#define SLOT_11	20335
#define SLOT_10	10165
#define SLOT_9	5081
#define SLOT_8	2540
#define SLOT_7	1269
#define SLOT_6	633
#define SLOT_5	315
#define SLOT_4	156

/** Last slots syncronize values in ms*/
#define LAST_SLOT_12	4052
#define LAST_SLOT_11	2018
#define LAST_SLOT_10	1001
#define LAST_SLOT_9	493
#define LAST_SLOT_8	239
#define LAST_SLOT_7	113
#define LAST_SLOT_6	48
#define LAST_SLOT_5	16
#define LAST_SLOT_4	1
#endif

/** MAC Constant based on to IEEE 802.15.4 standard */
#define aMaxPHYPacketSize       127
#define aTurnaroundTime         12 // symbol periods
#define aUnitBackoffPeriod      100
#define aBaseSlotDuration       60
#define aNumSuperframeSlots     16
#define aBaseSuperframeDuration (aBaseSlotDuration * aNumSuperframeSlots)
#define aMaxBE                  5
#define aMaxBeaconOverhead      75
#define aMaxBeaconPayloadLength (aMaxPHYPacketSize - aMaxBeaconOverhead)
#define aMaxFrameOverhead       25
#define aBasicFrameOverhead	13
#define aMinFrameOverhead       9
#define aMaxFrameResponseTime   1220
#define aMaxFrameRetries        3
#define aMaxLostBeacons         2
#define aMaxMACFrameSize        (aMaxPHYPacketSize - aMaxFrameOverhead)
#define aMaxSIFSFrameSize       18
#define aMinCAPLength           440
#define aMinLIFSPeriod          640
#define aMinSIFSPeriod          192
#define aResponseWaitTime       (32 * aBaseSuperframeDuration)
#define SYNCH_LOST_TOTAL		(aMaxLostBeacons+2)

/** Mac layers states */ 

typedef enum
{
	TX_IDLE  						= 0,
	TX_RE_CCA_CHECK					= 1,	
	TX_RE_TX						= 2,	
}mac_tx_engine_state_t;


typedef enum
{
	MAC_RX_MES  				= 0,
	MAC_TX_MES					= 1,	
	MAC_15_4_CONTROL			= 2,
	MAC_TIMER_INT_CB			= 3,
	MAC_PENDING					= 4,
}mac_event_id_t;

typedef enum
{
	WAITING_ACK  						= 0,
	WAITING_ACK_PENDING					= 1,	
	WAITING_EVENT_LAUNCH_TX				= 2,
	WAITING_EVENT_LAUNCH_TX_DATA_REQ	= 3,
	WAITING_DATA						= 4,
	TIMER_IDLE							= 5,
	MAC_STATUS_CHECK					= 6,	
}mac_timer_event_t;



/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	mac_event_id_t id;
	buffer_t 	*buf;
} mac_15_4_event_t;


/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	uint8_t count;
	buffer_t 	*buf[4];
} mac_tx_pool_t;


/** Mac layers states */ 
typedef enum
{
	MAC_STATE_NORMAL 						= 0,	
	MAC_WAITING_DATA_REQ_TIMEOUT			= 1,	
	MAC_WAITING_DATA_FROM_SERVER			= 2,
	MAC_ACTIVE_SCAN								= 3,
	MAC_ORPHAN_SCAN								= 4,
	MAC_SYNCHRONIZE_WITH_SERVER								= 5,	
}mac_internal_state_t;


/** Mac layers states */ 
typedef enum
{
	BEACON_TRACK  		= 0,		/*!< Active scan & beacon tarckin state,forward only Beacon frame. */
	DISSCONNECT		= 1,	/*!< Not associated, forward only mac-command-frames and ack. */
	CONNECT			= 2,		/*!< Device is complete associated with coordinator, forward data- and mac-command frames. */
	AD_HOC			= 3,			/*!< AD-HOC state, forward data- and ack-frames. */
}mac_state_t;

/** Association response status */ 
typedef enum
{
	ASSOC_SUCCESSFUL	= 0,	/*!< Association succesfully. */
	PAN_AT_CAPACITY  	= 1,	/*!< PAN network is full. */
	ACCESS_DENIED		= 2		/*!< Access denied by ACL-list. */
}assoc_status_t;

/** MAC modules running mode */
typedef enum
{
	AD_HOC_MODE = 0,				/*!< AD-HOC MODE. */
	BEACON_ENABLE_MODE = 1,			/*!< Beacon enable client mode. */
	BEACON_ENABLE_COORD_MODE = 2,	/*!< Beacon enable Coordinator mode. */
	ROUTER_MODE = 3					/*!< Beacon enable Router mode */
}mac_runnin_mode_t;

/** MAC logical Device type */
typedef enum
{
	MAC_RFD_TYPE = 0,				/*!< Client use RFD, not use dynamical memory management. */
	MAC_FFD_TYPE = 1				/*!< Coordinator / Router use FFD, use dynamical memory management. */
}mac_device_type_t;

#define MAC_ASSOCPERMIT			0x01
#define MAC_ASSOC_CORD			0x02
#define MAC_BATT_LIFE_EXT		0x04
#define MAC_RX_ON_WHEN_IDLE		0x08
#define MAC_PROMISCOUS_MODE		0x10
#define MAC_GTS_PERMIT			0x20
#define MAC_SUPPORT_SHORT_ADDR	0x40
#define MAC_AUTO_REQ			0x80


/** Mac layers PAN information base */ 
typedef struct {
	uint8_t rf_802_15_4_ext_addr[8];	/*!< Nodes 64-bit IEEE Address. */
	uint8_t rf_802_15_4_short_addr[2];	/*!< Nodes 16-bit Short Address. */
	uint8_t pan_id[2];			/*!< 16-bit Network id. */
	addrtype_t coord_addr_mode;
	uint8_t coord_ext_addr[8];		/*!< PAN coordinator's 64-bit IEEE Address. */
	uint8_t coord_short_Addr[2];		/*!< PAN coordinator's 16-bit Short Address. */
	uint8_t data_sqn;			/*!< Data & mac command packet sqn. */
	uint8_t max_csma_backoffs;
	uint8_t min_be;
	uint8_t last_tx_sqn;			/*!< Last transmitted tx number. */
	mac_state_t state;			/*!< Shows Mac layer state. */
	mac_runnin_mode_t	mode;
	uint8_t	logical_channel;		/*!< Nodes Logical channel. */
	uint8_t mac_options;			/*!<Bit mask for: MAC_ASSOCPERMIT=0x01, MAC_ASSOC_CORD=0x02,MAC_BATT_LIFE_EXT=0x04, MAC_RX_ON_WHEN_IDLE=0x08, MAC_PROMISCOUS_MODE=0x10,MAC_GTS_PERMIT=0x20, MAC_SUPPORT_SHORT_ADDR=0x40  */
	uint8_t beacon_order;			/*!< Define beacon interval at superframe mode. */
	uint8_t superframe_order;		/*!< Define Superframe activated period length at superframe mode. */
	mac_device_type_t device_type;
	uint8_t beacon_sqn;			/*!< Beacon packet sqn. */
	uint8_t transaction_persistence_time;
#ifdef IMPROVED_MAC /*Not used now*/
	uint32_t beacon_tx_time;
	uint16_t transaction_persistence_time;
	uint8_t battlife_ext_periods;
	uint8_t ack_wait_duration;
	uint8_t	beacon_payload;
	uint8_t	beacon_payload_length;
	uint8_t gts_permit;
#endif

} rf_802_15_4_pib_t;


/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	uint8_t 	coord_addr_mode;		/*!< Coordinators addressmode. */
	uint8_t 	panid[2];				/*!< PAN-id. */
	uint8_t 	address[8]; 			/*!< Coordinators address. */
	uint8_t 	security_use;			/*!< PAN security status. */
	uint8_t 	superframe_spec[2];		/*!< Superfarme specification indicates Beacon interval, superframe duration etc. */
	uint8_t 	gts_permit;				/*!< Does PAN permit GTS. */
	uint8_t 	assoc_permit;			/*!< Does PAN permit assocation. */
	uint8_t 	link_quality;			/*!< Link quality indication to coordinator. */
	uint8_t 	logical_channel;		/*!< PAN networks logical channel. */
#ifdef IMPROVED_MAC /*Not used now*/
	uint8_t		security_failure;
	uint32_t 	time_stamp;
	uint8_t 	acl_entry;			/*!< ACL-status. */
#endif
} pan_descriptor_t;


/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	buffer_t	*buf;
	uint8_t 	sqn;
	uint8_t		re_tx;
} ack_waiting_pool_t;


/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	sockaddr_t		src;
	portTickType	time;
	int8_t			rssi;
} disc_res_t;

/** PAN-descriptor */ /* This structure fill by beacon frame */
typedef struct {
	disc_res_t	result[PING_RESPONSE_MAX];
	uint8_t 	count;
} discover_res_t;


/** RF802_15_4 control message-id enumeration */
typedef enum
{
  BEACON_NOTIFY = 0,                    /*!< Beacon notify message to upper layer. */
  ASSOC_REQ = 1,           				/*!< Association request from upper layer.  */
  ASSOC_IND = 2,   				       	/*!< Association indication to upper layer. */
  ASSOC_RESPONSE = 3,      				/*!< Association response from upper layer waiting for client data request mac-command. */
  ASSOC_CONFIRM = 4,					/*!< Association confirm to upper layer waiting for client data request mac-command. */
  DISSASSOC_REQ = 5,       				/*!< Dissassocation request from upper layer. */
  DISSASSOC_IND = 6,                   	/*!< Dissassocation indication to upper layer. */
  GET_REQ = 7,                     		/*!< Get Mac-PIB attribute value message from upper layer. */
  GET_CONFIRM = 8,						/*!< Confirm message to GET request to upper layer. */
  SET_REQ = 9,                     		/*!< Set Mac-PIB attribute by gives parameter value message from upper layer. */
  SET_CONFIRM = 10,						/*!< Confirm message to SET request to upper layer. */
  GTS_REQ = 11,							/*!< GTS request message from upper layer. */
  GTS_CONFIRM = 12,						/*!< Confirm message to GTS request message to upper layer. */
  GTS_IND = 13,							/*!< GTS indication message to upper layer. */
  POLL_REQ = 14,						/*!< Poll request message from upper layer.*/
  COMM_STATUS = 15,						/*!< Genral communication status message to upper layer. */
  ORPHAN_IND = 16,						/*!< Orphan indication message to upper layer. */
  ORPHAN_RESPONSE = 17,					/*!< Orphan response message message from upper layer. */
  SCAN_REQ = 18,						/*!< Scan request message from upper layer. */
  SCAN_CONFIRM = 19,					/*!< Scan result message to upper layer. */
  START_REQ = 20,						/*!< Start PAN-network request message from upper layer. */
  START_CONFIRM = 21,					/*!< Start PAN-network confirm message to upper layer. */
  RX_ENABLE_REQ = 22,					/*!< Receiver enable set by gives time parameter request message from upper layer. */
  RX_ENABLE_CONFIRM = 23,				/*!< Confirm message to RX_ENABLE_REQ to upperlayer. */
  SYNCH_REQ = 24,						/*!< Synchronization request message from upperlayer. */
  SYNCH_LOSS_IND = 25,					/*!< Synchronization lost indication message to upperlayer. */
  RESET_REQ = 26,						/*!< Reset MAC-PIB request message from upper layer. */
  RESET_CONFIRM = 27,					/*!< Reset MAC-PIB confirm message to upper layer. */
  ROUTER_START = 28,					/*!< Start router mode with defined channel, PAN-id & address. */
  PEND_REQ = 29, 						/*!< FFD device can store data for specified client. */
  PEND_DATA_NOT_DOWNLOAD = 30			/*!< When RFD haven't dowload pending data from coordnator. */
} mac_control_id_t;   

typedef enum
{
	MAC_SUCCESS,
	MAC_FAILURE
}mac_error_t;
/** MAC-PIB attribute enumeration ID's */
typedef enum mac_pib_enum_t
{
	MAC_CURRENT_CHANNEL ,			/*!< Logical channel. */ 
	MAC_IEEE_ADDRESS ,				/*!< Long IEEE address. */ 
	MAC_CORD_IEEE_ADDRESS,			/*!< Coordinators Long IEEE address. */ 
	MAC_PAN_ID,						/*!< PAN id. */ 
	MAC_SO,							/*!< Superframe order. */ 
	MAC_BO, 						/*!< Beacon Order. */ 
	MAC_SHORT_ADDRESS,				/*!< Hort address. */					
    PENDING_TTL,					/*!< How many 15 seconds period FFD device store pending data buffer. */
	RUNNING_MODE,					/*!< Mac running mode. */
	ASSOC_PERMIT					/*!< Handle assocation permit flag. */
} mac_pib_enum_t;

/** MAC CSMA-process return values */
typedef enum csma_response_t
{
	MAC_TX_SUCCESS = 0 ,			/*!< TX complete. */ 		
	MAC_NO_ACK	= 1,				/*!< NO ack from destination. */ 
	MAC_CCA_BUSY = 2,				/*!< Channel too busy. */ 
	MAC_CCA_OK= 3,					/*!< Channel free. */ 
	MAC_NO_TIMESLOT = 4				/*!< No time slot before next beacon. */ 
} csma_response_t;


/** Synhrpnize lost reason enumeration ID's */
typedef enum synch_lost_reason_t
{
	PAN_ID_CONFLIGTH = 0,	/*!< Detect same PAN-id, but source is not current coordinator. */ 
	REALIGMENT ,			/*!< Coordinator has change network options. */ 
	BEACON_LOST,			/*!< Not received beacon. */
	NO_ACK_FROM_COORD		/*!< Coordinator has not send response for RFD device*/ 
} synch_lost_reason_t;


typedef struct {
mac_pib_enum_t id;
union
{
	uint8_t channel;
}param;
} mac_param_t;


/** Mac-header analyze response. */
typedef struct {
uint8_t f_type;				/*!< 802.15.4 MAC frame type DATA / ACK / BEACON / MAC_COMMAND. */ 
uint8_t ack_req;			/*!< Indicated packet ack requirement. */ 
uint8_t sqn;				/*!< Packet sequency number. */ 
uint8_t sec;				/*!< Security use (Not used). */ 
} rf_mac_header_analyze_t;


/** Beacon notification using in superframe-state. */
typedef struct {
	uint8_t bsn;						/*!< Sequence number of beacon frame. */
	pan_descriptor_t panDescriptor;		/*!< Description of PAN-network. */	
	uint8_t pend_addr_spec;				/*!< Shows pendind address count. */
	uint8_t pend_address_list[56];		/*!< Pending address-list. */
	uint8_t sdu_length;					/*!< Beacon payload length. */
	uint8_t sdu[5]; 					/*!< Beacon payload. */
} rf_802_15_4_beacon_notify_t;

/** Association request. */
typedef struct {
	uint8_t logical_channel;		/*!< Networks logical channel. */
	uint8_t cord_addrmode;			/*!< Coordinators address mode. */
	uint8_t cord_panid[2];			/*!< Network PAN-id. */
	uint8_t coord_address[8];		/*!< Coordinators address. */
	uint8_t cap_info;			/*!< Devices Mac-capability info. */
	uint8_t security_enable;		/*!< Security status. */
} rf_802_15_4_assoc_req_t;

#ifdef MAC_FFD
/** Association indication. */
typedef struct {
	uint8_t device_address[8];	/*!< Devices address which want to join. */
	uint8_t cap_info;			/*!< Devices Mac-capability info. */
	uint8_t security_use;		/*!< Security status. */
	uint8_t acl_entry;			/*!< ACL-entry status. */
} rf_802_15_4_assoc_ind_t;

/** Association indication. */
typedef struct {
	uint8_t 		device_address[8];			/*!< Address for assocation response. */
	uint8_t 		assoc_short_address[2];		/*!< Devices Allocated short-address. */
	assoc_status_t 	status;						/*!< Status of association. */
	uint8_t 		security_enable;			/*!< Security status. */
	uint8_t 		assoc_permit_flag;			/*!< Association permit flag state later. */
} rf_802_15_4_assoc_response_t;

/** Mac data buffer for Association response frame, only coordinator using. */
typedef struct {
	uint8_t address_type;
	uint8_t address[8];		/*!< Destination address for buffer. */
	uint8_t	buffer_type;
	uint8_t buffer[4]; 		/*!< Buffer for assocation response. */
} mac_data_buffer_t;

/** ED-scan confirm handle variable. */
typedef struct {
	uint8_t channel;		/*!< Best channel to start PAN-network. */
	int8_t ed_value; 		/*!< Next channel value. */
} channel_ed_t;


typedef struct {
	uint8_t 	ttl;		/*!< Best channel to start PAN-network. */
	buffer_t 	*buf; 	/*!< Buffer-array. */
} pend_data_t;


/** Pending buffer. */
typedef struct {
	uint8_t 	buffer_count;								/*!< Shows buffer count. */
	pend_data_t p_data[MAX_PENDING_BUFFER_COUNT]; 	/*!< Buffer-array. */
} mac_pending_buffer_t;

/** GTS-request. */
typedef struct {
	uint8_t GTS_characterist;				/*!< Characteristics of GTS bitmask, shows GTS length, type and direction. */
	uint8_t security_enable;				/*!< Security status. */
} rf_802_15_4_gts_req_t;

/** GTS-confirm. */
typedef struct {
	uint8_t GTS_characterist;				/*!< Characteristics of GTS bitmask, shows GTS length, type and direction. */
	uint8_t status;							/*!< status of GTS-request. */
} rf_802_15_4_gts_confirm_t;

/** GTS-indication. */
typedef struct {
	uint8_t device_address[8];			/*!< Address for devices which want GTS. */
	uint8_t GTS_characterist;			/*!< Characteristics of GTS bitmask, show GTS length, type and direction. */
	uint8_t security_use;				/*!< Security status. */
	uint8_t acl_entry;					/*!< ACL-entry status. */
} rf_802_15_4_gts_ind_t;

/** Orphan-indication. */
typedef struct {
	uint8_t orphan_address[8];		/*!< Address of the orphaned devices. */	
	uint8_t security_use;			/*!< Security status. */
	uint8_t acl_entry;				/*!< ACL-entry status. */
} rf_802_15_4_orphan_ind_t;

/** Orphan-response. */
typedef struct {
	uint8_t orphan_address[8];		/*!< Address of the orphaned devices. */
	uint8_t short_address[2];		/*!< Allocated short-address if device is associated with coordinator. */
	uint8_t	assoc_member;			/*!< Association status flag. */
	uint8_t security_enable;		/*!< Security status. */
} rf_802_15_4_orphan_response_t;

/** Start PAN-network request. */
typedef struct {
	uint8_t panid[2];			/*!< Networks PAN-id. */
	uint8_t	logical_channel;	/*!< Logical channel for network. */
	uint8_t	beacon_order;		/*!< Beacon-order which shows beacon interval. */
	uint8_t	superframe_order;	/*!< Superframe-order which shows superframe duration. */
	uint8_t	pan_cordinator;		/*!< TRUE if device is coordinator FALSE for Router. */
	uint8_t	batt_life_ext;		/*!< TRUE if Battery life extension supported. */
	uint8_t	cord_realigment;	/*!< TRUE if Coordinator has changed options of network. */
	uint8_t	security_enable;	/*!< Security status. */
} rf_802_15_4_start_req_t;

/** Start PAN-network request. */
typedef struct {
	uint8_t panid[2];			/*!< Routers PAN-id. */
	uint8_t	logical_channel;	/*!< Logical channel for network. */
} rf_802_15_4_start_router_t;
#endif

/** Disassocation request. */
typedef struct {
	uint8_t device_address[8];			/*!< Address of the devices. */
	uint8_t dissassocation_reason;		/*!< Reason flag, 1=Coordinator requested, 2=device requested. */
	uint8_t security_enable;			/*!< Security status. */
} rf_802_15_4_disassocite_req_t;		

/** Disassocation indication. */
typedef struct {
	uint8_t device_address[8];			/*!< Address of the disassocated devices. */
	uint8_t dissassocation_reason;		/*!< Reason flag, 1=Coordinator requested, 2=device requested. */
	uint8_t security_use;				/*!< Security status. */
	uint8_t acl_entry;					/*!< ACL-entry status. */
} rf_802_15_4_disassocite_ind_t;

/** RX-enable request. */
typedef struct {
	uint8_t deferpermit;				/*!< TRUE, if device can turn on next superframe after beacon. */	
	uint8_t rx_on_time[3];				/*!< RX_ON after this symbol duration. */	
	uint8_t rx_on_duration[3];			/*!< RX_ON duration time. */	
} rf_802_15_4_rx_enable_req_t;

/** Scan request. */
typedef struct {
	uint8_t scan_type;					/*!< 0=ED-scan, 1=Active-scan, 2=Passive-scan & 3=Orphan-scan. */	
	uint16_t scan_channels;				/*!< Channel list. */	
	uint8_t scan_duration;				/*!< Scan duration exponent value. */	
} rf_802_15_4_scan_req_t;

/** Scan confirm. */
typedef struct {
	uint8_t	status;						/*!< Status of scan prosess. */
	uint8_t scan_type;					/*!< 0=ED-scan, 1=Active-scan, 2=Passive-scan & 3=Orphan-scan. */
	uint16_t unscanned_channels;		/*!< Unscanned_channels. */
	uint8_t	resultlist_size;			/*!< Result-list size. */
	int8_t	ed_list[16];				/*!< Result of the ED-scan. */
	pan_descriptor_t pan_descriptor[4];	/*!< Result of the Passive or Active scan, list of network descriptions found. */
} rf_802_15_4_scan_confirm_t;

/** Commication status. */
typedef struct {
	uint8_t panid[2];					/*!< PAN-id. */	
	int8_t dest_addrmode;				/*!< Destination addressmode. */
	uint8_t dest_address[8];			/*!< Destination address. */
	uint8_t src_addrmode;				/*!< Source addressmode. */
	uint8_t src_address[8];				/*!< Source address. */
	uint8_t	status;						/*!< Status of the commications. */
} rf_802_15_4_comm_status_ind_t;


/** Synchronize request. */
typedef struct {
	uint8_t	logical_channel;		/*!< Logical channel. */
	uint16_t	synch_timer;		/*!< Synch timer value, based on superframe setups. */
	uint8_t		b_order;			/*!< Coordinators Beacon order that value setup synchronize. */
	uint8_t		s_order;
	uint16_t	last_slot_synch;	/*!< Last slot time synchronize value. */
} rf_802_15_4_synch_req_t;


/** Synchronize lost indication. */
typedef struct {
	uint8_t	logical_channel;			/*!< Realigment new Logical channel. */
	uint8_t	pan_id[2];				/*!< Realigment new Network id. */
	synch_lost_reason_t synch_lost_reason;		/*!< Lost reason */
} rf_802_15_4_synch_lost_t;

/** Poll request. */
typedef struct {
	uint8_t cord_addrmode;			/*!< Coordinators addressmode. */
	uint8_t cord_panid[2];			/*!< Coordinators PAN-id. */
	uint8_t cord_address[8];		/*!< Coordinators address. */
	uint8_t	security_enable;		/*!< Securitry status. */		
} rf_802_15_4_poll_req_t;


extern void rf_802_15_4_send_beacon(uint8_t param);
extern void rf_802_15_4_pib_reset(void);
extern void rf_802_15_4_store_address_to_ram(void);
extern void rf_802_15_4_ip_layer_address_mode_set(uint8_t support_short_addr);
extern portCHAR get_coord_address(sockaddr_t *address);
extern void mac_set_mac_pib_parameter(uint8_t *pointer, mac_pib_enum_t type);
extern void mac_handle_address_decoder(rf_address_mode_t mode);
extern void mac_assoc_permit_false(void);
extern portCHAR mac_pending_req(buffer_t *buf);
extern uint8_t *mac_get_mac_pib_parameter(mac_pib_enum_t type);
extern mac_error_t mac_pib_set( mac_param_t *par);

#endif /*RF_802_15_4_H*/
