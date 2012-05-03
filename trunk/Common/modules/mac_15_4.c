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
 * \file     mac_15_4.c
 * \brief    802.15.4  protocol module.
 *
 *  802.15.4 MAC and CSMA sequence: handler functions.
 *  Modular version for positioning support etc.
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "list.h"

#ifndef MAC_15_4_DEBUG
#undef HAVE_DEBUG
#endif

#include "debug.h"
#include "socket.h"
#include "buffer.h"
#include "bus.h"
#include "module.h"
/*#include "neighbor_routing_table.h"*/
#include "control_message.h"
/*#include "event_timer.h"*/
#include "cipv6.h"
#include "rf.h"
#include "mac.h"

/*
[NAME]
MAC_15_4

[ID]
MODULE_MAC_15_4,

[INFO]
#ifdef HAVE_MAC_15_4
  {mac_init, mac_handle, mac_check, 0, MODULE_MAC_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR mac_init(buffer_t *buf);
extern portCHAR mac_handle( buffer_t *buf );
extern portCHAR mac_check( buffer_t *buf );

/*
	Externals
	*/
#define MAC_TICK_FACTOR 1/32
#define PLATFORM_RAND_SEED 32

extern void timer_mac_launch(uint16_t mac_ticks);
extern void timer_mac_stop(void);

extern void rf_rx_callback(void *param);

extern sockaddr_t mac_short;
extern sockaddr_t mac_long;

/*
	Internals
	*/
#undef FC_DST_MODE
#undef FC_DST_ADDR_NONE
#undef FC_DST_16_BITS
#undef FC_DST_64_BITS

#undef FC_SRC_MODE
#undef FC_SRC_ADDR_NONE
#undef FC_SRC_16_BITS
#undef FC_SRC_64_BITS

/** */
#undef FC_INTRA_PAN
#undef FC_ACK
#undef FC_SEC
#undef FC_PENDING
#undef FC_FRAME_TYPE_MASK
#undef FC_BEACON_FRAME	
#undef FC_DATA_FRAME		
#undef FC_ACK_FRAME		
#undef FC_COMMAND_FRAME

#define FC_FRAME_TYPE_SHIFT 0
/** MAC Header masks */
#define FC_DST_MODE				0x0C00
#define FC_DST_ADDR_NONE	0x0000
#define FC_DST_16_BITS		0x0800
#define FC_DST_64_BITS		0x0C00

#define FC_SRC_MODE				0xC000   
#define FC_SRC_ADDR_NONE	0x0000
#define FC_SRC_16_BITS		0x8000
#define FC_SRC_64_BITS		0xC000 

/** */
#define FC_INTRA_PAN				0x0040     
#define FC_ACK							0x0020     
#define FC_SEC							0x0008 
#define FC_PENDING					0x0010    
#define FC_FRAME_TYPE_MASK 	0x0007
#define FC_BEACON_FRAME			0x0000     
#define FC_DATA_FRAME				0x0001     
#define FC_ACK_FRAME				0x0002     
#define FC_COMMAND_FRAME		0x0003

#define FC_ACK_PENDING		(FC_ACK_FRAME+FC_PENDING)
#define FC_ACK_NO_PENDING	(FC_ACK_FRAME)

#define MAC_RETRY_MAX 3

void mac_task( void *pvParameters );

#define MAC_CCA_TIME (random_generate(MAC_IFS)/32)+MAC_IFS
#define MAC_ACK_TIME MAC_IFS*2

typedef enum
{
	MAC_INIT = 0,
	MAC_ADHOC,
#ifndef AD_HOC_STATE
	MAC_SCAN,
#endif
#ifdef HAVE_RANGING
	MAC_RANGING,
#endif
	MAC_INVALID
}mac_mode_t;
	
typedef enum
{
	MAC_BEACON =		0,
	MAC_DATA = 			1,
	MAC_ACK = 			2,
	MAC_CMD = 			3,
	MAC_RANGING =		14,
	MAC_TYPE_NONE =	15
}mac_frame_type_t;

typedef enum
{
	MAC_NONE = 0,
	MAC_RECEIVE,
	MAC_TIMER_ACK,
	MAC_TIMER_CCA,
	MAC_TRANSMIT,
	MAC_CONTROL,
	MAC_TIMER_NONE,
	MAC_LOOP
}mac_event_t;	

typedef enum
{
	MAC_TX_OK,
	MAC_TX_OK_ACK,
	MAC_TX_CCA,
	MAC_TX_BUSY
}mac_tx_status_t;
	
mac_mode_t mac_mode;
/*mac_state_t mac_state;*/
uint8_t mac_sequence;
buffer_t *mac_tx_on_air;
uint8_t mac_tx_retry;

/* flags */
#define MAC_CLIENT 0x80
#define MAC_ASSOCIATED 0x40
#define COORD_SHORT 0x20
#define MAC_USE_SHORT_ADDR 0x01

uint8_t mac_flags;

mac_event_t mac_timer_event;

xQueueHandle mac_events;

buffer_t *mac_rx[STACK_BUFFERS_MAX];
uint8_t mac_rx_rd, mac_rx_wr;
buffer_t *mac_tx[STACK_BUFFERS_MAX];
uint8_t mac_tx_rd, mac_tx_wr;

#ifndef AD_HOC_STATE
xList mac_ctl;
#endif

uint8_t mac_pan[2];
uint8_t mac_coord[8];
uint8_t cur_channel = RF_DEFAULT_CHANNEL;
//uint8_t rf_tx_power = RF_DEFAULT_POWER;

void mac_rx_add(buffer_t *buf);
void mac_tx_add(buffer_t *buf);
void mac_tx_push(buffer_t *buf);
buffer_t *mac_tx_pull(void);
buffer_t *mac_rx_pull(void);
void mac_data_up(buffer_t *buf);
mac_tx_status_t mac_buffer_out(buffer_t *buf);
void mac_push(buffer_t *b);

mac_event_t mac_control(buffer_t **ppbuf);
void mac_adhoc(mac_event_t event, buffer_t *buf);

#ifndef AD_HOC_STATE
void mac_scan(mac_event_t curr_event, buffer_t *buf);
void mac_client(mac_event_t curr_event, buffer_t *buf);
#ifdef MAC_COORDINATOR
void mac_coord(mac_event_t curr_event, buffer_t *buf);
#endif

#ifdef MAC_SUPERFRAME
void mac_client_sf(mac_event_t curr_event, buffer_t *buf);
#ifdef MAC_COORDINATOR
void mac_coord_sf(mac_event_t curr_event, buffer_t *buf);
#endif
#endif

#endif

#ifdef MAC_RANGING
void mac_ranging(mac_event_t event, buffer_t *buf);
#endif

uint8_t mac_header_generate(buffer_t *buf);
mac_frame_type_t mac_buffer_parse(buffer_t *buf);

uint8_t *mac_address_push(uint8_t *ptr, sockaddr_t *addr);
uint8_t *mac_address_push_pan(uint8_t *ptr, sockaddr_t *addr);

void mac_timer_launch (mac_event_t event);
void mac_timer_stop(void);

void mac_timer_callback(void);

portCHAR mac_set_channel(uint8_t new_channel)
{
	rf_rx_disable();
	
	if(rf_channel_set(new_channel))
	{
		cur_channel = new_channel;
		return pdTRUE;
	}
	
	rf_rx_enable();
	return pdFALSE;
}

uint8_t mac_current_channel(void)
{
	return cur_channel;
}


buffer_t *mac_rx_pull(void)
{
	buffer_t *b;
	uint8_t tmp = mac_rx_rd;
	
	if (tmp == mac_rx_wr) return 0;
	b = mac_rx[tmp];
	mac_rx[tmp++] = 0;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_rx_rd = tmp;
	
	return b;
}

buffer_t *mac_tx_pull(void)
{
	buffer_t *b;
	uint8_t tmp = mac_tx_rd;
	
	if (tmp == mac_tx_wr) return 0;
	b = mac_tx[tmp];
	mac_tx[tmp++] = 0;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_tx_rd = tmp;
	
	return b;
}

void mac_tx_push(buffer_t *b)
{
	uint8_t tmp = mac_tx_rd;

	if (tmp == 0) tmp = STACK_BUFFERS_MAX - 1;
	else tmp--;
		
	mac_tx[tmp] = b;
	mac_tx_rd = tmp;
}

void mac_tx_add(buffer_t *b)
{
	uint8_t tmp = mac_tx_wr;
	
	mac_tx[tmp++] = b;
	b->to = MODULE_MAC_15_4;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_tx_wr = tmp;
}

void mac_rx_add(buffer_t *b)
{
	uint8_t tmp = mac_rx_wr;
	
	mac_rx[tmp++] = b;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_rx_wr = tmp;
}

/*
	Protocol part for core
	*/

portCHAR mac_init(buffer_t *buf)
{
	buf;
	debug("MAC init.\r\n");
#ifndef AD_HOC_STATE
#ifdef MAC_COORDINATOR
	mac_mode = MAC_INIT;
#else
	mac_mode = MAC_SCAN;	/*clients start in scan*/
#endif
	vListInitialise(&mac_ctl);
#else
	mac_mode = MAC_ADHOC;
#endif
	mac_rx_rd = mac_rx_wr = 0;
	mac_tx_rd = mac_tx_wr = 0;
	mac_pan[0] = 0xFF;
	mac_pan[1] = 0xFF;
	mac_events = xQueueCreate( 16, sizeof( mac_event_t ) );
	mac_timer_event = MAC_TIMER_NONE;
	
	debug("RF init.\r\n");
	rf_init();
	debug("Launch task.\r\n");
	xTaskCreate(mac_task, "MAC", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3 ), (xTaskHandle *) NULL );
	
	return pdTRUE;
}

portCHAR mac_handle( buffer_t *buf )
{
	switch (buf->dir)
	{
			case BUFFER_UP:
				debug("MAC: up\r\n");
				break;
			case BUFFER_DOWN:
				mac_tx_add(buf);
				{
					mac_event_t mac_new_event = MAC_TRANSMIT;
					xQueueSend(mac_events, &mac_new_event, 0);
				}
				return pdTRUE;
				
			default:
				debug("MAC: unknown dir\r\n");
				break; 
	}
	return pdFALSE;
}

portCHAR mac_check( buffer_t *buf )
{
	buf;
	return pdFALSE;
}

#ifdef HAVE_RF_ERROR
extern uint8_t rf_error;
#endif

/*
	Task and functionality part
	*/
/**
 * Mac task.
 *
 *
 * \param pvParameters not used
 */

void mac_task( void *pvParameters )
{
	buffer_t *buf=0;
	mac_event_t curr_event;
	portTickType xLastWakeTime=0;

	pvParameters;
	debug("Mac task start\r\n");
	
	/* Waiting that stack init is ready */
	{
		sockaddr_t temp_mac;
		temp_mac.addr_type = ADDR_NONE;
		while (temp_mac.addr_type == ADDR_NONE)
		{
			mac_get(&temp_mac);
			vTaskDelay(200/portTICK_RATE_MS);
		}
	}
	debug("Mac task go\r\n");
	{
		uint8_t i;
		uint8_t tmp_8;	
		tmp_8= (mac_sequence % 15) + 8;
		for(i=0; i< tmp_8; i++)
		{
			mac_sequence += random_generate(16);
		}
	}
	debug("Set address.\r\n");
	rf_set_address(&mac_long);
#ifdef HAVE_NRP
	debug("Softack on.\r\n");
	rf_address_decoder_mode(RF_SOFTACK_MONITOR);
#else
	debug("Decoder on.\r\n");
	rf_address_decoder_mode(RF_DECODER_ON);
#endif
	rf_rx_enable();
	for (;;)
	{
		if(xQueueReceive(mac_events, &(curr_event),5000 / portTICK_RATE_MS) == pdTRUE)
		{
#ifdef HAVE_RF_ERROR
			if (rf_error)
			{
				debug("RFE");
				debug_hex(rf_error);
				rf_error = 0;
			}
#endif
			rf_rx_callback(0);
			buf = mac_rx_pull();
			if (buf != 0)
			{
				if (curr_event != MAC_RECEIVE) xQueueSend(mac_events, &curr_event, 0);
				curr_event = MAC_RECEIVE;
			}			

			switch(curr_event)
			{
#ifndef AD_HOC_STATE
				case MAC_CONTROL:
					curr_event = mac_control(&buf);
					break;
#endif					
				case MAC_TRANSMIT:
					if (mac_timer_event != MAC_TIMER_NONE)
					{
						curr_event = MAC_LOOP;
					}
					else
					{
						buf = mac_tx_pull();
					}
					break;
					
				case MAC_RECEIVE:
					break;
					
				default:
					break;
			}	/*end event switch*/			
		}/* end event received */
		else curr_event = MAC_LOOP;
		
		while(curr_event != MAC_NONE)
		{
			switch(mac_mode)
			{
				case MAC_ADHOC:
					mac_adhoc(curr_event, buf);
					break;
#ifndef AD_HOC_STATE
				case MAC_SCAN:
					mac_scan(curr_event, buf);
					break;					
				case MAC_CLIENT:
					mac_client(curr_event, buf);
					break;
#ifdef MAC_COORDINATOR
				case MAC_COORD:
					mac_coord(curr_event, buf);
					break;
#endif	/*COORDINATOR*/
					
#ifdef MAC_SUPERFRAME
				case MAC_CLIENT_SF:
					mac_client_sf(curr_event, buf);
					break;
#ifdef MAC_COORDINATOR
				case MAC_COORD_SF:
					mac_coord_sf(curr_event, buf);
					break;
#endif	/*COORDINATOR*/
#endif	/*SUPERFRAME*/
#endif	/*not AD_HOC_STATE*/
					
#ifdef HAVE_RANGING
				case MAC_RANGING:
					mac_ranging(curr_event, buf);
					curr_event = MAC_NONE;
					break;
#endif
				default:
					stack_buffer_free(buf);
					curr_event = MAC_NONE;
			}/* end mode switch */
			buf = 0;
			curr_event = MAC_NONE;
			
			/* look up lists */
			if ((mac_timer_event == MAC_TIMER_NONE) && (mac_tx_on_air == 0))
			{
				buf = mac_tx_pull();
				if (buf != 0)	curr_event = MAC_TRANSMIT;
			}
			else if ((mac_timer_event == MAC_TIMER_NONE) && (mac_tx_on_air != 0))
			{
				curr_event = MAC_TIMER_CCA;
			}
		}	/*end event loop*/
#ifdef HAVE_DEBUG
		if ((RXFIFOCNT & 0xFF) != 0)
		{
			debug("F:");
			debug_int(RXFIFOCNT & 0xFF);
			debug("\r\n");
		}
		if ((DMAARM & 1) == 0)
		{
			debug("D!");
		}
#endif
	} /*end task loop*/
}


void mac_adhoc(mac_event_t curr_event, buffer_t *buf)
{
	mac_frame_type_t ftype;
	
/*	debug("ADHOC:");
	debug_int(curr_event);*/
	switch (curr_event)
	{
			case MAC_RECEIVE:
				ftype = mac_buffer_parse(buf); 
				switch(ftype)
				{
					case MAC_DATA:
						mac_data_up(buf);
						debug("d");
						break;
					case MAC_ACK:
						mac_timer_stop();
						stack_buffer_free(mac_tx_on_air);
						stack_buffer_free(buf);
						mac_tx_retry = 0;
						mac_tx_on_air = 0;
						debug("a");
						break;
					default:
						/*debug_hex(ftype);*/
						debug("d!");
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
				
			case MAC_TIMER_ACK:	
				mac_tx_retry++;
				if (mac_tx_retry > MAC_RETRY_MAX)
				{
					buf = mac_tx_on_air;
					mac_tx_on_air = 0;
					buf->buf_ptr = (buf->buf_end - buf->options.lowpan_compressed );
					ip_broken_link_notify(buf, 0);
					//stack_buffer_free(buf);
					buf = 0;
					mac_tx_retry = 0;
					mac_timer_stop();
					debug("TX fail.\r\n");
					return;			
				}				
				debug("RTR");
				
			case MAC_TIMER_CCA:	/*retransmit*/
				buf = mac_tx_on_air;
				mac_tx_on_air = 0;
			case MAC_TRANSMIT:
				if (buf != 0)
				{
					switch (mac_buffer_out(buf))
					{
						case MAC_TX_OK:
							mac_tx_on_air = 0;
							mac_timer_stop();
							stack_buffer_free(buf);
							mac_tx_retry = 0;
							break;
						case MAC_TX_OK_ACK:
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_ACK);
							break;
						case MAC_TX_BUSY:
							debug("MAC TX busy.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
							break;
						case MAC_TX_CCA:
						default:
							debug("MAC TX CCA.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
					}
				}
				buf = 0;
				break;
			default:
				break;
	}
	if (buf) stack_buffer_free(buf);
}

#ifndef AD_HOC_STATE
mac_mode_t mac_command(buffer_t *buf, uint8_t coordinator)
{
	return mac_mode;
}


void mac_scan(mac_event_t curr_event, buffer_t *buf)
{
	switch (curr_event)
	{
			case MAC_RECEIVE:
				break;
				
			case MAC_TRANSMIT:
				/*TODO: error NOT_ASSOCIATED*/
				mac_tx_add(buf);
				buf = 0;
				break;
			default:
				break;
	}
	if (buf) stack_buffer_free(buf);
}

void mac_client(mac_event_t curr_event, buffer_t *buf)
{
	mac_frame_type_t ftype;
	mac_mode_t new_mode = mac_mode;
	
	switch (curr_event)
	{
			case MAC_RECEIVE:
				ftype = mac_buffer_parse(buf); 
				switch(ftype)
				{
					case MAC_DATA:
						if (buf->src_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
						{	/*if address matches coordinator*/							
							mac_data_up(buf);
						}
						break;
					case MAC_CMD:
						new_mode = mac_command(buf, 0);
						break;
					case MAC_ACK:
						mac_timer_stop();
						stack_buffer_free(mac_tx_on_air);
						stack_buffer_free(buf);
						mac_tx_retry = 0;
						mac_tx_on_air = 0;
						break;
					default:
/*						debug_hex(ftype);*/
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
				
			case MAC_TIMER_ACK:	
				mac_tx_retry++;
				if (mac_tx_retry > MAC_RETRY_MAX)
				{
					buf = mac_tx_on_air;
					mac_tx_on_air = 0;
					stack_buffer_free(buf);
					buf = 0;
					mac_tx_retry = 0;
					mac_timer_stop();
					debug("TX fail.\r\n");
					return;			
				}				
				debug("RTR");
				
			case MAC_TIMER_CCA:	/*retransmit*/
				buf = mac_tx_on_air;
				mac_tx_on_air = 0;
			case MAC_TRANSMIT:
				if (buf != 0)
				{
					switch (mac_buffer_out(buf))
					{
						case MAC_TX_OK:
							mac_tx_on_air = 0;
							mac_timer_stop();
							stack_buffer_free(buf);
							mac_tx_retry = 0;
							break;
						case MAC_TX_OK_ACK:
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_ACK);
							break;
						case MAC_TX_BUSY:
							debug("MAC TX busy.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
							break;
						case MAC_TX_CCA:
						default:
							debug("MAC TX CCA.\r\n");
							mac_tx_on_air = buf;
							mac_timer_launch(MAC_TIMER_CCA);
					}
				}
				buf = 0;
				break;
			default:
				break;
	}
	if (buf) stack_buffer_free(buf);
}

#ifdef MAC_COORDINATOR
void mac_coord(mac_event_t curr_event, buffer_t *buf)
{
	switch (curr_event)
	{
			case MAC_RECEIVE:
				switch(mac_buffer_parse(buf))
				{
					case MAC_DATA:
						mac_data_up(buf);
						break;
					default:
						stack_buffer_free(buf);
				}
				buf = 0;
				break;
			case MAC_TRANSMIT:
				switch (mac_buffer_out(buf))
				{
					case OK:
						stack_buffer_free(buf);
						break;
					case CCA_RESERVED:
					default:
						mac_tx_add(buf);
						mac_timer_launch(MAC_TIMER_CCA);
				}
				buf = 0;
				break;
	}
	if (buf) stack_buffer_free(buf);
}
#endif

#endif /*AD_HOC_STATE*/



extern uint8_t update_neighbour_table(addrtype_t type, address_t address, uint8_t last_lqi, uint8_t last_sqn, uint8_t remove);

/* 
	General header functions 
	*/
mac_frame_type_t mac_buffer_parse(buffer_t *buf)
{
	uint8_t *ptr;
	uint8_t seq, i;
	uint16_t fc,tmp;
	mac_frame_type_t type = MAC_TYPE_NONE;

	if (buf == 0) return type;
		
	ptr = buf->buf;
	ptr += buf->buf_ptr;
	
	fc = *ptr++;
	fc += ((uint16_t) (*ptr++) << 8);
	
	seq = *ptr++;
	
	type = (mac_frame_type_t) ((fc & FC_FRAME_TYPE_MASK) >> FC_FRAME_TYPE_SHIFT); /* detect frametype */
/*		response->ack_req = (fc[0] &  FC_ACK);*/
	
	tmp = (fc & FC_DST_MODE);
	if(tmp == FC_DST_ADDR_NONE)
	{
		buf->dst_sa.addr_type = ADDR_NONE;
	}
	else
	{	/*read PAN*/
		buf->dst_sa.address[8] = *ptr++;
		buf->dst_sa.address[9] = *ptr++;
	}
	if(tmp == FC_DST_64_BITS)
	{
		/* Read dst pan */
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		for (i = 0; i < 8 ; i++)
		{
			buf->dst_sa.address[i] = *ptr++;
		}
	}
	if(tmp == FC_DST_16_BITS)
	{
		/* Read dst pan */
		buf->dst_sa.address[2] = buf->dst_sa.address[8];
		buf->dst_sa.address[3] = buf->dst_sa.address[9];
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		buf->dst_sa.address[0] = *ptr++;
		buf->dst_sa.address[1] = *ptr++;
	}
	buf->dst_sa.port = 0;
	/* Source address */
	tmp=(fc & FC_SRC_MODE);

	if(tmp == FC_SRC_ADDR_NONE)
	{
		buf->src_sa.addr_type = ADDR_NONE;
	}
	else
	{
		if (fc &  FC_INTRA_PAN)
		{
			buf->src_sa.address[8] = buf->dst_sa.address[8];
			buf->src_sa.address[9] = buf->dst_sa.address[9];
		}
		else
		{
			buf->src_sa.address[8] = *ptr++;
			buf->src_sa.address[9] = *ptr++;
		}
	}
	if(tmp == FC_SRC_64_BITS)
	{
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		for (i = 0; i < 8 ; i++)
		{
			buf->src_sa.address[i] = *ptr++;
		}
	}
	if(tmp== FC_SRC_16_BITS)
	{
		buf->src_sa.address[2] = buf->src_sa.address[8];
		buf->src_sa.address[3] = buf->src_sa.address[9];
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		buf->src_sa.address[0] = *ptr++;
		buf->src_sa.address[1] = *ptr++;
	}
	buf->buf_ptr = (uint16_t) ptr;
	buf->buf_ptr -= (uint16_t) buf->buf;
	
	/*check sequence*/
	if ( (buf->src_sa.addr_type != ADDR_NONE) && (type == MAC_DATA) &&
			(update_neighbour_table(buf->src_sa.addr_type, buf->src_sa.address, 
															buf->options.rf_lqi, seq, UPDATE_NEIGHBOUR) == 0) )
	{
		debug("Drop:seq.\r\n");
		type = MAC_TYPE_NONE;
	}
	if ((type == MAC_ACK))
	{
		 if ((mac_tx_on_air == 0) || (seq != (mac_sequence - 1)))
		 {
			 type = MAC_TYPE_NONE;
			 debug("Drop:ack.\r\n");
		 }
	}
	return type;
}

/**
 * Function creates MAC-frame.
 *
 * \param buf indicates pointer for buffer structure.
 *  \return  ack flag which indicated ack-requirement for frame.
 */
uint8_t mac_header_generate(buffer_t *buf)
{
	uint8_t header_size = 3; /* including Frame Control and sqn */
	uint8_t i;
	uint8_t address_length;
	uint8_t *ptr;
	uint16_t fc;
	
	fc = FC_DATA_FRAME;

	if (stack_check_broadcast(buf->dst_sa.address, buf->dst_sa.addr_type) == pdTRUE)
	{
		buf->dst_sa.addr_type = ADDR_BROADCAST;
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		for (i = 0; i < 4; i++)
		{
			buf->dst_sa.address[i]  = 0xff;
		}
	}
	else
	{
		fc |= FC_ACK;
	}
	/* Check Intra flag use */
	if((buf->dst_sa.addr_type == ADDR_COORDINATOR || mac_mode == MAC_CLIENT) 
			&& (mac_flags & MAC_ASSOCIATED))
	{
		buf->dst_sa.address[8] = mac_pan[0];
		buf->dst_sa.address[9] = mac_pan[1];
		
		if((mac_flags & COORD_SHORT))
		{
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
			buf->dst_sa.address[2] = buf->dst_sa.address[8];
			buf->dst_sa.address[3] = buf->dst_sa.address[9];
			buf->dst_sa.address[0] = mac_coord[0];
			buf->dst_sa.address[1] = mac_coord[1];
			
			address_length=2;
			fc |= FC_DST_16_BITS;
			header_size += 2;
		}
		else 
		{
			buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
			address_length=8;
			fc |= FC_DST_64_BITS;
			header_size += 8;
			for(i=0;i<8;i++)
			{
				buf->dst_sa.address[i] = mac_coord[i];
			}
		}
		fc |= FC_INTRA_PAN;
	}
	else
	{
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
		{
			fc |= FC_DST_16_BITS;
			header_size += 4;
			buf->dst_sa.address[8] = buf->dst_sa.address[2];
			buf->dst_sa.address[9] = buf->dst_sa.address[3];
		}
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
		{
			fc |= FC_DST_64_BITS;
			header_size += 10;
		}
		if(memcmp(&buf->dst_sa.address[8], mac_pan, 2 ) == 0 ) /* Compare PAN-IDs --> if match use intra-flag */
		{
			fc |= FC_INTRA_PAN;
			header_size -=2;
		}
	}	

	/* Checkout Source addresstype */
	if(((mac_flags & MAC_ASSOCIATED) || mac_mode==MAC_ADHOC) && (mac_flags & MAC_USE_SHORT_ADDR))
	{
		fc |= FC_SRC_16_BITS;
		header_size +=4;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
	}
	else
	{
		fc |= FC_SRC_64_BITS;
		header_size +=10;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
		memcpy(buf->src_sa.address, mac_long.address, 8);
	}
	
	/* Pack up */
	if(stack_buffer_headroom( buf,(uint16_t)header_size)==pdFALSE)
	{
		stack_buffer_free(buf);
		return pdTRUE;
	}
	
	buf->buf_ptr -= header_size;
	ptr = buf->buf;
	ptr += buf->buf_ptr;
	
	*ptr++ = fc; 	/* FCF*/
	*ptr++ = fc>>8;
	*ptr++ = mac_sequence++;		/*packet sequence number*/
	
	/* Craete address field */
	/* Destination address */
	ptr = mac_address_push_pan(ptr, &(buf->dst_sa));
	ptr = mac_address_push(ptr, &(buf->dst_sa));
	if ((fc & FC_INTRA_PAN) == 0)
	{
		ptr = mac_address_push_pan(ptr, &(buf->src_sa));
	}
	ptr = mac_address_push(ptr, &(buf->src_sa));

	if (fc & FC_ACK) return 1;
	else return 0;
}

uint8_t *mac_address_push_pan(uint8_t *ptr, sockaddr_t *addr)
{
	uint8_t *ptr2 = addr->address;
	
	switch (addr->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
				ptr2 += 6;
		case ADDR_802_15_4_PAN_SHORT:
				ptr2 += 2;
				*ptr++ = *ptr2++;
				*ptr++ = *ptr2++;
				break;
		default:
				*ptr++ = 0xff;
				*ptr++ = 0xff;
	}
	return ptr;
}

uint8_t *mac_address_push(uint8_t *ptr, sockaddr_t *addr)
{
	uint8_t i;
	switch (addr->addr_type)
	{
		case ADDR_802_15_4_PAN_SHORT:
				*ptr++ = addr->address[0];
				*ptr++ = addr->address[1];
				break;
		case ADDR_802_15_4_PAN_LONG:
				for (i=0; i<8; i++)
				{
					*ptr++ = addr->address[i];
				}
				break;
				
#ifndef AD_HOC_STATE
#endif
				
		default:
				*ptr++ = 0xff;
				*ptr++ = 0xff;
	}
	return ptr;
}

void mac_rx_push(void);
void mac_rx_push(void)
{
	mac_event_t mac_new_event = MAC_RECEIVE;
	xQueueSendFromISR(mac_events, &mac_new_event, pdFALSE);			
}

void mac_push(buffer_t *b)
{
	b->to = MODULE_MAC_15_4;
	b->dir = BUFFER_UP;
	b->from = MODULE_NONE;
	
	mac_rx_add(b);
}

void mac_data_up(buffer_t *b)
{
	b->to = MODULE_NONE;
	b->dir = BUFFER_UP;
	b->from = MODULE_MAC_15_4;
	
	stack_buffer_push(b);
}

mac_tx_status_t mac_buffer_out(buffer_t *buf)
{
	uint8_t ack = 0;
	
	if (mac_tx_on_air) return MAC_TX_BUSY;
	
	if (buf->to != MODULE_NONE)
	{
		buf->options.lowpan_compressed = (uint8_t) buffer_data_length(buf);
		ack = mac_header_generate(buf);
		buf->to = MODULE_NONE;
	}
	else
	{
		uint16_t ind = buf->buf_ptr;
		uint16_t fc = buf->buf[ind+1];
		fc <<= 8;
		fc += buf->buf[ind];
		if (fc & FC_ACK) ack = 1;		
	}
	switch (rf_write(buf))
	{
		case pdTRUE:
			mac_tx_on_air = buf;
			if (ack) return MAC_TX_OK_ACK;
			else return MAC_TX_OK;
		case pdTRUE+1:
			return MAC_TX_CCA;
		default:
			return MAC_TX_BUSY;
	}
}

/*
 Timer functions
 */
void mac_timer_launch (mac_event_t event)
{
	timer_mac_stop();
	mac_timer_event = event;
	switch(event)
	{
		case MAC_TIMER_CCA:
			timer_mac_launch(MAC_CCA_TIME);
			break;
			
		case MAC_TIMER_ACK:
			timer_mac_launch(MAC_ACK_TIME);

		default:
			break;	
	}
}

void mac_timer_stop()
{
	timer_mac_stop();
	mac_timer_event = MAC_TIMER_NONE;
}

void mac_timer_callback(void)
{
	portBASE_TYPE prev_task = pdFALSE;
	prev_task = xQueueSendFromISR(mac_events, &mac_timer_event, prev_task);		
	timer_mac_stop();
}

/**
 * Function setup IP modules address mode and also forward MAC and short address information.
 * 
 *
 *  \param  support_short_addr 1=Support & 0=Not support.
 */
void rf_802_15_4_ip_layer_address_mode_set(uint8_t support_short_addr)
{
	if(support_short_addr)
	{
		ip_address_setup( 1, mac_short.address); 
	}
	else
		ip_address_setup( 0,NULL); 
}

