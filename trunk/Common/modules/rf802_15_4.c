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
 * \file     rf802_15_4.c
 * \brief    802.15.4  protocol module.
 *
 *  802.15.4 MAC and CSMA sequence: handler functions.
 *  A more feature-rich implementation of the MAC, using
 *  acknowledgements and beacons (when configured to do so)
 *  Supports only long address format
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#ifndef RF_802_15_4_DEBUG
#undef HAVE_DEBUG
#endif

#include "debug.h"
#include "socket.h"
#include "buffer.h"
#include "bus.h"
#include "module.h"
#include "neighbor_routing_table.h"
#include "control_message.h"
#include "event_timer.h"
#include "rf.h"
#include "mac.h"
#include "cipv6.h"

/*
[NAME]
RF_802_15_4

[ID]
MODULE_RF_802_15_4,

[INFO]
#ifdef HAVE_RF_802_15_4
  {rf_802_15_4_init, rf_802_15_4_handle, rf_802_15_4_check, 0, MODULE_RF_802_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif

[FUNCS]*/
extern portCHAR rf_802_15_4_init(buffer_t *buf);
extern portCHAR rf_802_15_4_handle( buffer_t *buf );
extern portCHAR rf_802_15_4_check( buffer_t *buf );


/* Internal use only!!! */
void check_mac_state(void);

/* Generally used */
uint8_t rf_802_15_4_create_mac_frame(buffer_t *buf);
uint16_t mac_random_generate(void);
csma_response_t rf_802_15_4_csma_ca(buffer_t *buf);
uint8_t mac_header_analyze(buffer_t *buf);
void mac_sqn_update(uint8_t data);
uint8_t mac_dest_check(buffer_t *buf, uint8_t flag);

uint8_t mac_rfd_src_check(buffer_t *buf);

/* Only with MAC FFD device */
void mac_check_pending_status(void);

/* Beacon enable use next*/
void rf_802_15_4_timeslot_update(void *unused_parameter);
void mac_ctrl_message_builder(buffer_t *buf ,mac_control_id_t message_id ,  uint8_t temp, uint8_t value);
void mac_command_frames_handle(buffer_t *buf);
void mac_set_synch_parameters(void);
void mac_build_command_frame_header(buffer_t *buf);
uint8_t * mac_add_address(buffer_t *buf, uint8_t *ind, uint8_t type, uint8_t pan_id);
void mac_init_superframe_options(uint8_t minus);
void mac_parse_beacon(buffer_t *tmp_buffer);

uint8_t push_buf_tx_pool(buffer_t *b, uint8_t make_header);

extern int8_t timer_rf_launch(uint16_t ticks);
extern void rf_rx_callback( void *param );
extern void timer_rf_stop(void);

mac_timer_event_t mac_timer_event;

/* Global */	
rf_802_15_4_pib_t rf_802_15_4_PIB;
xQueueHandle rf_802_15_4_queue;
buffer_t *hp_buffer=0, *hp_buf_secondary=0, *buffer_on_air=0;
#ifndef AD_HOC_STATE
mac_internal_state_t mac_status = MAC_STATE_NORMAL;
control_message_t *scan_res_ptr=0;
uint8_t timeslot_count=16, beacon_track=0, synch_comp=0;
uint16_t synch=0, last_slot_synch_time=0;
portTickType last_slot_start=0;

#ifdef MAC_FFD
mac_pending_buffer_t pending_buffer;

#else
void rf_superfarme_synch_event(void *param);
#endif
#endif

#ifdef SUPERFRAME_MODE
	uint8_t lost_beacon_count=0;
#endif
uint8_t cap_active=0, rf_address_decoder_enable=0;
uint8_t slotted=0;
uint8_t re_tx_after_cca_busy=0;
uint8_t mac_re_tx=0;
sockaddr_t device_own_address;
mac_tx_engine_state_t mac_tx_engine_state;


buffer_t *mac_rx[STACK_BUFFERS_MAX];
uint8_t mac_rx_rd, mac_rx_wr;
buffer_t *mac_tx[STACK_BUFFERS_MAX];
uint8_t mac_tx_rd, mac_tx_wr;

void mac_tx_add(buffer_t *b);
void mac_push(buffer_t *b);
void mac_tx_buf(buffer_t *b);
void check_tx_pool(void);
buffer_t *mac_rx_pull(void);
void mac_rx_add(buffer_t *b);
void scan_operation(void);
void generate_random_time_for_tx(void);
/**
	*	Function to push buffers to MAC, used by the RF driver
	*/
void mac_push(buffer_t *b)
{
	b->to = MODULE_RF_802_15_4;
	b->dir = BUFFER_UP;
	b->from = MODULE_NONE;

	mac_rx_add(b);
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

void mac_rx_add(buffer_t *b)
{
	uint8_t tmp = mac_rx_wr;
	
	mac_rx[tmp++] = b;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_rx_wr = tmp;
}
/*
void mac_tx_push(buffer_t *b)
{
	uint8_t tmp = mac_tx_rd;

	if (tmp == 0) tmp = STACK_BUFFERS_MAX - 1;
	else tmp--;
		
	mac_tx[tmp] = b;
	mac_tx_rd = tmp;
}*/

void mac_tx_add(buffer_t *b)
{
	uint8_t tmp = mac_tx_wr;
	
	mac_tx[tmp++] = b;
	if (tmp >= STACK_BUFFERS_MAX) tmp = 0;
	mac_tx_wr = tmp;
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


typedef enum
{
	MAC_BEACON =		0,
	MAC_DATA = 			1,
	MAC_ACK = 			2,
	MAC_CMD = 			3,
	MAC_RANGING =		14,
	MAC_TYPE_NONE =	15
}mac_frame_type_t;



void vmac_task( void *pvParameters );
extern sockaddr_t mac_long;
extern xQueueHandle     buffers;

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

uint8_t mac_tx_rd=0, mac_tx_wr=0;


/**
 *  Standard nanostack module initalizer.
 *
 *  \return  pdTRUE    OK
 */
portCHAR rf_802_15_4_init( buffer_t *buf )
{
	
#ifndef AD_HOC_STATE
#ifdef MAC_FFD
	uint8_t i=0;
	for(i=0; i<MAX_PENDING_BUFFER_COUNT ;i++)
	{
		pending_buffer.p_data[i].buf=0;
	}
#endif /* MAC_FFD */
#endif /* AD_HOC_STATE */
	mac_rx_rd = mac_rx_wr = 0;
	mac_tx_rd = mac_tx_wr = 0;
	rf_802_15_4_queue = xQueueCreate( 20, sizeof( mac_15_4_event_t ) );
	rf_802_15_4_pib_reset();
	rf_init();
	xTaskCreate( vmac_task, "MAC", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3 ), (xTaskHandle *) NULL );
	return pdTRUE;
}

/**
 *  Standard nanostack buffer handler.
 *
 * Function prefilters buffers and forwards downward buffers to 
 * rf_802_15_4_queue-table.
 *
 * Function has the following states:
 *	- CONNECT, this case only forwards data frames to upper and from upper layers
 *	- DISCONNECT, forward just mac-commands and acks
 *	- BEACON_TRACK, forward only beacon framesopen-zb_ref_guide_v1.1.pdf to rf_802_15_4_queue
 * 	- AD-HOC, forward only data and ack frames
 *	
 * Handle also analyze packets MAC header from Radio. After header analyze process continue based on frame type.
 *
 *  \param   buf      Buffer to process
 *  \return  pdTRUE   Buffer handled by this module
 *  \return  pdFALSE	Buffer rejected by module
 */
portCHAR rf_802_15_4_handle( buffer_t *buf )
{
	mac_15_4_event_t mac_event;

	if(buf->options.type == BUFFER_CONTROL && buf)		/* Control message received */
	{	/* Control-message */
		mac_event.id = MAC_15_4_CONTROL;
		mac_event.buf = buf;
		if (xQueueSend(rf_802_15_4_queue, &mac_event,10) == pdFALSE)
		{
			stack_buffer_free(mac_event.buf);
			debug("MAC queue full\r\n");
		}
		buf=0;
		return pdTRUE;
	}

	if(buf && (buf->dir == BUFFER_DOWN))/* From upperlayer */
	{
		mac_event.id = MAC_TX_MES;
		mac_event.buf = buf;
		switch(rf_802_15_4_PIB.mode)
		{
			case AD_HOC_MODE:
				if (xQueueSend(rf_802_15_4_queue, &mac_event,10) == pdFALSE)
				{
					
					stack_buffer_free(buf);
					debug("TX pool full\r\n");
				}
				buf=0;
				break;

			default:
#ifndef AD_HOC_STATE
				/* Forward only in CONNECT-STATE */
				
				if(rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD)
				{
					if (xQueueSend(rf_802_15_4_queue, &mac_event,10) == pdFALSE)
					{
						stack_buffer_free(mac_event.buf);
						debug("TX pool full\r\n");
					}
					buf=0;
				}
				if(buf)
				{
#ifdef MAC_RFD
					if(rf_802_15_4_PIB.pan_id[0] != 0xff && rf_802_15_4_PIB.pan_id[1] != 0xff)
					{
						/* Information for Application */
						control_message_t *ptr;
						ptr = ( control_message_t*) buf->buf;
						ptr->message.ip_control.message_id = NOT_ASSOCIATED;
						push_to_app(buf);
						buf=0;
					}
#endif /* MAC_RFD */
				}
#endif /* AD_HOC_STATE */
				if(buf)
				{
					stack_buffer_free(buf);
					buf=0;
				}
				break;
		}
		return pdTRUE;
	}
	if(buf &&  (buf->dir == BUFFER_UP))		/* Detect if buffer is a control message  */
	{
		stack_buffer_free(buf);
		buf=0;
	}
	return pdTRUE; 
}

portCHAR rf_802_15_4_check( buffer_t *buf )
{
  
#ifdef HAVE_LOOP
	if (buf->buf_ptr == 0)
		return pdTRUE;
#endif
	return pdFALSE;  
}

#ifndef AD_HOC_STATE
/**
 *  Parse beacon frame and update result active scan.
 *
 * Function use scan_res_ptr pointer which indicates buffer for result of active scan process. MAC task initialize global pointer to indicated to this buffer.
 *
 *  \param   buf      Buffer to process
 */
void mac_parse_beacon(buffer_t *tmp_buffer)
{ 
	uint8_t *ind, s, tmp_8=8, j;
	ind = (tmp_buffer->buf + tmp_buffer->buf_ptr);
	s = scan_res_ptr->message.mac_control.message.scan_confirm.resultlist_size;
	for(j=0; j<2; j++)
	{
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].superframe_spec[j] =  *ind++;
	}

	if(tmp_buffer->src_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
	{
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].coord_addr_mode = ADDR_MODE_16;
		tmp_8=2;
		
	}
	else
	{
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].coord_addr_mode = ADDR_MODE_64;
		tmp_8=8;
	}

	for(j=0; j<2; j++)
	{
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].panid[j] = tmp_buffer->src_sa.address[tmp_8 + j];
	}

	for(j=0; j < tmp_8; j++)
	{
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].address[j] = tmp_buffer->src_sa.address[j];
	}
	
	if(scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].superframe_spec[1] & 0x80)
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].assoc_permit =TRUE;
	else
		scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].assoc_permit = FALSE;

	scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].link_quality = tmp_buffer->options.rf_lqi;
	scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].gts_permit = *ind++;
	scan_res_ptr->message.mac_control.message.scan_confirm.pan_descriptor[s].logical_channel = rf_802_15_4_PIB.logical_channel;
	s++;
	scan_res_ptr->message.mac_control.message.scan_confirm.resultlist_size = s;
} 
#endif

void generate_random_time_for_tx(void)
{
	uint16_t rand;
	rand = mac_random_generate();
	mac_timer_event = WAITING_EVENT_LAUNCH_TX;
	timer_rf_launch(rand);
	
}

void check_tx_pool(void)
{
	if((buffer_on_air==0 && mac_timer_event == TIMER_IDLE) && ((mac_tx_rd != mac_tx_wr) || hp_buffer))
	{
		generate_random_time_for_tx();
	}
}

void mac_tx_buf(buffer_t *b)
{
	portCHAR response;
	response = rf_write(b);
	if( response == pdTRUE )
	{
		mac_tx_engine_state = TX_IDLE;
#ifndef AD_HOC_STATE
		if(mac_status == MAC_ACTIVE_SCAN ||  mac_status == MAC_ORPHAN_SCAN)
		{
			stack_buffer_free(b);
			if(buffer_on_air)
			{
				buffer_on_air=0;
			}
			mac_timer_event = MAC_STATUS_CHECK;
			timer_rf_launch(100000/PLATFORM_TIMER_DIV);
		}
		else
		{
#endif
			if(b->options.handle_type == HANDLE_ACK_REQ || b->options.handle_type == HANDLE_ASSOC_REQ || b->options.handle_type == HANDLE_DATA_REQ)
			{
				mac_timer_event = WAITING_ACK;
				buffer_on_air = b;
				timer_rf_launch(8000/PLATFORM_TIMER_DIV);
				return;
			}
			else
			{
				stack_buffer_free(b);
				if(buffer_on_air)
				{
					buffer_on_air=0;
				}
				check_tx_pool();
			}
#ifndef AD_HOC_STATE
		}
#endif
		return;
	}
	else if(response == pdFALSE)
	{
	
		if(re_tx_after_cca_busy > 4)
		{
			/* Information for Application */
			control_message_t *ptr;
			ptr = ( control_message_t*) b->buf;
			ptr->message.ip_control.message_id = MAC_CCA_ERR;
			push_to_app(b);
			b=0;
			if(buffer_on_air)
				buffer_on_air=0;
			mac_tx_engine_state = TX_IDLE;
			debug("TX:Chan busy\r\n");
			return;
		}
		else
		{
			re_tx_after_cca_busy++;
			mac_tx_engine_state = TX_RE_CCA_CHECK;
			buffer_on_air = b;
			generate_random_time_for_tx();
			return;
		}
	}
	else
	{
		re_tx_after_cca_busy++;
		mac_tx_engine_state = TX_RE_CCA_CHECK;
		buffer_on_air = b;
		generate_random_time_for_tx();
		debug("Radio Bus err.\r\n");
		//while(1){}
	}
}

/**
 * Mac task.
 *
 * Reads message queues from rf_802_15_4_queue and detects buffer type.
 * Types: HANDLE_RX, HANDLE_TX and HANDLE_CONTROL.
 * HANDLE_RX, detect frametype and forward data to upper.
 * HANDLE_TX, build data-frame based on source and destination address.
 * HANDLE_CONTROL, from NWK_CONFIG-module task handle: send response back.
 *
 * \param pvParameters not used
 */

#ifndef AD_HOC_STATE
//mac_internal_state_t mac_status = MAC_STATE_NORMAL;
uint16_t channel_list=0;
uint8_t scan_index=0;
portTickType mac_status_timer=0;
buffer_t *mac_result=0;
#endif
//uint8_t task_status=0;
void vmac_task( void *pvParameters )
{
	uint8_t i, tmp_8=0, permission_tx=0;
	buffer_t *buf=0;
	control_message_t *ptr;
	portTickType xLastWakeTime=0;
	mac_15_4_event_t mac_event;
#ifndef AD_HOC_STATE
	uint16_t tmp_16;
	uint8_t s=0;
#endif
	debug("\r\nMac-task start");
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
	vTaskDelay(200/portTICK_RATE_MS);
	rf_802_15_4_PIB.data_sqn = rf_802_15_4_PIB.rf_802_15_4_ext_addr[0];
	tmp_8= (rf_802_15_4_PIB.data_sqn % 15);
	for(i=0; i< tmp_8; i++)
	{
		mac_random_generate();
	}
	tmp_8=0;
	
#ifdef AD_HOC_STATE
#ifdef RF_ADDRESS_DECODER
	mac_handle_address_decoder(RF_DECODER_ON);	
#endif
#endif
	rf_rx_enable();
	xLastWakeTime = xTaskGetTickCount();
	mac_timer_event = TIMER_IDLE;
	mac_tx_engine_state = TX_IDLE;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		/*rx_mes=pdFALSE;
		if(task_status)
		{
			rx_mes= xQueueReceive(rf_802_15_4_queue, &(mac_event),10 / portTICK_RATE_MS);	
		}
		else
		{
			rx_mes= xQueueReceive(rf_802_15_4_queue, &(mac_event),1000 / portTICK_RATE_MS);	
		}*/
		if (xQueueReceive(rf_802_15_4_queue, &(mac_event),1000 / portTICK_RATE_MS) == pdTRUE) /* Waiting for message ques */
		{
			
			
			/*if (buf != 0)
			{
				if (mac_event.id != MAC_RX_MES) xQueueSend(rf_802_15_4_queue, &mac_event, 0);
					mac_event.id = MAC_RX_MES;
			}*/

			switch (mac_event.id)
			{
				case MAC_RX_MES:
					rf_rx_callback(0);
					buf = mac_rx_pull();
					if(buf)
					{
						/* Parse MAC header */
						tmp_8 = mac_header_analyze(buf);
						switch(tmp_8)
						{
							case FC_DATA_FRAME:
								debug("Data\r\n");
								buf->from = MODULE_RF_802_15_4;
								buf->to = MODULE_NONE;
								buf->dir = BUFFER_UP;
								stack_buffer_push(buf);
								buf=0;
								break;

							case FC_ACK_FRAME:
								
								timer_rf_stop();
								
								mac_tx_engine_state = TX_IDLE;
								mac_timer_event = TIMER_IDLE;
#ifndef AD_HOC_STATE
								if(buffer_on_air->options.handle_type == HANDLE_ASSOC_REQ)
								{
									mac_status = MAC_WAITING_DATA_REQ_TIMEOUT;
									mac_timer_event = MAC_STATUS_CHECK;
									timer_rf_launch(15000/PLATFORM_TIMER_DIV);
									//mac_status_timer = xTaskGetTickCount();
									//task_status=1;
									
								}
								else if(buffer_on_air->options.handle_type == HANDLE_DATA_REQ)
								{
									mac_status = MAC_WAITING_DATA_FROM_SERVER;
									mac_timer_event = MAC_STATUS_CHECK;
									//mac_status_timer = xTaskGetTickCount();
									//task_status=1;
									timer_rf_launch(64000/PLATFORM_TIMER_DIV);
								}
								else
								{
									
								}
								
#else
								check_tx_pool();
#endif
								debug("ack\r\n");
								stack_buffer_free(buffer_on_air);
								stack_buffer_free(buf);
								mac_re_tx = 0;
								re_tx_after_cca_busy=0;
								buffer_on_air = 0;
								buf=0;
								check_tx_pool();
								break;
#ifndef AD_HOC_STATE
							case FC_COMMAND_FRAME:
								mac_command_frames_handle(buf);
								break;

							case FC_BEACON_FRAME:
								if(rf_802_15_4_PIB.state == BEACON_TRACK && buf)
								{
									if(beacon_track==1 || synch_comp==1)
									{
#ifdef SUPERFRAME_MODE
										if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG) tmp_8=8;
										else tmp_8=2;
										if(memcmp(&(buf->src_sa.address[tmp_8]),rf_802_15_4_PIB.pan_id,2 ) == 0)
										{
											tmp=0;
											if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG)
											{
												if(memcmp(buf->src_sa.address,rf_802_15_4_PIB.coord_ext_addr,8 ) == 0) tmp=1;
											}
											else
											{
												if(memcmp(buf->src_sa.address,rf_802_15_4_PIB.coord_short_Addr,2 ) == 0) tmp=1;
											}
											if(tmp)
											{
												if(beacon_track==1)
												{
													check_tx_pool();
												}
												else
												{
													timer_rf_stop();
													debug("synch\r\n");
													synch_comp=0;
													mac_status = MAC_STATE_NORMAL;
													rf_802_15_4_PIB.state = CONNECT;
													ptr = ( control_message_t*) buf->buf;
													ptr->message.ip_control.message_id = ASSOCATION_WITH_COORDINATOR;
													push_to_app(buf);
												}
												/* Initialize general parameter for superframe mode */
												lost_beacon_count = 0;
												mac_init_superframe_options(0);
												debug("BT..OK\r\n");
											}
										}
#endif	/* SUPERFRAME_MODE */
									}
									else
									{
										mac_parse_beacon(buf);
										stack_buffer_free(buf);
										buf=0;
									}
								}
								if(buf)
								{
									stack_buffer_free(buf);
									buf=0;
								}
								break;
#endif
							default:
								stack_buffer_free(buf);
								buf=0;
								check_tx_pool();
								break;
						}
						
					}
					break;


				case MAC_TX_MES:
					mac_event.buf->to = MODULE_ICMP;
					mac_tx_add(mac_event.buf);

					if(mac_timer_event==TIMER_IDLE)
					{
						generate_random_time_for_tx();
					}
					break;

				case MAC_15_4_CONTROL:
					buf = mac_event.buf;
					mac_event.buf=0;
					ptr = ( control_message_t*) buf->buf;
					switch(ptr->message.mac_control.message_id)
					{
#ifndef AD_HOC_STATE
#ifdef MAC_RFD	
						case ASSOC_REQ:		/* Assocation process start */
						{
							//uint8_t tmp=0;
							debug("\r\n Assoc req ");
							rf_802_15_4_PIB.logical_channel = ptr->message.mac_control.message.assoc_req.logical_channel;
							rf_channel_set(rf_802_15_4_PIB.logical_channel);
							tmp_8 = ptr->message.mac_control.message.assoc_req.cap_info;
							
							memcpy(rf_802_15_4_PIB.pan_id, ptr->message.mac_control.message.assoc_req.cord_panid, 2);
							if(ptr->message.mac_control.message.assoc_req.cord_addrmode == ADDR_MODE_16)
							{
								rf_802_15_4_PIB.coord_addr_mode = ADDR_802_15_4_PAN_SHORT;
								memcpy(rf_802_15_4_PIB.coord_short_Addr, ptr->message.mac_control.message.assoc_req.coord_address, 2);
							}
							else
							{
								rf_802_15_4_PIB.coord_addr_mode = ADDR_802_15_4_PAN_LONG;
								memcpy(rf_802_15_4_PIB.coord_ext_addr, ptr->message.mac_control.message.assoc_req.coord_address, 8);
							}
#ifdef RF_AUTO_ACK
#ifdef RF_ADDRESS_DECODER											
							mac_handle_address_decoder(RF_DECODER_ON);
#endif
#endif
							buf->options.type = BUFFER_DATA;
							buf->buf_ptr=0;
							buf->buf_end=2;
							buf->buf[0] = CMD_ASSOC_REQ;
							buf->buf[1] = tmp_8;
							mac_build_command_frame_header(buf); /* Assocation request */
							buf->options.handle_type = HANDLE_ASSOC_REQ;
							mac_tx_add(buf);
							if(mac_timer_event==TIMER_IDLE)
							{
								generate_random_time_for_tx();
							}
						}
							break;
#endif /* MAC_RFD */

#ifdef MAC_FFD
						case START_REQ:		/* Start PAN-network */
							if(ptr->message.mac_control.message.start_req.cord_realigment == FALSE)
							{
								debug("Start PAN req\r\n");
								if(ptr->message.mac_control.message.start_req.pan_cordinator == TRUE)
								{
									memcpy(rf_802_15_4_PIB.pan_id, ptr->message.mac_control.message.start_req.panid, 2);	
									rf_802_15_4_PIB.mac_options |= MAC_ASSOC_CORD;
									rf_802_15_4_PIB.mode = BEACON_ENABLE_COORD_MODE;
								}
#ifdef RF_ADDRESS_DECODER
								mac_handle_address_decoder(RF_DECODER_COORDINATOR);
#endif
								rf_802_15_4_ip_layer_address_mode_set(1);
								
								if(ptr->message.mac_control.message.start_req.beacon_order != 15)
								{
#ifdef SUPERFRAME_MODE
									/* Superframemode comes there */
									rf_802_15_4_PIB.beacon_order = ptr->message.mac_control.message.start_req.beacon_order;
									rf_802_15_4_PIB.superframe_order = ptr->message.mac_control.message.start_req.superframe_order;
									timeslot_count=10;
									mac_set_synch_parameters(); /* Set defined by Beacon order parameter */
#ifdef HAVE_PERIOD_TIMER
									ptTimerStart(synch, &rf_802_15_4_timeslot_update);
#else
#warning "Not defined HAVE_PERIOD_TIMER variable"
#endif
#else
									debug("Superframe mode not supported\r\n");
#endif	
								}

								rf_802_15_4_PIB.mac_options |= MAC_ASSOCPERMIT;
								if(ptr->message.mac_control.message.start_req.batt_life_ext)
									rf_802_15_4_PIB.mac_options |= MAC_BATT_LIFE_EXT;

								rf_802_15_4_PIB.logical_channel = ptr->message.mac_control.message.start_req.logical_channel;
								rf_channel_set(rf_802_15_4_PIB.logical_channel);

								rf_802_15_4_PIB.state = CONNECT;
								buf->buf_end = 0;
								buf->buf_ptr = 0;
								ptr = ( control_message_t*) buf->buf;
								ptr->message.ip_control.message_id = COORDINATOR_STARTED;
								push_to_app(buf);
								buf=0;
								mac_event.buf = 0;
							}
							else
							{
									stack_buffer_free(buf);
									buf = 0;
							}
						break;

					case ROUTER_START:
							memcpy(rf_802_15_4_PIB.pan_id, ptr->message.mac_control.message.router_start.panid, 2);
							rf_802_15_4_PIB.mac_options |= MAC_ASSOC_CORD;
							rf_802_15_4_PIB.mode = ROUTER_MODE;
#ifdef RF_ADDRESS_DECODER
							mac_handle_address_decoder(RF_DECODER_ON);
#endif
							rf_802_15_4_ip_layer_address_mode_set(1);

							rf_802_15_4_PIB.logical_channel = ptr->message.mac_control.message.router_start.logical_channel;
							rf_channel_set(rf_802_15_4_PIB.logical_channel);
							rf_802_15_4_PIB.state = CONNECT;
							buf->buf_end = 0;
							buf->buf_ptr = 0;
							ptr = ( control_message_t*) buf->buf;
							ptr->message.ip_control.message_id = GATEWAY_STARTED;
							push_to_app(buf);
							buf = 0;
							break;

					case ORPHAN_RESPONSE:		/* Orphan response send for orphaned child device */
							/*add check for gateway mode?*/
							if(rf_802_15_4_PIB.mode == BEACON_ENABLE_COORD_MODE)
							{
								uint8_t *dptr;
								buf->src_sa.address[0] = ptr->message.mac_control.message.orphan_response.short_address[0];
								buf->src_sa.address[1] = ptr->message.mac_control.message.orphan_response.short_address[1];
								memcpy(buf->dst_sa.address, ptr->message.mac_control.message.orphan_response.orphan_address, 8);
								buf->dst_sa.address[8] = 0xff;
								buf->dst_sa.address[9] = 0xff;

								buf->options.type = BUFFER_DATA;
								buf->buf_end = 0;
								buf->buf_ptr = 0;
								dptr = (buf->buf + buf->buf_ptr);
								*dptr++ = CMD_CORDINATOR_REALIGN;
								for(i=0; i < 2 ; i++)
								{
									*dptr++ = rf_802_15_4_PIB.pan_id[i];
								}
								for(i=0; i < 2 ; i++)
								{
									*dptr++ = rf_802_15_4_PIB.rf_802_15_4_short_addr[i];
								}
								*dptr++ = rf_802_15_4_PIB.logical_channel;
								*dptr++ = buf->src_sa.address[0];
								*dptr++ = buf->src_sa.address[1];
								buf->buf_end = (dptr - buf->buf);
								mac_build_command_frame_header(buf);
								buf->options.handle_type = HANDLE_ACK_REQ;
								mac_tx_add(buf);
								if(mac_timer_event==TIMER_IDLE)
								{
									generate_random_time_for_tx();
								}
								buf = 0;
							}
							else
							{
								stack_buffer_free(buf);
								buf = 0;
							}
							break;
#endif /* MAC_FFD */
						case SCAN_REQ:		/* Start Scan process: */
							/*add check for gateway mode?*/							
							debug(" Scan req:");
							channel_list = ptr->message.mac_control.message.scan_req.scan_channels;
							if(ptr->message.mac_control.message.scan_req.scan_type == ACTIVE_SCAN || ptr->message.mac_control.message.scan_req.scan_type == ORPHAN_SCAN)
							{ 
								mac_result=0;
								while(mac_result == 0)
								{
									mac_result = stack_buffer_get(50);			/* For scan result */
									if(mac_result==0)
										vTaskDelay(20);
								}
								mac_result->from = buf->to;
								mac_result->to = buf->from;
								
								scan_res_ptr = ( control_message_t*) mac_result->buf;
								scan_res_ptr->message.mac_control.message_id = SCAN_CONFIRM;
								scan_res_ptr->message.mac_control.message.scan_confirm.scan_type = ptr->message.mac_control.message.scan_req.scan_type;
								scan_res_ptr->message.mac_control.message.scan_confirm.resultlist_size = 0;

								if(ptr->message.mac_control.message.scan_req.scan_type == ACTIVE_SCAN)
								{
									debug("Active \r\n");
									rf_802_15_4_PIB.state = BEACON_TRACK;
									mac_status = MAC_ACTIVE_SCAN;
								}
#ifdef MAC_RFD
								else
								{
									debug(" Orphan \r\n");	
									rf_802_15_4_PIB.state = DISSCONNECT;
									slotted=0;
									timeslot_count=16;
									mac_status = MAC_ORPHAN_SCAN;
								}
#endif
								stack_buffer_free(buf);
								mac_event.buf=0;
								buf=0;
								scan_index=0;	
								/* Start scan */
								//task_status=1;
								scan_operation();	
							}

							if((ptr->message.mac_control.message.scan_req.scan_type == ED_SCAN && buf)) /* ED scan modeR */
							{
								control_message_t *ctrl_ptr=0;
								debug(" ED\r\n");
								buf->buf_end = 0;
								buf->buf_ptr = 0;
								buf->to = MODULE_NWK_MANAGER;
								buf->from = MODULE_RF_802_15_4;
								ctrl_ptr = ( control_message_t*) buf->buf;
								rf_802_15_4_PIB.state = BEACON_TRACK;
								ctrl_ptr->message.mac_control.message_id = SCAN_CONFIRM;
								ctrl_ptr->message.mac_control.message.scan_confirm.scan_type = ED_SCAN;
								ctrl_ptr->message.mac_control.message.scan_confirm.resultlist_size = 0;
								s = ctrl_ptr->message.mac_control.message.scan_confirm.resultlist_size;
								tmp_16=0;
								for(i=0; i < 16; i++)
								{
									tmp_16 = (1 << i);
									if(channel_list & tmp_16)
									{
										if(rf_channel_set(11+i))
										{
											rf_802_15_4_PIB.logical_channel = 11+i;
											pause_us(50);
											ctrl_ptr->message.mac_control.message.scan_confirm.ed_list[s] = rf_analyze_rssi();
											s++;
										}
									}
								}
								ctrl_ptr->message.mac_control.message.scan_confirm.resultlist_size = s;
								rf_802_15_4_PIB.state = DISSCONNECT;
								buf->options.type = BUFFER_CONTROL;
								stack_buffer_push(buf);
								buf=0;
							}

							break;
#ifdef MAC_RFD
#ifdef SUPERFRAME_MODE						
						case SYNCH_REQ:		/* Start synchronize process at superframe mode */
							synch = ptr->message.mac_control.message.synch_req.synch_timer; 			/* Set timeslot timer period length*/
							rf_802_15_4_PIB.beacon_order = ptr->message.mac_control.message.synch_req.b_order;	/* Store superframe setup to MAC PIB */
							rf_802_15_4_PIB.superframe_order = ptr->message.mac_control.message.synch_req.s_order;
							last_slot_synch_time = ptr->message.mac_control.message.synch_req.last_slot_synch; /* Set time limit for last timeslot send */
							stack_buffer_free(buf);
							buf = 0;
							if(synch)
							{
								uint16_t count;
								rf_802_15_4_PIB.state = BEACON_TRACK;	/* Set MAC state to Beacon Tracking */
								slotted=1;			/* Activate slotted-CSAMA-CA */
								synch_comp=1;
								mac_status = MAC_SYNCHRONIZE_WITH_SERVER;
								/* Waiting Beacon */
								mac_status_timer = xTaskGetTickCount();
							}
						break;
#endif
#endif
#endif
						default:
							stack_buffer_free(mac_event.buf);
							break;
					}
					break;

				case MAC_TIMER_INT_CB:

					switch(mac_timer_event)
					{
#ifndef AD_HOC_STATE
						case MAC_STATUS_CHECK:

						check_mac_state();
						break;
#endif
						case WAITING_ACK:

						case WAITING_ACK_PENDING:
							mac_re_tx++;
							debug_int(mac_re_tx);
							debug(" re_tx\r\n");
							mac_timer_event = TIMER_IDLE;

							if(buffer_on_air)
							{
							if(mac_re_tx > MAX_RE_TX)
							{
								mac_tx_engine_state = TX_IDLE;
								mac_re_tx=0;
								check_tx_pool();
	#ifdef AD_HOC_STATE
								buffer_on_air->buf_ptr = (buffer_on_air->buf_end - buffer_on_air->options.lowpan_compressed );
								ip_broken_link_notify(buffer_on_air, 0);
	#else
								if(rf_802_15_4_PIB.device_type==MAC_RFD_TYPE)
								{
	#ifdef MAC_RFD
									/* Information for ntework manger that lost connect to coordinator */
	#ifdef HAVE_PERIOD_TIMER
									ptTimerStop();
	#endif /* HAVE_PERIOD_TIMER */
									rf_802_15_4_pib_reset();
									mac_handle_address_decoder(RF_DECODER_NONE);
									rf_802_15_4_ip_layer_address_mode_set(0);
									rf_802_15_4_PIB.mode =  BEACON_ENABLE_MODE;
									rf_802_15_4_PIB.state = DISSCONNECT;
									ptr = ( control_message_t*) buffer_on_air->buf;
									ptr->message.ip_control.message_id = ASSOCIATION_LOST;
									push_to_app(buffer_on_air);
	#endif /* MAC_RFD */
								}
								else
								{
									/* Self healing command */
									if(buffer_on_air->options.lowpan_compressed)
									{
										buffer_on_air->buf_ptr = (buffer_on_air->buf_end - buffer_on_air->options.lowpan_compressed );
										ip_broken_link_notify(buffer_on_air, 0);
									}
									else
									{
										stack_buffer_free(buffer_on_air);
									}
								}
	#endif /* AD_HOC_STATE */
								buffer_on_air=0;
								debug("No ack\r\n");
							}
							else
							{
								mac_tx_engine_state = TX_RE_TX;
								generate_random_time_for_tx();
							}

							}
							else
							{
								debug("rx ack earlier?\r\n");
							}
							break;
						case WAITING_EVENT_LAUNCH_TX:
#ifndef AD_HOC_STATE
							if(slotted)
							{
								if(timeslot_count == 1)
								{
									if ((xTaskGetTickCount() - last_slot_start)*portTICK_RATE_MS < last_slot_synch_time) permission_tx=1;
									else permission_tx=0;
								}
							}
							else
								permission_tx=1;
	#else
							permission_tx=1;
	
	#endif
							mac_timer_event = TIMER_IDLE;
							break;
							default:
							mac_timer_event = TIMER_IDLE;
							break;

					}
					break;
#ifndef AD_HOC_STATE
#ifdef MAC_FFD
				case MAC_PENDING:
					buf = mac_event.buf;
					mac_event.buf=0;
					for(i=0; i< MAX_PENDING_BUFFER_COUNT; i++)
					{
						if(pending_buffer.p_data[i].buf == 0)
						{ 
							pending_buffer.p_data[i].buf = buf;
							pending_buffer.p_data[i].ttl = rf_802_15_4_PIB.transaction_persistence_time;
							pending_buffer.buffer_count++;
							rf_send_ack(1);
							i=MAX_PENDING_BUFFER_COUNT;
							buf=0;
						}
					}
					if(buf)
					{
						stack_buffer_free(buf);
						buf=0;
					}
					break;
#endif
#endif
				default:
					debug("Not supp\r\n");
					stack_buffer_free(mac_event.buf);
					break;
			}
		}
		//Check TX engine state and permission
		if(permission_tx)
		{
			buffer_t *b;

			switch (mac_tx_engine_state)
			{
				case TX_IDLE:
						if(hp_buffer)
						{
							b=hp_buffer;
							hp_buffer=0;
							if(hp_buf_secondary)
							{
								hp_buffer=hp_buf_secondary;
								hp_buf_secondary=0;
							}
						}
						else
						{
							b=mac_tx_pull();
						}
						//init re tx variables
						re_tx_after_cca_busy=0;
						mac_re_tx=0;
						if(b)
						{
							if(b->to == MODULE_ICMP)
							{
								b->options.lowpan_compressed = (uint8_t) buffer_data_length(b);
								if(rf_802_15_4_create_mac_frame(b))
								{
									b->options.handle_type = HANDLE_ACK_REQ;
								}
							}
							else
							{
								b->options.lowpan_compressed=0;
							}
							mac_tx_buf(b);
						}
						else
						{
							debug("Tx-idle:buf err\r\n");
							while(1){}
						}
		
				break;
				
				case TX_RE_TX:
					re_tx_after_cca_busy=0;
					if(buffer_on_air==0)
					{
						debug("TX_RE_TX: buf ERR\r\n");
					}
					b=buffer_on_air;
					mac_tx_buf(b);
				
				break;
	
				case TX_RE_CCA_CHECK:
					if(buffer_on_air==0)
					{
						debug("TX_RE_CCA: buf ERR\r\n");
						while(1){}
					}
					b=buffer_on_air;
					mac_tx_buf(b);
				break;
	
				default:
					debug("Sis. ongelma\r\n");
					while(1){}
				break;
			}
			permission_tx=0;
		}
#ifndef AD_HOC_STATE
#ifdef SUPERFRAME_MODE
		if(mac_status == MAC_SYNCHRONIZE_WITH_SERVER && (xTaskGetTickCount() - mac_status_timer) *portTICK_RATE_MS > 65000)
		{
			mac_status = MAC_STATE_NORMAL;
			if(synch_comp)
			{
				buffer_t *tmp_buffer=0;
				rf_802_15_4_PIB.state = DISSCONNECT;
				synch_comp=0;
				debug("synch..not ok\r\n");
				tmp_buffer = stack_buffer_get(50);
				if(tmp_buffer)
				{
					tmp_buffer->buf_end = 0;
					tmp_buffer->buf_ptr = 0;
					ptr = ( control_message_t*) tmp_buffer->buf;
					ptr->message.ip_control.message_id = ASSOCATION_LOSTED;
					push_to_app(tmp_buffer);
					tmp_buffer=0;
				}
				tmp_buffer = stack_buffer_get(50);
				if(tmp_buffer)
				{
					/* Create Synch loss indication control-message */
					mac_ctrl_message_builder(tmp_buffer ,SYNCH_LOSS_IND , NO_ACK_FROM_COORD, 0);
					tmp_buffer=0;
				}
			}
		}
#endif
#endif

		if ((xTaskGetTickCount() - xLastWakeTime)*portTICK_RATE_MS > 15000)
		{

#ifndef AD_HOC_STATE
#ifdef MAC_FFD
			mac_check_pending_status();
#endif
#endif
			//check_tables_status(0);
			xLastWakeTime = xTaskGetTickCount();
		}










		
#ifndef AD_HOC_STATE
#ifdef SUPERFRAME_MODE
		if(beacon_track)
		{
			tmp_16=6;
			while(tmp_16 != 0)
			{
				vTaskDelay(6 / portTICK_RATE_MS);
				if(beacon_track==2)
				{
					tmp_16=0;
				}
				if(tmp_16)
				{
					lost_beacon_count++;
					if(lost_beacon_count > aMaxLostBeacons || lost_beacon_count == SYNCH_LOST_TOTAL)
					{
						if(lost_beacon_count == SYNCH_LOST_TOTAL)
						{
							rf_802_15_4_PIB.state = DISSCONNECT;
							debug("no Beacon RX\r\n");
								
							tmp_buffer = stack_buffer_get(150);
							if(tmp_buffer)
							{
								/* Create Synch loss indication control-message */
								mac_ctrl_message_builder(tmp_buffer ,SYNCH_LOSS_IND , BEACON_LOST, 0);
								tmp_buffer=0;
								tmp_16=0;
							}
						}
						else
						{
							cap_active=0;
							debug("Need Re-SYNCH...");
							tmp_16 = ((synch*32)/10);
						}
					}
					else
					{
						beacon_track=0;
						mac_init_superframe_options(5);
						tmp_16=0;
					}
				}
			}
		}
#endif
#endif
	}/* end of for-loop */
}/* end of mac_task */


#ifndef AD_HOC_STATE

void check_mac_state(void)
{
	buffer_t *buf;
	control_message_t *ptr;
	debug("mac_state \r\n");
	mac_timer_event = TIMER_IDLE;
	switch (mac_status)
	{
		case MAC_WAITING_DATA_REQ_TIMEOUT:

			mac_status = MAC_STATE_NORMAL;
			buf = stack_buffer_get(20);
			if(buf)
			{
				buf->options.type = BUFFER_DATA;
				buf->buf_ptr=0;
				buf->buf_end=1;
				buf->buf[0] = CMD_DATA_REQ;
				mac_build_command_frame_header(buf); /* Data request */
				buf->options.handle_type = HANDLE_DATA_REQ;
				mac_tx_add(buf);
				if(mac_timer_event==TIMER_IDLE)
				{
					generate_random_time_for_tx();
				}
			}
		break;


#ifdef MAC_RFD
		case MAC_WAITING_DATA_FROM_SERVER:
			mac_status = MAC_STATE_NORMAL;
			if(rf_802_15_4_PIB.state == CONNECT)
			{

			}
			else
			{
				/* Information for ntework manger that lost connect to coordinator */
	#ifdef HAVE_PERIOD_TIMER
				ptTimerStop();
	#endif /* HAVE_PERIOD_TIMER */
				rf_802_15_4_pib_reset();
				mac_handle_address_decoder(RF_DECODER_NONE);
				rf_802_15_4_ip_layer_address_mode_set(0);
				rf_802_15_4_PIB.mode =  BEACON_ENABLE_MODE;
				rf_802_15_4_PIB.state = DISSCONNECT;
				buf=stack_buffer_get(20);
				if(buf)
				{
					
					ptr = ( control_message_t*) buf->buf;
					ptr->message.ip_control.message_id = ASSOCIATION_LOST;
					push_to_app(buf);
					buf=0;
				}
			}
		break;
#endif /* MAC_RFD */
		case MAC_ACTIVE_SCAN:

		case MAC_ORPHAN_SCAN:
	
			if(scan_index==16)
			{
				if(mac_status == MAC_ACTIVE_SCAN)
				{
					rf_802_15_4_PIB.state = DISSCONNECT;
				}
				else
				{
					if(mac_result)
					{
						rf_802_15_4_pib_reset();
						mac_handle_address_decoder(RF_DECODER_NONE);
						rf_802_15_4_ip_layer_address_mode_set(0);
					}
				}
				mac_status = MAC_STATE_NORMAL; 
				if(mac_result)
				{
					mac_result->options.type = BUFFER_CONTROL;
					stack_buffer_push(mac_result);
					mac_result=0;
				}
				
			}
			else
			{
				debug_int(scan_index);
				debug(" chan\r\n");
				scan_operation();
			}
		break;

	default:

		break;


	}
}


void scan_operation(void)
{
	
	uint16_t tmp_16=0;
	buffer_t *buf;
	buf=stack_buffer_get(20);
	if(buf)
	{
		buf->options.type = BUFFER_DATA;
		buf->buf_end = 1;
		buf->buf_ptr = 0;
		if(mac_status == MAC_ACTIVE_SCAN)
		{
			buf->buf[0] = CMD_BEACON_REQ;
		}
		else if(mac_status == MAC_ORPHAN_SCAN)
		{
			buf->buf[0] = CMD_ORPHAN_NOTIFY;
		}
		mac_build_command_frame_header(buf);

		
		tmp_16 = (1 << scan_index);
		if(channel_list & tmp_16)
		{
			if(rf_channel_set(11+scan_index))
			{
				rf_802_15_4_PIB.logical_channel = 11+scan_index;
				mac_tx_add(buf);
				if(mac_timer_event==TIMER_IDLE)
				{
					generate_random_time_for_tx();
				}
			}
		}
		else
		{
			tmp_16=0;
		}	
		scan_index++;
	}
	else
	{
		debug("no buffers\r\n");
	}
	//mac_status_timer = xTaskGetTickCount();
}
#endif
/* General funnctios for all modes */
/**
 * Function parse general mac header part Frame control, sqn and address field.
 *
 * \param buf pointer tu buffer.
 * \param response include result of the analyze (Frame type, sqn and ack_req).
 */
uint8_t mac_header_analyze(buffer_t *buf)
{
	uint8_t *ind, intra=0, tmp_8=0, tmp,temp_length=0, fc[2], i, sqn, ack_req;
	uint8_t type = FC_NOT_DEFINED_FRAME;

	ind = (buf->buf + buf->buf_ptr);
	fc[0] = *ind++;					/* Read framecontrol field */
	fc[1] = *ind++;	
	sqn = *ind++;

	type = (fc[0] &  FC_FRAME_TYPE_MASK); /* detect frametype */

	ack_req = (fc[0] &  FC_ACK);
	intra =(fc[0] &  FC_INTRA_PAN);
	/* Check address field */
	/* Destination address */
	tmp=(fc[1] & FC_DST_MODE);
	if(tmp == FC_DST_ADDR_NONE)
	{
		tmp_8=0;
		buf->dst_sa.addr_type = ADDR_NONE;
	}	
	if(tmp == FC_DST_64_BITS)
	{
		tmp_8=8;
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
	}
	if(tmp== FC_DST_16_BITS)
	{
		tmp_8=2;
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
	}
	if(tmp_8)
	{
		/* Read dst pan */
		for(i=0; i<2; i++)
		{
			buf->dst_sa.address[tmp_8+i] = *ind++;
			device_own_address.address[tmp_8+i] =rf_802_15_4_PIB.pan_id[i];
		}
		for (i = 0; i < tmp_8 ; i++)
		{
			buf->dst_sa.address[i] = *ind++;
			if(tmp_8==8)
				device_own_address.address[i] = rf_802_15_4_PIB.rf_802_15_4_ext_addr[i];
			else
				device_own_address.address[i] = rf_802_15_4_PIB.rf_802_15_4_short_addr[i];
		}
		device_own_address.addr_type = buf->dst_sa.addr_type;
	}
	buf->dst_sa.port = 0;
	/* Source address */
	tmp=(fc[1] & FC_SRC_MODE);

	if(tmp == FC_SRC_ADDR_NONE)
	{
		tmp_8=0;
		buf->src_sa.addr_type = ADDR_NONE;
	}
	if(tmp == FC_SRC_64_BITS)
	{
		tmp_8=8;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;
	}
	if(tmp == FC_SRC_16_BITS)
	{
		tmp_8=2;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
	}

	if(tmp_8)
	{
		if(intra)
		{
			for (i = 0; i < tmp_8 ; i++)
			{
				buf->src_sa.address[i] = *ind++;
			}
			if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_LONG)
				temp_length=8;
			else if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT)
				temp_length=2;
			else
			{
				for(i=0; i<2;i++)
				{
					buf->src_sa.address[tmp_8+i] = rf_802_15_4_PIB.pan_id[i];
				}
			}
			if(temp_length)
			{
				for(i=0; i<2;i++)
				{
					buf->src_sa.address[tmp_8+i] = buf->dst_sa.address[temp_length+i];
				}
			}
		}
		else
		{
			for (i = 0; i < 2 ; i++)
			{
				buf->src_sa.address[tmp_8+i] = *ind++;
			}
			for (i = 0; i < tmp_8 ; i++)
			{
				buf->src_sa.address[i] = *ind++;
			}
		}
	}
	buf->buf_ptr = (ind - buf->buf);
	if(buf->src_sa.addr_type != ADDR_NONE)
	{
		// Filter duplicated packet and updated neighbortable
		if(update_neighbour_table(buf->src_sa.addr_type, buf->src_sa.address,buf->options.rf_dbm, sqn, UPDATE_NEIGHBOUR) ==0)
		{
			#ifndef HAVE_NRP
			//debug("Drop:seq.\r\n");
			type = FC_NOT_DEFINED_FRAME;
			#endif
		}
	}
	if ((type == FC_ACK_FRAME))
	{	
		 if ((buffer_on_air == 0) || (sqn != rf_802_15_4_PIB.last_tx_sqn))
		 {
			 type = FC_NOT_DEFINED_FRAME;
			 debug("Drop:ack.\r\n");
		 }
	}
	
	if(type == FC_COMMAND_FRAME || type == FC_DATA_FRAME)
	{
		/* destination address Filter */
		#ifdef HAVE_NRP
		if(buf->dst_sa.addr_type != ADDR_NONE)
		{
			if(mac_dest_check(buf, 0)==1 && ack_req)
			{
				rf_send_ack(0);
			}
		}
		#else
		i=0;
		if(rf_address_decoder_enable==0)
		{					
			
			if((type==FC_COMMAND_FRAME) &&  (buf->buf[buf->buf_ptr] == CMD_ASSOC_RESPONSE))
				i = mac_dest_check(buf, 1);
			else
				i = mac_dest_check(buf, 0);
		}
		else 
		{
			if((rf_802_15_4_PIB.mode != AD_HOC_MODE && (rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD)) && rf_802_15_4_PIB.device_type==MAC_RFD_TYPE)
				i = mac_rfd_src_check(buf);
			else
				i=1;
		}
		if(i)
		{
#ifndef RF_AUTO_ACK
#ifndef RF_ADDRESS_DECODER
			if(ack_req)
			{
				rf_send_ack(0);
			}
#endif
#endif
#ifdef AD_HOC_STATE
			if(type==FC_COMMAND_FRAME || type==FC_BEACON_FRAME)
			{
				type = FC_NOT_DEFINED_FRAME;
			}
	
#endif
		}
		else
		{
			type = FC_NOT_DEFINED_FRAME;
		}
#endif /* HAVE_NRP */
	}
	return type;
}


/**
 * Function parse and handle mac command frames.
 *
 * \param buf pointer tu buffer.
 * \param packet_check indicate dublicate packets.
 */
#ifndef AD_HOC_STATE
void mac_command_frames_handle(buffer_t *buf)
{
	uint8_t tmp_8;
	uint8_t mac_command=0, *ind, i;
	ind = (buf->buf + buf->buf_ptr);
	mac_command = *ind++;
	switch(mac_command)
	{
#ifdef MAC_FFD						
		case CMD_ORPHAN_NOTIFY:
			//debug("\r\nOrphan Notify");
			mac_ctrl_message_builder(buf ,ORPHAN_IND , 0, 0);
			buf=0;
			break;
		case CMD_BEACON_REQ:
			debug("Beacon Req\r\n");
			if(rf_802_15_4_PIB.mode == BEACON_ENABLE_COORD_MODE)
				rf_802_15_4_send_beacon(0);		/* Send beacon */
		
			stack_buffer_free(buf);
			buf = 0;
			break;

		case CMD_ASSOC_REQ:
			debug("Assoc Req\r\n");
			if (rf_802_15_4_PIB.mode == BEACON_ENABLE_COORD_MODE)		
			{
#ifndef RF_AUTO_ACK
				rf_send_ack(0);
#endif
				tmp_8 = *ind++;
				mac_ctrl_message_builder(buf ,ASSOC_IND , tmp_8, 0);
				buf=0;
			}
			if(buf)
			{
				stack_buffer_free(buf);
				buf = 0;	
			}
			break;
		case CMD_DATA_REQ:
			if(rf_802_15_4_PIB.mode == BEACON_ENABLE_COORD_MODE)
			{				
				debug("Data req\r\n");
					if(pending_buffer.buffer_count)
					{
						if(buf->src_sa.addr_type==ADDR_802_15_4_PAN_LONG)
							tmp_8=8;
						else
							tmp_8=2;

						for(i=0; i < MAX_PENDING_BUFFER_COUNT ; i++)
						{
							if(pending_buffer.p_data[i].buf && (pending_buffer.p_data[i].buf->dst_sa.addr_type == buf->src_sa.addr_type))
							{
								if( memcmp(pending_buffer.p_data[i].buf->dst_sa.address, buf->src_sa.address, tmp_8) == 0)
								{
		#ifndef RF_AUTO_ACK
									rf_send_ack(1);
		#endif
									/* Copy original buffer for forwarding */
									memcpy(buf, pending_buffer.p_data[i].buf, sizeof(buffer_t) + pending_buffer.p_data[i].buf->buf_end);
									buf->size = BUFFER_SIZE;									
									ind = (buf->buf +buf->buf_ptr );
									ind+=2;
									*ind++ = (rf_802_15_4_PIB.data_sqn);
									mac_sqn_update(1);
									buf->options.handle_type = HANDLE_ACK_REQ;
									stack_buffer_free(pending_buffer.p_data[i].buf);
									pending_buffer.p_data[i].buf=0;
									i=MAX_PENDING_BUFFER_COUNT;
									pending_buffer.buffer_count--;

									if(pending_buffer.buffer_count==0) rf_send_ack(0);

									if(hp_buffer==0 || hp_buf_secondary == 0)
									{
										if(hp_buffer==0)
										{
											hp_buffer=buf;
										}
										else
										{
											hp_buf_secondary = buf;
										}
										/*if(mac_timer_event == TIMER_IDLE)
										{
											mac_timer_event = WAITING_EVENT_LAUNCH_TX;
											timer_rf_launch(760/PLATFORM_TIMER_DIV); //Launch faster permission for tx
											
										}*/
									}
									else
									{
										mac_tx_add(buf);
										
									}
									if(mac_timer_event==TIMER_IDLE)
									{
										mac_timer_event = WAITING_EVENT_LAUNCH_TX;
										timer_rf_launch(760/PLATFORM_TIMER_DIV);//Launch faster permission for tx
									}
									buf=0;
								}
							}
						}
						if(buf)
						{			
#ifndef RF_AUTO_ACK
							rf_send_ack(0);
#endif
							stack_buffer_free(buf);
							buf=0;
														
						}
					}
					else
					{
						/* send ack without pending*/
	#ifndef RF_AUTO_ACK
						rf_send_ack(0);
	#endif
						stack_buffer_free(buf);
						buf=0;
					}						
			}
			if(buf)
			{
				stack_buffer_free(buf);
				buf=0;	
			}
			break;
#else
		case CMD_ASSOC_RESPONSE:
			timer_rf_stop();
			debug("ASSOC_RESPONSE\r\n");
			rf_send_ack(0);
			//task_status=0;
			
			mac_status = MAC_STATE_NORMAL;
			buf->buf_ptr=(ind - buf->buf);
			if(rf_802_15_4_PIB.state == DISSCONNECT)
			{
				for(i=0; i<2; i++)
				{
					rf_802_15_4_PIB.rf_802_15_4_short_addr[i] = *ind++;
				}
				tmp_8 = *ind++;
				if(tmp_8 == ASSOC_SUCCESSFUL )
				{
					rf_802_15_4_PIB.mac_options |= MAC_ASSOC_CORD;
					memcpy(rf_802_15_4_PIB.coord_ext_addr, buf->src_sa.address, 8);	
					/* Create Assocation confirm control-message */
					if(rf_802_15_4_PIB.rf_802_15_4_short_addr[0] == 0xfe && rf_802_15_4_PIB.rf_802_15_4_short_addr[1] == 0xff)
						rf_802_15_4_PIB.mac_options &= (~MAC_SUPPORT_SHORT_ADDR);
					else
						rf_802_15_4_PIB.mac_options |= MAC_SUPPORT_SHORT_ADDR;
#ifdef RF_AUTO_ACK
#ifdef RF_ADDRESS_DECODER											
						mac_handle_address_decoder(RF_DECODER_ON);
#endif
#endif
					/* Init IP layer use long address type */
					rf_802_15_4_ip_layer_address_mode_set(1);
					rf_802_15_4_PIB.state = CONNECT;
				}
				mac_ctrl_message_builder(buf ,ASSOC_CONFIRM , tmp_8, 0);
				buf=0;
			}
			break;

		case CMD_CORDINATOR_REALIGN:
			rf_send_ack(0);
		
			for(i=0; i < 2 ;i++)
			{
				rf_802_15_4_PIB.pan_id[i]= *ind++;
			}
			for(i=0; i < 2 ;i++)
			{
				rf_802_15_4_PIB.coord_short_Addr[i] = *ind++;
			}
			
			rf_802_15_4_PIB.logical_channel = *ind++;
			rf_channel_set(rf_802_15_4_PIB.logical_channel);
	
			for(i=0; i < 2 ;i++)
			{
				rf_802_15_4_PIB.rf_802_15_4_short_addr[i]= *ind++;
			}
	
			/* Create Synch loss indication control-message */
			mac_ctrl_message_builder(buf ,SYNCH_LOSS_IND , REALIGMENT, 0);
			buf=0;
			break;
#endif
		default:
			stack_buffer_free(buf);
			buf=0;
			break;
	}/*end of command type*/
	if(buf)
	{
		stack_buffer_free(buf);
		buf=0;
	}
}
#else /*AD_HOC_STATE*/
void mac_command_frames_handle(buffer_t *buf)
{
		stack_buffer_free(buf);
}
#endif /*AD_HOC_STATE*/

/**
 * Function create MAC-frame.
 *
 * Function got pointer for buffer-structure and build data-frame automatic.
 *
 * \param buf indicates pointer for buffer structure.
 *  \return  ack flag which indicated ack-requirement for frame.
 */
uint8_t rf_802_15_4_create_mac_frame(buffer_t *buf)
{
	uint8_t i , tmp_8=0,fc[2], address_length=0;
	uint8_t intra=0,ack=FALSE;
	uint8_t header_size = 3; /* including Frame Control and sqn */
	uint8_t *dptr;
	fc[0] = FC_DATA_FRAME;
	fc[1] = 0;

	if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
		address_length=8;
	if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
		address_length=2;
	if(address_length)
	{
		tmp_8 = buf->dst_sa.addr_type;
		buf->dst_sa.addr_type = ADDR_BROADCAST;
		for( i=0; i<address_length ;i++)
		{
			if( buf->dst_sa.address[i] != 0xff)
			{
				i=20;
				fc[0] |= FC_ACK;
				ack=TRUE;
				buf->dst_sa.addr_type = (addrtype_t) tmp_8;
			}
		}
	}
	if(buf->dst_sa.addr_type == ADDR_BROADCAST)
	{
		buf->dst_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
		memset(buf->dst_sa.address, 0xff, 4);
	}

	/* Check Intra flag use */
	if((buf->dst_sa.addr_type == ADDR_COORDINATOR || rf_802_15_4_PIB.device_type==MAC_RFD_TYPE) && (rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD))
	{
		if(rf_802_15_4_PIB.coord_addr_mode == ADDR_802_15_4_PAN_SHORT)
		{
			address_length=2;
			fc[1] |= FC_DST_16_BITS;
			header_size += 2;
			memcpy(buf->dst_sa.address, rf_802_15_4_PIB.coord_short_Addr, address_length);
		}
		else 
		{
			address_length=8;
			fc[1] |= FC_DST_64_BITS;
			header_size += 8;
			memcpy(buf->dst_sa.address, rf_802_15_4_PIB.coord_ext_addr, address_length);
		}
		buf->dst_sa.addr_type = rf_802_15_4_PIB.coord_addr_mode;
		for(i=0;i<2;i++)
		{
			buf->dst_sa.address[address_length+i] = rf_802_15_4_PIB.pan_id[i];
		}
		intra=1;
		fc[0] |= FC_INTRA_PAN;
	}
	else
	{
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
		{
			tmp_8=8;
			fc[1] |= FC_DST_64_BITS;
			header_size += 10;
		}
		if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
		{
			tmp_8=2;
			fc[1] |= FC_DST_16_BITS;
			header_size += 4;
		}

		if(memcmp(&buf->dst_sa.address[tmp_8],rf_802_15_4_PIB.pan_id,2 ) == 0 ) /* Compare PAN-IDs --> if match use intra-flag */
		{
			intra=1;
			fc[0] |= FC_INTRA_PAN;
			header_size -=2;
		}
	}	

	/* Checkout Source addresstype */
	if(((rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD) || rf_802_15_4_PIB.mode==AD_HOC_MODE ) && rf_802_15_4_PIB.mac_options & MAC_SUPPORT_SHORT_ADDR)
	{
		fc[1] |= FC_SRC_16_BITS;
		header_size +=4;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_SHORT;
	}
	else
	{
		fc[1]  |= FC_SRC_64_BITS;
		header_size +=10;
		buf->src_sa.addr_type = ADDR_802_15_4_PAN_LONG;	
	}

	if(stack_buffer_headroom( buf,(uint16_t)header_size)==pdFALSE)
	{
		control_message_t *ptr=0;
		debug("mac stop\r\n");
		buf->buf_end = 0;
		buf->buf_ptr = 0;
		ptr = ( control_message_t*) buf->buf;
		ptr->message.ip_control.message_id = TOO_LONG_PACKET;
		push_to_app(buf);
		buf=0;
		return 0xff;
	}
	
	buf->buf_ptr -= header_size;
	dptr = buf->buf + buf->buf_ptr;
	*dptr++ = fc[0]; 	/* FCF*/
	*dptr++ = fc[1];

	*dptr++ = rf_802_15_4_PIB.data_sqn;		/*packet sequence number*/
	mac_sqn_update(1);
	/* Craete address field */
	/* Destination address */
	dptr=stack_insert_address_to_buffer(dptr, buf->dst_sa.addr_type, buf->dst_sa.address);
	/* Source address */
	if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_LONG)
	{
		if(intra == 0) dptr=mac_add_address(buf, dptr, 1, 1);
		else dptr=mac_add_address(buf, dptr, 1, 0);
	}
	else
	{
		if(intra == 0)
		{
			dptr=mac_add_address(buf, dptr, 2, 1);
		}
		else
		{
			dptr=mac_add_address(buf, dptr, 2, 0);
		}
	}
	return ack;
}

/**
 * Store address information to ram for address decoder.
 *
 */
void rf_802_15_4_store_address_to_ram(void)
{
	sockaddr_t address;
	uint8_t i;
	/* Write devices default address to ram */
	for(i=0; i<2;i++)
	{
		address.address[i]= rf_802_15_4_PIB.rf_802_15_4_short_addr[i];
	}
	for(i=0; i<2;i++)
	{
		address.address[2+i]= rf_802_15_4_PIB.pan_id[i];
	}
	address.addr_type = ADDR_802_15_4_PAN_SHORT;
	rf_set_address(&address);
	for(i=0; i<8;i++)
	{
		address.address[i] = rf_802_15_4_PIB.rf_802_15_4_ext_addr[i];
	}
	for(i=0; i<2;i++)
	{
		address.address[8+i]= rf_802_15_4_PIB.pan_id[i];
	}
	address.addr_type = ADDR_802_15_4_PAN_LONG;
	rf_set_address(&address);
}

/**
 * Reset MAC PIB information structure.
 *
 * This function call when want to initialize MAC PIB to default values. 
 *
 *  
 */
void rf_802_15_4_pib_reset(void)
{
	uint8_t i;
	slotted=0;
	cap_active=1;
	/* Init data_pending_buffer */
#ifdef MAC_FFD
#ifndef AD_HOC_STATE
	pending_buffer.buffer_count = 0;
	rf_802_15_4_PIB.beacon_sqn = 0;
#endif
	rf_802_15_4_PIB.device_type = MAC_FFD_TYPE;
	
#else
	rf_802_15_4_PIB.device_type=MAC_RFD_TYPE;
#endif
	/* Init MAC-PIB to default */
	rf_802_15_4_PIB.mac_options=0;	/* GTS/ASSOC, not permit, ASSOC_PAN_CORD,BATT_LIFE-EXT false... */
#ifdef SUPPORT_SHORT_ADDRESS
	rf_802_15_4_PIB.mac_options |=  MAC_SUPPORT_SHORT_ADDR;
#endif
	rf_802_15_4_PIB.mac_options |=  MAC_AUTO_REQ;
	rf_802_15_4_PIB.max_csma_backoffs = 5;
	rf_802_15_4_PIB.min_be = 2;
	rf_802_15_4_PIB.state = AD_HOC;
	rf_802_15_4_PIB.mode = AD_HOC_MODE;
	rf_802_15_4_PIB.transaction_persistence_time = 5; /* time when coordiantor delete strored packet, when no one want them 75 seconds */

	rf_802_15_4_PIB.logical_channel = RF_DEFAULT_CHANNEL;
	rf_802_15_4_PIB.last_tx_sqn = 0;
	rf_802_15_4_PIB.beacon_order = 15;
	rf_802_15_4_PIB.superframe_order = 15;
	memset(rf_802_15_4_PIB.coord_ext_addr, 0, 8);
	
	for(i=0; i < 2 ;i++)
	{
		rf_802_15_4_PIB.rf_802_15_4_short_addr[i]= 0xff;
		rf_802_15_4_PIB.pan_id[i]= 0xff;
		rf_802_15_4_PIB.coord_short_Addr[i]=0x00;
	}
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
		ip_address_setup( 1,rf_802_15_4_PIB.rf_802_15_4_short_addr); 
	}
	else
		ip_address_setup( 0,NULL); 
}

/**
 * Function update MAC data and beacon sqn number by gives parameter.
 * 
 *
 *  \param  sqn_data 1=DATA & 0=BEACON.
 */

void mac_sqn_update(uint8_t sqn_data)
{
	if(sqn_data)
	{
		rf_802_15_4_PIB.last_tx_sqn = rf_802_15_4_PIB.data_sqn;
		if(rf_802_15_4_PIB.data_sqn == 0xff)
			rf_802_15_4_PIB.data_sqn=0;
		else
			rf_802_15_4_PIB.data_sqn++;
	}
	else
	{
		if(rf_802_15_4_PIB.beacon_sqn == 0xff)
				rf_802_15_4_PIB.beacon_sqn=0;
		else
			rf_802_15_4_PIB.beacon_sqn++;
	}
}

/**
 * Function is for set up some MAC-PIB parameters.
 *
 *  \param pointer to set up arguments.
 *  \param type type attribute for MAC-PIB parameter.
 */

void mac_set_mac_pib_parameter(uint8_t *pointer, mac_pib_enum_t type)
{
	uint8_t i;
	if(type==MAC_SHORT_ADDRESS)	
	{
		rf_802_15_4_PIB.rf_802_15_4_short_addr[0]= pointer[0];
		rf_802_15_4_PIB.rf_802_15_4_short_addr[1]= pointer[1];
		rf_802_15_4_store_address_to_ram();
	}
	else if(type==MAC_IEEE_ADDRESS)
	{
		for(i=0;i<8; i++)
		{
			rf_802_15_4_PIB.rf_802_15_4_ext_addr[i]= pointer[i];
		}
		rf_802_15_4_store_address_to_ram();
	}
#ifndef AD_HOC_STATE
	else if(type == PENDING_TTL)
	{
		rf_802_15_4_PIB.transaction_persistence_time = *pointer;
	}
#endif
	else if(RUNNING_MODE)
	{
		if( *pointer == BEACON_ENABLE_MODE)
		{
			rf_802_15_4_PIB.mode =  BEACON_ENABLE_MODE;
			rf_802_15_4_PIB.state = DISSCONNECT;
		}
	}
}

/**
 * API for address decoder use for diffrent device type.
 *
 * \param mode RF_DECODER_NONE / RF_DECODER_COORDINATOR / RF_DECODER_ON.
 */

void mac_handle_address_decoder(rf_address_mode_t mode)
{
	switch(mode)
	{
		case RF_DECODER_NONE:
			rf_address_decoder_mode(mode);
			rf_address_decoder_enable=0;
			break;

		case RF_DECODER_COORDINATOR:
		case RF_DECODER_ON:
			if(rf_address_decoder_mode(mode) == pdTRUE)
			{
				rf_802_15_4_store_address_to_ram();
				rf_address_decoder_enable=1;
			}
			break;
	}
}

/**
 * Function build and send control messages to Network manager or Application.
 * 
 *
 *  \param  buf pointer for data structure.
 *  \param  message_id define message structure.
 *  \param temp iclude some control parameter.
 *  \param value wanted setup value defined by temp parameter.
 */
#ifndef AD_HOC_STATE
void mac_ctrl_message_builder(buffer_t *buf , mac_control_id_t message_id ,  uint8_t temp, uint8_t value)
{
	control_message_t *ctrl_ptr=0;
	ctrl_ptr = ( control_message_t*) buf->buf;
	ctrl_ptr->message.mac_control.message_id = message_id;
	switch(message_id)
	{

#ifdef MAC_RFD
		case SYNCH_LOSS_IND:
			if(temp != REALIGMENT)
			{
				rf_802_15_4_PIB.mode =  BEACON_ENABLE_MODE;
				rf_802_15_4_PIB.state = DISSCONNECT;
			}
			/* Create Synch loss indication control-message */
			ctrl_ptr->message.mac_control.message.synch_loss_reason.synch_lost_reason = temp;
			break;
#else
		case ORPHAN_IND:
			memcpy(ctrl_ptr->message.mac_control.message.orphan_ind.orphan_address, buf->src_sa.address, 8);
			break;

		case ASSOC_IND:
			ctrl_ptr->message.mac_control.message.assoc_ind.cap_info = temp;
			ctrl_ptr->message.mac_control.message.assoc_ind.security_use = 0;
			memcpy(ctrl_ptr->message.mac_control.message.assoc_ind.device_address, buf->src_sa.address, 8);
			break;
#endif
		case ASSOC_CONFIRM:
			ctrl_ptr->message.mac_control.message.assoc_confirm = (assoc_status_t) temp;
			break;

		default:
			stack_buffer_free(buf);
			buf=0;
			break;
	}
	if(buf)
	{
		buf->options.type = BUFFER_CONTROL;
		buf->to 	= MODULE_NWK_MANAGER;
		buf->from 	= MODULE_RF_802_15_4;
		buf->dir    = BUFFER_UP; 
		stack_buffer_push(buf);
		buf=0;
	}
}
#else
void mac_ctrl_message_builder(buffer_t *buf , mac_control_id_t message_id ,  uint8_t temp, uint8_t value)
{
	stack_buffer_free(buf);
	buf=0;
}
#endif
/* Random value table */
/*const uint8_t rand_table[16] =
{
	0x08, 0x0b, 0x03, 0x06, 0x10, 0x0d, 0x02, 0x0a, 0x04, 0x0f, 0x0e, 0x05, 0x0c, 0x09, 0x07, 0x11
};*/

/* Random value table */
/*const uint16_t rand_table[16] =
{
	2250/32, 505/32, 640/32, 1610/32, 1343/32, 459/32, 320/32, 858/32, 960/32, 711/32, 2580/32, 1290/32, 1267/32, 1087/32, 1930/32, 1111/32
};*/


/* Random value table */

const uint16_t mac_rand_table[16] =
{
	1999/PLATFORM_TIMER_DIV, 3505/PLATFORM_TIMER_DIV, 2640/PLATFORM_TIMER_DIV, 1665/PLATFORM_TIMER_DIV, 2187/PLATFORM_TIMER_DIV, 3459/PLATFORM_TIMER_DIV, 1320/PLATFORM_TIMER_DIV, 2858/PLATFORM_TIMER_DIV, 1960/PLATFORM_TIMER_DIV, 3711/PLATFORM_TIMER_DIV, 1389/PLATFORM_TIMER_DIV, 1532/PLATFORM_TIMER_DIV, 2734/PLATFORM_TIMER_DIV, 3452/PLATFORM_TIMER_DIV, 3471/PLATFORM_TIMER_DIV, 4012/PLATFORM_TIMER_DIV
};

/**
 * Random value function.
 *
 * Function create random value between 1-16 and return that. Function use random value for index to select random value from rand_table-array. 
 *
 *  \return  rand random value.
 */
uint16_t mac_random_generate(void) {
	uint16_t rand;
	static unsigned seed =1;
	if((((seed << 1) ^ seed) & 0x08) != 0)
                seed = (seed << 1) | 1;
        else
		seed <<= 1;
	seed &= 0x0f;
	rand =(mac_rand_table[seed]);
        return rand;
}

/**
 * Current used function which address to buffer from MAC-PIB.
 * 
 *  Only for stack internal use.
 */
uint8_t * mac_add_address(buffer_t *buf, uint8_t *ind, uint8_t type, uint8_t pan_id)
{
	uint8_t i;
	if(pan_id)
	{
		*ind++ = rf_802_15_4_PIB.pan_id[0];
		*ind++ = rf_802_15_4_PIB.pan_id[1];
	}

	if(type==1)
	{
		for(i=0; i < 8 ; i++)
		{
		*ind++ = rf_802_15_4_PIB.rf_802_15_4_ext_addr[i];
		}
	}
	else if(type==2)
	{
		for(i=0; i < 2 ; i++)
		{
		*ind++ = rf_802_15_4_PIB.rf_802_15_4_short_addr[i];
		}
	}
	else
	{

	}
	return ind;
}
#ifndef AD_HOC_STATE
#ifdef SUPERFRAME_MODE

/**
 * Set MAC layers superframe current parameter.
 * 
 *  Only for stack internal use.
 */
void mac_init_superframe_options(uint8_t minus)
{
	/* Initialize general parameter for superframe mode */
	timeslot_count=16;
	cap_active=1;
	ptTimerStart((synch - minus), &rf_802_15_4_timeslot_update); /* Launch time slot timer */
	rf_802_15_4_PIB.state = CONNECT;
}

/**
 * Function synchronize superframe durations timeslot.
 *
 * Period timer cuold call this function when it detect that timeslot duration is over.
 * Function decreace timeslot_count and if detect zero value it setup cap_active --> zero.
 *
 */

void rf_802_15_4_timeslot_update(void *unused_parameter)
{
	timeslot_count--;
	if(timeslot_count==1) last_slot_start = xTaskGetTickCount();	/* Save time when last time slot start */

	if(timeslot_count==0)
	{
		cap_active=0;
#ifdef MAC_FFD
		/* add check for gateway mode? */	
		rf_802_15_4_send_beacon(1);
#else
		ptTimerStop();
		rf_superfarme_synch_event(0);
#endif
	}
}
#endif /* SUPERFRAME_MODE */


#ifdef MAC_FFD

portCHAR mac_pending_req(buffer_t *buf)
{
	uint8_t fc[2], header_len=0, *dptr;
	mac_15_4_event_t mac_event;
	portCHAR ret_val=pdFALSE;
	/*add check for gateway mode?*/
	if(pending_buffer.buffer_count < MAX_PENDING_BUFFER_COUNT && rf_802_15_4_PIB.mode == BEACON_ENABLE_COORD_MODE)
	{
		if(pending_buffer.buffer_count < MAX_PENDING_BUFFER_COUNT)
		{
			header_len=3;
			fc[1]=0;
			if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_LONG)
			{
				fc[1] = FC_DST_64_BITS;
				header_len += 10;
			}
			else
			{
				fc[1] = FC_DST_16_BITS;
				header_len += 4;
			
			}
			if(buf->from == MODULE_NWK_MANAGER)
			{
				fc[1] |= FC_SRC_64_BITS;
				header_len += 10;
				fc[0] = FC_COMMAND_FRAME;
			}
			else
			{
				fc[0] = (FC_DATA_FRAME +FC_INTRA_PAN);
				if((rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD) && (rf_802_15_4_PIB.mac_options & MAC_SUPPORT_SHORT_ADDR))
				{
					fc[1] |= FC_SRC_16_BITS;
					header_len += 2;
				}
				else
				{
					fc[1] |= FC_SRC_64_BITS;
					header_len += 8;
				}
			}
			fc[0] |= FC_ACK;

			if(stack_buffer_headroom(buf,header_len) == pdFALSE)
			{
				control_message_t *ptr = ( control_message_t*) buf->buf;
				ptr->message.ip_control.message_id = TOO_LONG_PACKET;
				push_to_app(buf);
				buf=0;
				return pdFALSE;
			}
			

			buf->buf_ptr -= header_len;
			dptr = (buf->buf + buf->buf_ptr);
			*dptr++ = fc[0];
			*dptr++ = fc[1];
			*dptr++ = 0x00;
			*dptr++ = rf_802_15_4_PIB.pan_id[0];
			*dptr++ = rf_802_15_4_PIB.pan_id[1];
			if(buf->dst_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
			{
				dptr=stack_insert_address_to_buffer(dptr, ADDR_802_15_4_SHORT, buf->dst_sa.address);
			}
			else
			{
				dptr=stack_insert_address_to_buffer(dptr, ADDR_802_15_4_LONG, buf->dst_sa.address);
			}
			if(buf->from == MODULE_NWK_MANAGER)
			{	
				*dptr++ = rf_802_15_4_PIB.pan_id[0];
				*dptr++ = rf_802_15_4_PIB.pan_id[1];
				dptr=mac_add_address(buf, dptr, 1, 0);
			}
			else
			{
				if((rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD) && (rf_802_15_4_PIB.mac_options & MAC_SUPPORT_SHORT_ADDR))
					dptr=mac_add_address(buf, dptr, 2, 0);
				else
					dptr=mac_add_address(buf, dptr, 1, 0);
			}
			
			mac_event.id = MAC_PENDING;
			mac_event.buf=buf;
			
			if (xQueueSend(rf_802_15_4_queue, &mac_event,0) == pdTRUE)
			{
				ret_val = pdTRUE;
			}
			else
			{
				stack_buffer_free(mac_event.buf);
				debug("MAC queue full\r\n");
			}
			buf=0;
		}
	}
	else
	{
		if(buf)
		{
			stack_buffer_free(buf);
		}
		debug("no space\r\n");
	}
	return ret_val;
}


/**
 * Function set assocation permit to NOT_ACCPEPT state.
 *
 * NWK_MANAGER call this funktion when it want that other device can join its PAN-network
 */
void mac_assoc_permit_false(void)
{
	rf_802_15_4_PIB.mac_options &= (~MAC_ASSOCPERMIT);
}

/**
 * Set MAC layers superframe synch parameters.
 * 
 *  Only for stack internal use.
 */
void mac_set_synch_parameters(void)
{
	switch (rf_802_15_4_PIB.beacon_order)
	{
		case 4:
			synch=BI_4;
			last_slot_synch_time = LAST_SLOT_4;
			break;
		case 5:
			synch=BI_5;
			last_slot_synch_time = LAST_SLOT_5;
			break;
		case 6:
			synch=BI_6;
			last_slot_synch_time = LAST_SLOT_6;
			break;
		case 7:
			synch=BI_7;
			last_slot_synch_time = LAST_SLOT_7;
			break;
		case 8:
			synch=BI_8;
			last_slot_synch_time = LAST_SLOT_8;
			break;
		case 9:
			synch=BI_9;
			last_slot_synch_time = LAST_SLOT_9;
			break;
		case 10:
			synch=BI_10;
			last_slot_synch_time = LAST_SLOT_10;
			break;
		case 11:
			synch=BI_11;
			last_slot_synch_time = LAST_SLOT_11;
			break;
		case 12:
			synch=BI_12;
			last_slot_synch_time = LAST_SLOT_12;
			break;
		default:
			synch=BI_7;
			last_slot_synch_time = LAST_SLOT_7;
			break;
	}
}

/**
 * Function create and sen beacon frame.
 *
 * Function create, send beacon frame and setup event timer for superframe time slot syncronizing and setup also period timer for next send event.
 *
 * \param param Superframe-mode synch-process only only sets --> 1 which start new superframe.
 */
void rf_802_15_4_send_beacon(uint8_t param)
{
	buffer_t *b;
	b=stack_buffer_get(0);
	if(b)
	{  
		uint8_t fc[2], tmp_8=0, *ind;
		portCHAR ret_value;
		b->buf_end = 0;
		b->buf_ptr = 0;
		ind = (b->buf + b->buf_ptr);
		fc[0] = FC_BEACON_FRAME;
		if(rf_802_15_4_PIB.mac_options & MAC_SUPPORT_SHORT_ADDR) fc[1] = FC_SRC_16_BITS;
		else fc[1] = FC_SRC_64_BITS;
		*ind++ = fc[0]; 	/* FCF*/
		*ind++ = fc[1];
		*ind++ = (rf_802_15_4_PIB.beacon_sqn);	/* packet sequence number */
		mac_sqn_update(0);
		/* Address field has just destination: dest address and PAN-id are broadcast */
		if(rf_802_15_4_PIB.mac_options & MAC_SUPPORT_SHORT_ADDR)
			ind=mac_add_address(b, ind, 2, 1);	/* PAN+Short address */
		else 
			ind=mac_add_address(b, ind, 1, 1);	/* PAN+Long address */
		/* Superframe specification */
		*ind++ = (rf_802_15_4_PIB.beacon_order | (rf_802_15_4_PIB.superframe_order << 4));
		if(rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD)
			tmp_8 = 0x80;
		if(rf_802_15_4_PIB.mac_options & MAC_BATT_LIFE_EXT)
			tmp_8 |= 0x10;
		tmp_8 |= 0x40;
		tmp_8 |= 0x0F;
		*ind++ = tmp_8;			
		*ind++ = 0x00;		/* GTS field first disabled */						
		*ind++ = 0x00;		/* Pending addresslist  first disabled*/
		*ind++ = 0x00;		/* Beacon payload includes normal network layer information */
		b->buf_end = (ind -b->buf);
		
		if(param)
		{
			ret_value = rf_write_no_cca(b);
#ifdef SUPERFRAME_MODE
			ptTimerStart(synch, &rf_802_15_4_timeslot_update);
			timeslot_count=16;
			cap_active=1;
#endif
		}
		else
		{
			if(hp_buffer==0 || hp_buf_secondary == 0)
			{
				if(hp_buffer==0)
				{
					hp_buffer=b;
				}
				else
				{
					hp_buf_secondary = b;
				}
			}
			else
			{
				mac_tx_add(b);
				/*if(mac_timer_event==TIMER_IDLE)
				{
					mac_timer_event = WAITING_EVENT_LAUNCH_TX;
					timer_rf_launch(160/PLATFORM_TIMER_DIV);//Launch faster permission for tx
				}*/
			}
			if(mac_timer_event == TIMER_IDLE)
			{
				generate_random_time_for_tx();
			}
		}
	}
}

/**
 * Function update pending buffers time to live values and free buffers when ttl is zero.
 *
 */
void mac_check_pending_status(void)
{
	/*add check for gateway mode?*/
	uint8_t i, minus_count=0;
	control_message_t *ctrl_ptr=0;
	if(pending_buffer.buffer_count)
	{
		for(i=0; i< MAX_PENDING_BUFFER_COUNT; i++)
		{
			if(pending_buffer.p_data[i].buf)
			{
				if( pending_buffer.p_data[i].ttl > 1) pending_buffer.p_data[i].ttl--;
				else
				{
					pending_buffer.p_data[i].buf->buf_ptr=0;
					pending_buffer.p_data[i].buf->buf_end=0;
					ctrl_ptr = ( control_message_t*) pending_buffer.p_data[i].buf->buf;
					ctrl_ptr->message.ip_control.message_id = 			PENDING_TIMEOUT;
					push_to_app(pending_buffer.p_data[i].buf);
					pending_buffer.p_data[i].buf=0;
					minus_count++;
				}
			}
		}
	}
	if(minus_count)
	{ 
		pending_buffer.buffer_count -= minus_count;	
		if(pending_buffer.buffer_count==0) 
			rf_send_ack(0);
	}
}


#else

#ifdef SUPERFRAME_MODE
/**
 * Function call start beacon tracking.
 *
 * Nodes event timer launch this function when coordinator use superframe mode.
 * This messgae launch beacon track process.
 *
 */
void rf_superfarme_synch_event(void *param)
{
	beacon_track=1; /* Activate beacon track case for MAC-task*/
	rf_802_15_4_PIB.state = BEACON_TRACK; /* For MAC-handle filter */
}
#endif

void rf_802_15_4_slot_timing(uint8_t slot, uint16_t *synch_value, uint16_t *last_slot)
{
	uint16_t synch_val = SLOT_4;
	uint16_t last_slot_val = LAST_SLOT_4;
	
	switch (slot)
	{
		case 4:
			synch_val=SLOT_4;
			last_slot_val= LAST_SLOT_4;
			break;
		case 5:
			synch_val=SLOT_5;
			last_slot_val= LAST_SLOT_5;
			break;
		case 6:
			synch_val=SLOT_6;
			last_slot_val= LAST_SLOT_6;
			break;
		case 7:
			synch_val=SLOT_7;
			last_slot_val= LAST_SLOT_7;
			break;
		case 8:
			synch_val=SLOT_8;
			last_slot_val= LAST_SLOT_8;
			break;
		case 9:
			synch_val=SLOT_9;
			last_slot_val= LAST_SLOT_9;
			break;
		case 10:
			synch_val=SLOT_10;
			last_slot_val= LAST_SLOT_10;
			break;
		case 11:
			synch_val=SLOT_11;
			last_slot_val= LAST_SLOT_11;
			break;
		case 12:
			synch_val=SLOT_12;
			last_slot_val= LAST_SLOT_12;
			break;
	}
	*synch_value = synch_val;
	*last_slot = last_slot_val;
}

#endif



/**
 * Function build mac command frame headers.
 *
 * Header structure based on MAc COMMAND ID which is buffers first byte.
 *
 * \param buf pointer for buffer 
 */
void mac_build_command_frame_header(buffer_t *buf)
{
	uint8_t fc[2], len=3, intra=0, i, adr_len=0, src_pan=0, orphan=0;
	uint8_t *dptr;
	fc[0] = FC_COMMAND_FRAME;
	fc[1] = 0;
	switch(buf->buf[ buf->buf_ptr])
	{
#ifdef MAC_RFD
		case CMD_ORPHAN_NOTIFY:
			fc[0] = (FC_COMMAND_FRAME | FC_INTRA_PAN);
			fc[1] = (FC_DST_16_BITS | FC_SRC_64_BITS);
			len += 12;
			intra=1;
			buf->dst_sa.addr_type=ADDR_802_15_4_PAN_SHORT;
			memset(buf->dst_sa.address, 0xff, 2);
			buf->src_sa.addr_type=ADDR_802_15_4_PAN_LONG;
			memcpy(buf->src_sa.address, rf_802_15_4_PIB.rf_802_15_4_ext_addr, 8);
			orphan=1;
			break;

		case CMD_ASSOC_REQ:
			src_pan=1;
		case CMD_DATA_REQ:
			fc[0] |= FC_ACK;
			fc[1] = (FC_SRC_64_BITS);
			buf->src_sa.addr_type=ADDR_802_15_4_PAN_LONG;
			memcpy(buf->src_sa.address, rf_802_15_4_PIB.rf_802_15_4_ext_addr, 8);		

			buf->dst_sa.addr_type = rf_802_15_4_PIB.coord_addr_mode;
			if(rf_802_15_4_PIB.coord_addr_mode == ADDR_802_15_4_PAN_SHORT)
			{
				fc[1] |= FC_DST_16_BITS;
				len += 14;
				memcpy(buf->dst_sa.address, rf_802_15_4_PIB.coord_short_Addr, 2);
			}
			else
			{
				rf_802_15_4_PIB.coord_addr_mode = ADDR_802_15_4_PAN_LONG;
				fc[1] |= FC_DST_64_BITS;
				len += 20;
				memcpy(buf->dst_sa.address, rf_802_15_4_PIB.coord_ext_addr, 8);
			}
			
			break;

		case CMD_BEACON_REQ:
			fc[1] = FC_DST_16_BITS;
			len += 4;
			buf->dst_sa.addr_type=ADDR_802_15_4_PAN_SHORT;
			memset(buf->dst_sa.address, 0xff, 2);
			buf->src_sa.addr_type = ADDR_NONE;
			break;
#else
		case CMD_CORDINATOR_REALIGN:
			orphan=1;
			fc[1] = (FC_DST_64_BITS | FC_SRC_64_BITS);
			len += 20;
			buf->dst_sa.addr_type=ADDR_802_15_4_PAN_LONG;
			buf->src_sa.addr_type=ADDR_802_15_4_PAN_LONG;
			memcpy(buf->src_sa.address, rf_802_15_4_PIB.coord_ext_addr, 8);
			break;
#endif
	}
	if(stack_buffer_headroom( buf,(uint16_t)len)==pdFALSE)
	{
		control_message_t *ptr=0;
		buf->buf_end = 0;
		buf->buf_ptr = 0;
		ptr = ( control_message_t*) buf->buf;
		ptr->message.ip_control.message_id = TOO_LONG_PACKET;
		push_to_app(buf);
		buf=0;
	}

	buf->buf_ptr -= len;
	dptr = buf->buf + buf->buf_ptr;
	*dptr++ = fc[0]; 	/* FCF*/
	*dptr++ = fc[1];
	*dptr++ = (rf_802_15_4_PIB.data_sqn);	/*packet sequence number*/
	mac_sqn_update(1);
	if(orphan)
	{
		*dptr++ = 0xff; 	
		*dptr++= 0xff;
	}
	else
	{
		*dptr++ = rf_802_15_4_PIB.pan_id[0]; 	
		*dptr++= rf_802_15_4_PIB.pan_id[1];
	}

	if(buf->dst_sa.addr_type == ADDR_802_15_4_PAN_SHORT) adr_len=2;
	else adr_len=8;
	for(i=0; i<adr_len;i++)
	{
		*dptr++ = buf->dst_sa.address[i];
	}
	if(buf->src_sa.addr_type != ADDR_NONE)
	{
		if(intra==0)
		{
			if(src_pan)
			{
				*dptr++ = 0xff;
				*dptr++ = 0xff;
			}
			else
			{
				*dptr++ = rf_802_15_4_PIB.pan_id[0]; 	
				*dptr++ = rf_802_15_4_PIB.pan_id[1];
			}
		}
		if(buf->src_sa.addr_type == ADDR_802_15_4_PAN_SHORT) adr_len=2;
		else adr_len=8;
		for(i=0; i<adr_len;i++)
		{
			*dptr++ = buf->src_sa.address[i];
		}
	}
}
#endif /* AD_HOC-STATE */
/**
 * Function call get coordinator address from MAC-PIB.
 *
 * \param address poniter for address structure where functions write address information.
 */
portCHAR get_coord_address(sockaddr_t *address)
{
	if(rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD)
	{
		uint8_t i, tmp_8=0;
		address->addr_type = rf_802_15_4_PIB.coord_addr_mode;
		if(rf_802_15_4_PIB.coord_addr_mode == ADDR_802_15_4_PAN_SHORT) tmp_8=2;
		else tmp_8=8;
		for(i=0; i < tmp_8 ; i++)
		{
			if(rf_802_15_4_PIB.coord_addr_mode == ADDR_802_15_4_PAN_LONG)
				address->address[i] = rf_802_15_4_PIB.coord_ext_addr[i];
			else
				address->address[i] = rf_802_15_4_PIB.coord_short_Addr[i];
		}
		return pdTRUE;
	}
	return pdFALSE;
}
/**
 * Function compare source adrres to coordinator address.
 *
 * .
 *
 * \param buf pointer for buffer
 * \param
 * \return 1 when address is from coordinator
 * \return 0 when packet its not from coordinator 
 */
uint8_t mac_rfd_src_check(buffer_t *buf)
{
	if(buf->src_sa.addr_type==ADDR_802_15_4_PAN_SHORT)
	{
		if(memcmp(buf->src_sa.address, rf_802_15_4_PIB.coord_short_Addr, 2) == 0) return 1;
	}
	else
	{
		if(memcmp(buf->src_sa.address, rf_802_15_4_PIB.coord_ext_addr, 8) == 0) return 1;
	}
	return 0;
}


/**
 * Function check destination address and also source when device is RFD and its running beacon enable mac-mode.
 *
 * .
 *
 * \param buf pointer for buffer
 * \param
 * \return 1 when destination match, RFD in the Beacon-enable state compare
 * \return 0 when packet its not for this device 
 */

uint8_t mac_dest_check(buffer_t *buf, uint8_t flag)
{
#ifdef HAVE_NRP
	if(device_own_address.addr_type == ADDR_802_15_4_PAN_SHORT)
	{
		if(memcmp(device_own_address.address, buf->dst_sa.address, 4) == 0) return 1;
	}
	else
	{
		if(stack_compare_address(&device_own_address, &buf->dst_sa) == pdTRUE) return 1;
	}
#else
	 uint8_t from_cord=1;
	/* check destination address */
#ifdef MAC_RFD
#ifndef AD_HOC_STATE
    from_cord=0;
	if(rf_802_15_4_PIB.mode != AD_HOC_MODE && (rf_802_15_4_PIB.mac_options & MAC_ASSOC_CORD))
	{
		from_cord = mac_rfd_src_check(buf);
	}
	else
	{
		from_cord=1;
	}
#endif
#endif
	if(from_cord)
	{
		if(flag)
		{
			if( memcmp(buf->dst_sa.address,rf_802_15_4_PIB.rf_802_15_4_ext_addr, 8 ) == 0 ) return 1;
		}
		else
		{
	
			if(device_own_address.addr_type == ADDR_802_15_4_PAN_SHORT)
			{
				if(memcmp(device_own_address.address, buf->dst_sa.address, 4) == 0) return 1;
				if(flag==0)
				{
					if(stack_check_broadcast(buf->dst_sa.address, buf->dst_sa.addr_type) == pdTRUE)
						return 1;
				}
			}
			else
			{
				if(stack_compare_address(&device_own_address, &buf->dst_sa) == pdTRUE) return 1;
			}
		}
	}
#endif /* HAVE_NRP */
	return 0; /* NOT from coordinator or NOT for this device --> packet will be discard */
}


mac_error_t mac_pib_set( mac_param_t *par)
{
	mac_error_t response = MAC_SUCCESS;
	switch (par->id)
	{
		case MAC_CURRENT_CHANNEL:
			if(rf_channel_set(par->param.channel))
			{
				rf_802_15_4_PIB.logical_channel = par->param.channel;
				pause_us(50);
			}
			else response = MAC_FAILURE;;
			break;
		default:
			response = MAC_FAILURE;
		break;

	}
	return response;
}

uint8_t *mac_get_mac_pib_parameter(mac_pib_enum_t type)
{
	uint8_t *response=NULL;
	if(type== MAC_CURRENT_CHANNEL)
		response = &(rf_802_15_4_PIB.logical_channel);

	if(type== MAC_IEEE_ADDRESS)
		response = rf_802_15_4_PIB.rf_802_15_4_ext_addr;
		
	return response;
}
