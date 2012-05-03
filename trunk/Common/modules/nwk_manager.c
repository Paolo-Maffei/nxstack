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
 * \file     nwk_manager.c
 * \brief    802.15.4 MAC management module.
 *
 *  802.15.4 MAC management module: handler functions.
 *
 */


/* Standard includes. */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifndef NWK_CONFIG_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"
#include "socket.h"
#include "buffer.h"

#include "module.h"
#include "neighbor_routing_table.h"
#include "event_timer.h"
#include "rf.h"
#include "queue.h"
#include "control_message.h"

/*
[NAME]
NWK_MANAGER

[ID]
MODULE_NWK_MANAGER,

[INFO]
#ifdef HAVE_NWK_MANAGER
  {nwk_manager_init, nwk_manager_handle, nwk_manager_check, 0, MODULE_NWK_MANAGER, 0, ADDR_NONE, 0 },
#endif
[FUNCS]*/
extern portCHAR nwk_manager_init( buffer_t *buf);
extern portCHAR nwk_manager_handle( buffer_t *buffer );
extern portCHAR nwk_manager_check( buffer_t *buf );


void nwk_manager_create_ctrl_message_to_mac(buffer_t *buf, mac_control_id_t message_id , uint8_t parameter);
uint8_t cap=0;
nwk_manager_state_t nwk_state;
nwk_manager_pib_t nwk_config_pib;
#ifdef MAC_FFD
child_status_type_t child_status;
#endif



/**
 *  Standard nanostack module initalizer.
 *
 *	\param buf A buffer for compatibility with stack functions
 *  \return  pdTRUE    OK
 */
portCHAR nwk_manager_init( buffer_t *buf )
{
#ifdef MAC_FFD
	nwk_config_pib.child_count=0;
#else
	nwk_config_pib.parent_info.assoc_status = 0;
#endif

	cap = NWK_CAP_DEFAULTS;
	return pdTRUE;
}

#ifndef AD_HOC_STATE
/**
 *  Lauch RFD device Assocation process.
 *
 *  \return  pdTRUE    OK
 */
void nwk_manager_launch(void)
{
	buffer_t *buffer;
	buffer = 0;

	while(buffer == 0)
	{
		buffer = stack_buffer_get(50);
		if(buffer == 0)
		{
			debug("could not get buffer.\r\n");
			vTaskDelay(50 / portTICK_RATE_MS );
		}
	}
	nwk_state = NWK_DISCOVER_STATE;
	nwk_manager_create_ctrl_message_to_mac(buffer, SCAN_REQ, ACTIVE_SCAN);
	stack_buffer_push(buffer);
	buffer=0;	
}
#endif


/**
 *  Standard nanostack buffer handler.
 *
 *
 *	\param   buf      Buffer to process
 *  \return  pdTRUE   Buffer handled by this module
 *  \return  pdFALSE	Buffer rejected by module
 */
portCHAR nwk_manager_handle( buffer_t *buffer )
{
#ifndef AD_HOC_STATE
	control_message_t *ptr=0;
	uint8_t i, list_size=0;
#ifdef MAC_FFD
	uint8_t address[8];
	uint8_t short_address[2];
	uint8_t j;
	channel_ed_t channel_ed;
	uint8_t cap_info;
	channel_ed.ed_value=(-1);
#else /* COORDINATOR */
	uint8_t b_order, s_order;
	uint8_t addr_length;
#endif /* COORDINATOR */

	if(buffer->options.type == BUFFER_CONTROL)
	{
		ptr = ( control_message_t*) buffer->buf;
		switch (nwk_state)
		{
#ifndef MAC_FFD
			case NWK_DISCOVER_STATE:
				if(ptr->message.mac_control.message_id == SCAN_CONFIRM)
				{
					uint8_t selected_net = list_size;

					list_size = ptr->message.mac_control.message.scan_confirm.resultlist_size;
					debug_int(list_size);
					debug(" response count\r\n");
					if(list_size)
					{
						uint8_t lqi = 0;
						for(i=0; i<list_size; i++)
						{
							if((ptr->message.mac_control.message.scan_confirm.pan_descriptor[i].assoc_permit == 1) && (lqi < ptr->message.mac_control.message.scan_confirm.pan_descriptor[i].link_quality))
							{
								selected_net= i;
								lqi = ptr->message.mac_control.message.scan_confirm.pan_descriptor[i].link_quality;
							}
						}
					}
					if (selected_net < list_size)
					{
						nwk_config_pib.parent_info.channel  = ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].logical_channel;
						nwk_config_pib.parent_info.address_type = ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].coord_addr_mode;
						if(nwk_config_pib.parent_info.address_type == ADDR_MODE_16)
						{
							addr_length=2;
						}
						else
						{
							addr_length=8;
						}
						for(i=0; i< addr_length ; i++)
						{
							if(addr_length==8)
							{
								nwk_config_pib.parent_info.long_address[i] = ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].address[i];
							}
							else
							{
								nwk_config_pib.parent_info.short_address[i] = ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].address[i];
							}
						}
						for(i=0; i< 2 ; i++)
						{
							nwk_config_pib.parent_info.pan_id[i] = ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].panid[i];
						}
						b_order = (ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].superframe_spec[0] & 0x0f);
						s_order = (ptr->message.mac_control.message.scan_confirm.pan_descriptor[selected_net].superframe_spec[0] & 0xf0);
						s_order = (s_order >> 4);
						nwk_config_pib.parent_info.beacon_order = b_order;
						nwk_config_pib.parent_info.superframe_order = s_order;
						nwk_state = NWK_ASSOC_STATE;
						nwk_manager_create_ctrl_message_to_mac(buffer, ASSOC_REQ, nwk_config_pib.parent_info.channel);
						stack_buffer_push(buffer);
						buffer=0;
					}
					else
					{
						stack_buffer_free(buffer);
						buffer=0;
						rf_rx_disable();
						vTaskDelay(15000 / portTICK_RATE_MS );
						rf_rx_enable();
						nwk_manager_launch();
					}
				}
				if(buffer)
				{
					stack_buffer_free(buffer);
					buffer=0;
				}
				break;

			case NWK_ASSOC_STATE:
				if(ptr->message.mac_control.message_id == ASSOC_CONFIRM)
				{
					if( ptr->message.mac_control.message.assoc_confirm == ASSOC_SUCCESSFUL )
					{
						nwk_state = CLIENT_STATE;
						nwk_config_pib.parent_info.assoc_status = 1;
						if(nwk_config_pib.parent_info.beacon_order != 15)
						{
							nwk_manager_create_ctrl_message_to_mac(buffer, SYNCH_REQ, 0);
							stack_buffer_push(buffer);
							buffer=0;
						}
						if(buffer)
						{
							ptr = ( control_message_t*) buffer->buf;
							ptr->message.ip_control.message_id = ASSOCIATION_WITH_COORDINATOR;
							push_to_app(buffer);
							buffer=0;
						}
					}
					if( ptr->message.mac_control.message.assoc_confirm != ASSOC_SUCCESSFUL && buffer )
					{
						stack_buffer_free(buffer);
						buffer=0;
						rf_rx_disable();
						vTaskDelay(15000 / portTICK_RATE_MS );
						rf_rx_enable();
						nwk_manager_launch();
					}
				
				}
				if(ptr->message.mac_control.message_id == SYNCH_LOSS_IND)
				{
					switch (ptr->message.mac_control.message.synch_loss_reason.synch_lost_reason)
					{
						case NO_ACK_FROM_COORD:
							stack_buffer_free(buffer);
							buffer=0;
							nwk_config_pib.parent_info.assoc_status = 0;
							nwk_manager_launch();
							break;
						default:
							stack_buffer_free(buffer);
							buffer=0;
							break;
					}
				}

				if(buffer)
				{
					stack_buffer_free(buffer);
					buffer=0;
				}
				break;
			case CLIENT_STATE:
				/* There will come some control messgaes*/
				if(ptr->message.mac_control.message_id == SYNCH_LOSS_IND)
				{
					switch (ptr->message.mac_control.message.synch_loss_reason.synch_lost_reason)
					{
						case BEACON_LOST:
							nwk_manager_create_ctrl_message_to_mac(buffer, SCAN_REQ, ORPHAN_SCAN);
							stack_buffer_push(buffer);
							buffer=0;
							break;

						case REALIGMENT:
							nwk_manager_create_ctrl_message_to_mac(buffer, SYNCH_REQ, 0);
							stack_buffer_push(buffer);
							buffer=0;
							break;

						case PAN_ID_CONFLIGTH:
							stack_buffer_free(buffer);
							buffer=0;
							break;

						case NO_ACK_FROM_COORD:
							stack_buffer_free(buffer);
							buffer=0;
							nwk_config_pib.parent_info.assoc_status = 0;
							nwk_manager_launch();
							break;

					}
				}
				if(ptr->message.mac_control.message_id == SCAN_CONFIRM && ptr->message.mac_control.message.scan_confirm.scan_type == ORPHAN_SCAN)
				{
					nwk_config_pib.parent_info.assoc_status = 0;
					ptr = ( control_message_t*) buffer->buf;
					ptr->message.ip_control.message_id = ASSOCIATION_LOST;
					push_to_app(buffer);
					buffer=0;
					nwk_manager_launch();
				}
				if(buffer)
				{
					stack_buffer_free(buffer);
					buffer=0;
				}
				break;
#endif
#ifdef MAC_FFD
			case NWK_FORMATION_STATE:
				if(ptr->message.mac_control.message_id == SCAN_CONFIRM)
				{
					list_size = ptr->message.mac_control.message.scan_confirm.resultlist_size;
					j=11;
					for(i=0; i< list_size;i++)
					{
						if(ptr->message.mac_control.message.scan_confirm.ed_list[i] < channel_ed.ed_value)
						{
							channel_ed.ed_value=ptr->message.mac_control.message.scan_confirm.ed_list[i];
							channel_ed.channel =j;
						}
						j++;
						
					}

					nwk_state = CORD_STATE;					
					nwk_manager_create_ctrl_message_to_mac(buffer, START_REQ, channel_ed.channel);
					stack_buffer_push(buffer);
					buffer=0;
				}
				if(ptr->message.mac_control.message_id != SCAN_CONFIRM && buffer)
				{
					stack_buffer_free(buffer);
					buffer=0;
					vTaskDelay(15000 / portTICK_RATE_MS );
					nwk_manager_launch();
				}
				break;

			case CORD_STATE:
				if(ptr->message.mac_control.message_id == ASSOC_IND)
				{
					uint8_t status=0;
					for(i=0; i< 8; i++)
					{
						 address[i] = ptr->message.mac_control.message.assoc_ind.device_address[i];
					}
					cap_info = ptr->message.mac_control.message.assoc_ind.cap_info;
					/* Check if already child */	
					address[2] = nwk_config_pib.pan_id[0];
					address[3] = nwk_config_pib.pan_id[1];
					
					j=0;
					child_status = NOT_CHILD;
					child_status = check_child_role(ADDR_802_15_4_PAN_SHORT, address);
					switch (child_status)
					{
						case NOT_CHILD:
						case NO_CAPASITY_AFTER_NEW_CHILD:
							nwk_config_pib.child_count++;
							update_neighbour_table(ADDR_802_15_4_PAN_SHORT, address,-30, 0, ADD_CHILD);
						case CHILD:
							status = ASSOC_SUCCESSFUL;
							break;

						case DISCARD_ASSOC:
							status = PAN_AT_CAPACITY;
							break;

						default:
							break;
					}

					/* Update */
					address[2] =ptr->message.mac_control.message.assoc_ind.device_address[2];
					address[3] =ptr->message.mac_control.message.assoc_ind.device_address[3];
					buffer->buf_ptr=0;
					buffer->buf_end=4;
					buffer->options.type = BUFFER_DATA;
					short_address[0] = address[0];
					short_address[1] = address[1];
					j=0;
					buffer->buf[j++] = CMD_ASSOC_RESPONSE;
					for(i=0; i<2; i++)
					{
						buffer->buf[j++] = short_address[i];
					}

					if(child_status== NO_CAPASITY_AFTER_NEW_CHILD || child_status == DISCARD_ASSOC)
					{
						mac_assoc_permit_false();
					}
					buffer->buf[j++] = status;
					for(i=0; i<8; i++)
					{
						buffer->dst_sa.address[i] = address[i];
					}
					buffer->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
					buffer->from = MODULE_NWK_MANAGER;
					if( mac_pending_req(buffer) != pdTRUE)
					{
						debug("Pend fail\r\n");
					}
					buffer=0;
				}

				if(ptr->message.mac_control.message_id == ORPHAN_IND && buffer)
				{
					/* Check is device already my child */
					uint8_t check = 0;
					for(i=0; i<8; i++)
					{
						address[i]=ptr->message.mac_control.message.orphan_ind.orphan_address[i];
					}

					if(nwk_config_pib.child_count)
					{
#ifdef HAVE_ROUTING
						if(check_child_role( ADDR_802_15_4_SHORT, address) == CHILD)
						{
							check=1;
						}
#endif
					}
					
					if(check==0)
					{
						stack_buffer_free(buffer);
						buffer=0;
					}
					else
					{
						for(i=0; i<8; i++)
						{
							buffer->dst_sa.address[i] = ptr->message.mac_control.message.orphan_ind.orphan_address[i];
						}
						buffer->dst_sa.addr_type = ADDR_802_15_4_PAN_LONG;
						buffer->src_sa.addr_type = ADDR_NONE;
						nwk_manager_create_ctrl_message_to_mac(buffer, ORPHAN_RESPONSE, 0);
						stack_buffer_push(buffer);
						buffer=0;
					}
				}
				if(buffer)
				{
					stack_buffer_free(buffer);
					buffer=0;
				}
				break;
#endif
			default:
				stack_buffer_free(buffer);
				buffer=0;
				break;
		}
	}
#endif /*AD_HOC_STATE*/
	if(buffer)
	{
		stack_buffer_free(buffer);
		buffer=0;
	}

return pdTRUE;
}

/**
 *  Standard nanostack buffer checker.
 *
 *	\brief This function returns _always_ pdFALSE.
 *
 *	\param   buf      Buffer to process
 *  \return  pdTRUE   Buffer passed module check
 *  \return  pdFALSE	Buffer rejected by module
 */
portCHAR nwk_manager_check( buffer_t *buf )
{
	return pdFALSE;  
}

#ifndef AD_HOC_STATE
#ifdef MAC_FFD
/**
 *	Helper function for setting PAN ID
 *
 *	\param	pointer	A pointer to an array of two elements of type uint8_t containing the PAN ID
 *
 */
void nwk_manager_set_pan_id(uint8_t *pointer)
{
	nwk_config_pib.pan_id[0]=pointer[0];
	nwk_config_pib.pan_id[1]=pointer[1];
}
#endif
#endif /*AD_HOC_STATE*/

#ifndef AD_HOC_STATE

/**
 *	Build and send control messages to MAC module.
 *
 *	\brief	This function creates a control message acoording to the parameters and sends it to the MAC module 
 *
 *  \param  buf	Pointer for data structure.
 *  \param  message_id	Message type
 *  \param parameter	Setup value: logical channel for START_REQ and ASSOC_REQ, scan type for SCAN_REQ
 */
void nwk_manager_create_ctrl_message_to_mac(buffer_t *buf, mac_control_id_t message_id , uint8_t parameter)
{
	uint8_t i;
	control_message_t *ptr;
	buf->socket = 0;
	buf->options.type = BUFFER_CONTROL;
	buf->to 		= MODULE_RF_802_15_4;
	buf->from 	= MODULE_NWK_MANAGER;
	buf->dir		= BUFFER_DOWN;
	ptr = ( control_message_t*) buf->buf;
	ptr->message.mac_control.message_id = message_id;
	switch(message_id)
	{
#ifdef MAC_FFD
		case ORPHAN_RESPONSE:
			ptr->message.mac_control.message.orphan_response.short_address[0] = buf->dst_sa.address[0];
			ptr->message.mac_control.message.orphan_response.short_address[1] = buf->dst_sa.address[1];
			ptr->message.mac_control.message.orphan_response.assoc_member = TRUE;
			for(i=0; i<8; i++)
			{
				ptr->message.mac_control.message.orphan_response.orphan_address[i] = buf->dst_sa.address[i];
			}
			break;

		case START_REQ:
			ptr->message.mac_control.message.start_req.panid[0] = nwk_config_pib.pan_id[0];
			ptr->message.mac_control.message.start_req.panid[1] = nwk_config_pib.pan_id[1];
			ptr->message.mac_control.message.start_req.logical_channel = parameter;
#ifdef SUPERFRAME_MODE
			ptr->message.mac_control.message.start_req.beacon_order = BEACON_ORDER;
			ptr->message.mac_control.message.start_req.superframe_order = SUPERFRAME_ORDER;
#else
			ptr->message.mac_control.message.start_req.beacon_order = 15;
			ptr->message.mac_control.message.start_req.superframe_order = 15;
#endif
			ptr->message.mac_control.message.start_req.pan_cordinator =TRUE;
			ptr->message.mac_control.message.start_req.batt_life_ext = FALSE;
			ptr->message.mac_control.message.start_req.cord_realigment = FALSE;
			ptr->message.mac_control.message.start_req.security_enable = FALSE;
			break;
#else
		case SYNCH_REQ:
			ptr->message.mac_control.message.synch_req.logical_channel = nwk_config_pib.parent_info.channel;
			ptr->message.mac_control.message.synch_req.b_order= nwk_config_pib.parent_info.beacon_order;
			ptr->message.mac_control.message.synch_req.s_order= nwk_config_pib.parent_info.superframe_order;
			rf_802_15_4_slot_timing(nwk_config_pib.parent_info.beacon_order,
			&(ptr->message.mac_control.message.synch_req.synch_timer),
			&(ptr->message.mac_control.message.synch_req.last_slot_synch)
			);
			break;

		case ASSOC_REQ:
				ptr->message.mac_control.message.assoc_req.logical_channel = parameter;
				ptr->message.mac_control.message.assoc_req.cord_addrmode = nwk_config_pib.parent_info.address_type;
				if(nwk_config_pib.parent_info.address_type==ADDR_MODE_16)
				{
					for(i=0; i< 2 ; i++)
					{
						ptr->message.mac_control.message.assoc_req.coord_address[i]= nwk_config_pib.parent_info.short_address[i];
					}
				}
				else
				{
					for(i=0; i< 8 ; i++)
					{
						ptr->message.mac_control.message.assoc_req.coord_address[i]= nwk_config_pib.parent_info.long_address[i];
					}
				}
				
				for(i=0; i< 2 ; i++)
				{
					ptr->message.mac_control.message.assoc_req.cord_panid[i]=nwk_config_pib.parent_info.pan_id[i];
				}
				ptr->message.mac_control.message.assoc_req.cap_info = cap;
				ptr->message.mac_control.message.assoc_req.security_enable = FALSE;	
			break;

#endif
		case SCAN_REQ:
			ptr->message.mac_control.message.scan_req.scan_type = parameter;
			ptr->message.mac_control.message.scan_req.scan_channels = SCAN_ALL;
			ptr->message.mac_control.message.scan_req.scan_duration = 8;
			break;
		default:

			break;	
	}
}
#endif /*AD_HOC_STATE*/
