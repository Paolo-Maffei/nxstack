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
 * \file     ssi.c
 * \brief    SSI protocol module.
 *
 *  SSI protocol module: SSI sensor data server.
 *	Module usage is covered in examples micro_ssi and
 *  micro_compass.
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#ifndef SSI_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"

#include "stack.h"
#include "buffer.h"

#include "module.h"
#include "ssi.h"
#include "event_timer.h"

/*
[NAME]
SSI

[ID]
MODULE_SSI,

[INFO]
#ifdef HAVE_SSI
  {ssi_init, ssi_handle, ssi_check, 0, MODULE_SSI, 0, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR ssi_init(buffer_t *buf);
extern portCHAR ssi_handle( buffer_t *buf );
extern portCHAR ssi_check( buffer_t *buf );


/* module internals */

extern ssi_sensor_t ssi_sensor[];	/*defined by application*/
extern uint8_t *ssi_description[];	/*defined by application*/
extern uint8_t *ssi_unit[];		/*defined by application*/

extern uint8_t ssi_n_sensors;

void ssi_update(void *param);

uint8_t ssi_update_posted;
uint8_t ssi_timer_id;

void ssi_discovery(void *pbuf);
void ssi_data_request(void *pbuf);


/**
 *  Initialize SSI module.
 *
 *  \return  pdTRUE    OK
 */
portCHAR ssi_init(buffer_t *buf)
{
/*	ssi_n_sensors = sizeof(ssi_sensor)/sizeof(ssi_sensor_t);*/
	ssi_timer_id = evtTimerAlloc(events, sizeof(event_t));

	ssi_update_posted = 0;
	return pdTRUE;
}

/**
 *  SSI buffer handler.
 *
 *	\param   buf       pointer to buffer
 *  \return  pdTRUE    OK
 */
portCHAR ssi_handle(buffer_t *buf)
{
	uint8_t b_ind;
	ssi_cmd_t cmd;
	if (buf->dir == BUFFER_UP)
	{	/*receive*/
		b_ind = buf->buf_ptr;
		
		if ((buf->buf[b_ind] == 0) || (buf->buf[b_ind] == SSI_ADDRESS_ALL)) /*It's for me*/
		{ b_ind++;
/*			if (buf->buf[b_ind] & SSI_CRC_MASK) use_crc = 1;*/
			cmd = (ssi_cmd_t) (buf->buf[b_ind++] & 0x5F);
			switch(cmd)
			{
				case SSI_CMD_QUERY:
					debug("SSI: query\r\n");
					b_ind = buf->buf_ptr;
					buf->socket = 0;
					buf->from = MODULE_SSI;
					buf->dir = BUFFER_DOWN;
					buf->to = MODULE_NONE;
					{	/*swap addresses*/
						sockaddr_t tmp_addr;
						
						memcpy(&(tmp_addr), &(buf->src_sa), sizeof(sockaddr_t));
						memcpy(&(buf->src_sa), &(buf->dst_sa), sizeof(sockaddr_t));
						memcpy(&(buf->dst_sa), &(tmp_addr), sizeof(sockaddr_t));
					}
					cmd = SSI_CMD_QUERY_REPLY;
					buf->buf[b_ind++] = 0; /*my address*/
					buf->buf[b_ind++] = (uint8_t) cmd; /*discovery reply*/
					buf->buf[b_ind++] = SSI_VERSION_MAJOR;
					buf->buf[b_ind++] = SSI_VERSION_MINOR;
					buf->buf[b_ind++] = BUFFER_SIZE >> 8; /*buf->buffer size*/
					buf->buf[b_ind++] = (uint8_t) BUFFER_SIZE;
					buf->buf[b_ind++] = 0; /*reserved*/
					buf->buf[b_ind++] = 0; /*reserved*/
					buf->buf[b_ind++] = 0; /*reserved*/
					buf->buf[b_ind++] = 0; /*reserved*/
					
					buf->buf_end = b_ind;
					stack_buffer_push(buf);
					break;
					
				case SSI_CMD_DISCOVERY:
					debug("SSI: discovery.\r\n");
					{
						event_t event;
						event.process = ssi_discovery;
						event.param = (void *) buf;
						buf->options.hop_count = 5;/* only one hops */
						buf->buf[buf->buf_end] = 0;
						xQueueSend( events, ( void * ) &event, ( portTickType ) 0 );
					}
					break; /*end discovery handling*/
				
				case SSI_CMD_DATA_REQUEST:
					debug("SSI: request");
					{
						event_t event;
						event.process = ssi_data_request;
						event.param = (void *) buf;

						buf->buf_ptr = b_ind;
						buf->buf[buf->buf_end] = buf->buf[b_ind - 1];
						while (b_ind < buf->buf_end)
						{
							uint8_t i;
							uint16_t id = buf->buf[b_ind++];
							
							id <<= 8;
							id += buf->buf[b_ind++];
							for (i=0; i< ssi_n_sensors; i++)
							{
								if (id == ssi_sensor[i].id)
								{
									ssi_sensor[i].status |= SSI_STATUS_REQUESTED;
									ssi_sensor[i].status &= ~SSI_STATUS_UPDATED;
								}
							}
						}
						xQueueSend( events, ( void * ) &event, ( portTickType ) 0 );
					}
					break;
					
#ifdef SSI_HAVE_CONFIG
				case SSI_CMD_SET_CONFIGURATION_DATA:
					debug("SSI: config");
					{
						uint8_t *ptr;
						uint8_t *end;
						uint8_t attr_type = 0;
						uint8_t val_type = 0;
						uint8_t size;
						uint16_t sensor_id;
						int8_t error = 0;
						ssi_attr_t attr;
						
/*						event_t event;
						
						event.process = ssi_data_request;
						event.param = (void *) buf;*/

						buf->buf_ptr = b_ind;
						ptr = buffer_data_pointer(buf);
						end = buffer_data_end(buf);
						
						while (ptr < end)
						{
							sensor_id = *ptr++;
							sensor_id <<= 8;
							sensor_id = *ptr++;

							attr_type = *ptr >> 4;
							val_type = *ptr++ & 0x0F;						

							attr.attr_type = attr_type;
							attr.value_type = val_type;
							
							size = ssi_attr_size(attr_type);

							if (size > 4)
							{ error = -1;
							}
							else
							{
								memcpy(attr.attr, ptr, size);
							}
							ptr += size;
							
							size = ssi_attr_size(val_type);

							if (size > 4)
							{ error = -1;
							}
							else
							{
								memcpy(attr.value, ptr, size);
							}
							ptr += size;
							if (!error)
							{
								error = ssi_sensor_config(sensor_id, &attr);
								if (!error)
								{
									ssi_pack_response(sensor_id, attr);
								}
								else
								{
									ssi_pack_error(sensor_id, attr);
								}
							}
						}
					}
					break;					
#endif						
				default:
					debug("SSI: Unknown command.\r\n");
					stack_buffer_free(buf);
					break;
			}
		}
		else
		{
			stack_buffer_free(buf);
		}
	}
	return pdFALSE;
}

/**
 *  SSI buffer check.
 *
 *	\param   buf       pointer to buffer
 *
 *  \return  pdTRUE     OK
 *  \return  pdFALSE    OK
 */
portCHAR ssi_check(buffer_t *buf)
{
#ifdef HAVE_NUDP
	if ((buf->from == MODULE_NUDP) && (buf->dir == BUFFER_UP))
	{
		if (buf->dst_sa.port == 40)
		{
			return pdTRUE;
		}
	}
#endif
#ifdef HAVE_CUDP
	if ((buf->from == MODULE_CUDP) && (buf->dir == BUFFER_UP))
	{
		if (buf->dst_sa.port == 40)
		{
			return pdTRUE;
		}
	}
#endif
	return pdFALSE;
}

/**
 *  SSI value update event check.
 *
 *	\param   param     not used
 *
 */
void ssi_update(void *param)
{
	ssi_update_posted = 0;
}

/*
 *  SSI sensor value update.
 *
 *	\param   ind       nr of sensor (in array, not ID)
 *  \param   value     new value
 *
 */
void ssi_sensor_update(uint8_t ind, uint32_t value)
{
	if (ind < ssi_n_sensors)
	{
		ssi_sensor[ind].sdata.i = value;
		ssi_sensor[ind].status |= SSI_STATUS_UPDATED;
		ssi_sensor[ind].status &= ~SSI_STATUS_REQUESTED;
		if (ssi_update_posted == 0)
		{
			event_t event;
			event.process = &ssi_update;

			event.param = (void *) 0;
			ssi_update_posted = 1;
			xQueueSend(events, &event, 20);
			
		}
	}
}

/*
 *  SSI sensor value update from ISR.
 *
 *	\param   ind       nr of sensor (in array, not ID)
 *  \param   value     new value
 *
 */
void ssi_sensor_update_from_ISR(uint8_t ind, uint32_t value)
{
	if (ind < ssi_n_sensors)
	{
		ssi_sensor[ind].sdata.i = value;
		ssi_sensor[ind].status |= 0x80;
		if (ssi_update_posted == 0)
		{
			event_t event;
			event.process = &ssi_update;

			event.param = (void *) 0;
			ssi_update_posted = 1;
			xQueueSendFromISR(events, &event, pdFALSE);
		}
	}
}

/**
 *  SSI discovery handler.
 *
 *	\param   pbuf       pointer to buffer
 *  \return  pdTRUE    OK
 */
void ssi_discovery(void *pbuf)
{
	buffer_t *buf;
	buffer_t *request = (buffer_t *) pbuf;
	uint8_t i;
	
	buf = stack_buffer_get(0);
	if (buf == 0)
	{
		event_t event;
		
		debug("SSI: Can't get buffer\r\n");
		event.process = ssi_discovery;
		event.param = (void *) request;

		evtTimerStart(ssi_timer_id, (uint8_t *) &event,	10 / portTICK_RATE_MS );

		return;
	}

	buf->buf_ptr = request->buf_ptr;	/*get header space*/
	buf->buf_end = request->buf_ptr;
	
	buffer_push_uint8(buf, 0x00); /*address*/
	buffer_push_uint8(buf, SSI_CMD_DISCOVERY_REPLY); /*command*/

	buf->src_sa.addr_type = ADDR_NONE;
#ifdef HAVE_NUDP
	buf->src_sa.port = 40;
#endif
#ifdef HAVE_CUDP
	buf->src_sa.port = 40;
#endif
	memcpy(&(buf->dst_sa), &(request->src_sa), sizeof(sockaddr_t));

	buf->from = MODULE_SSI;
	buf->dir = BUFFER_DOWN;
	buf->to = MODULE_NONE;
	
	i=request->buf[request->buf_end];

	while (buf && ( buffer_data_free(buf) > 34) && (i < ssi_n_sensors) )
	{
		debug("SSI: Creating answer...\r\n");

		buffer_push_uint8(buf, ssi_sensor[i].id >> 8); /*fill in*/
		buffer_push_uint8(buf, ssi_sensor[i].id); /*the sensor id*/

		memset(buffer_data_end(buf), 0, 24); /*fill description and unit w/ zero*/
		strncpy(buffer_data_end(buf), ssi_description[i], 16);
		buf->buf_end += 16;
		strncpy(buffer_data_end(buf), ssi_unit[i], 8);
		buf->buf_end += 8;
		buffer_push_uint8(buf, ssi_sensor[i].unit); /*fill in data type*/
		buffer_push_uint8(buf, ssi_sensor[i].scaler); /*fill in scaler*/

		memset(buffer_data_end(buf), 0, 4); /*min w/ zero*/
		buf->buf_end += 4;
		memset(buffer_data_end(buf), 0 ,4); /*max w/ zero*/
		buf->buf_end += 4;

		i++;
	}	/*end of sensor loop*/
	if ( (buf) && (buffer_data_free(buf) >= 2) && (i == (ssi_n_sensors -1)) )
	{
		debug("SSI: Pushing buffer...\r\n");
		buffer_push_uint8(buf, SSI_INS_END); /*end*/
		buffer_push_uint8(buf, SSI_INS_END); /*mark*/
		stack_buffer_push(buf);
		buf = 0;
		stack_buffer_free(request);
		request = 0;
	}
	
	if (request)
	{
		event_t event;
		
		debug("SSI: Request...\r\n");
		event.process = ssi_discovery;
		event.param = (void *) request;

		request->buf[request->buf_end] = i;
		
		evtTimerStart(ssi_timer_id, (uint8_t *) &event,	120 / portTICK_RATE_MS );
		/*xQueueSend( events, ( void * ) &event, ( portTickType ) 0 );*/
		if (buf)
		{
			stack_buffer_push(buf);
		}
		buf = 0;
	}
}

/**
 *  SSI data request handler.
 *
 *	\param   pbuf       pointer to buffer
 */
void ssi_data_request(void *pbuf)
{
	buffer_t *buf;
	buffer_t *request = (buffer_t *) pbuf;
	uint8_t i;
	uint16_t b_ind;
	uint8_t ready = 1;
	
	b_ind = request->buf_ptr;
	while (b_ind < request->buf_end)
	{
		uint16_t id = request->buf[b_ind++];
		id <<= 8;
		id += request->buf[b_ind++];
		for (i=0; i< ssi_n_sensors; i++)
		{
			if (id == ssi_sensor[i].id)
			{
				if ((ssi_sensor[i].status & SSI_STATUS_UPDATED) == 0)
				{
					ready = 0;
				}
			}
			
		}
	}
	
	if (ready) buf = stack_buffer_get(0);
	else buf = 0;
	
	if (buf == 0)
	{
		event_t event;
		event.process = ssi_data_request;
		event.param = (void *) request;

		evtTimerStart(ssi_timer_id, (uint8_t *) &event,	1 );
		return;
	}
	buf->buf_ptr = request->buf_ptr;	/*get header space*/
	buf->buf_end = request->buf_ptr;
	
	buffer_push_uint8(buf, 0x00); /*address*/
	buffer_push_uint8(buf, SSI_CMD_DATA_RESPONSE_NO_STATUS); /*command*/

	buf->src_sa.addr_type = ADDR_NONE;
#ifdef HAVE_NUDP
	buf->src_sa.port = 40;
#endif
#ifdef HAVE_CUDP
	buf->src_sa.port = 40;
#endif
	memcpy(&(buf->dst_sa), &(request->src_sa), sizeof(sockaddr_t));

	buf->from = MODULE_SSI;
	buf->dir = BUFFER_DOWN;
	buf->to = MODULE_NONE;
	while ((buffer_data_length(request) > 1) && (buffer_data_free(buf) > 6))
	{
		uint16_t id = buffer_pull_uint8(request);
		id <<=8;
		id += buffer_pull_uint8(request);
		
		for (i=0; i< ssi_n_sensors; i++)
		{
			if (ssi_sensor[i].id == id) break;
		}
		if (i < ssi_n_sensors)
		{
			uint32_t value = ssi_sensor[i].sdata.i;
			buffer_push_uint8(buf, (id >> 8));
			buffer_push_uint8(buf, id);
			buffer_push_uint8(buf, (value >> 24));
			buffer_push_uint8(buf, (value >> 16));
			buffer_push_uint8(buf, (value >> 8));
			buffer_push_uint8(buf, (value));
			ssi_sensor[i].status &= SSI_STATUS_UPDATED; 
		}
	}
	debug("-> reply.\r\n");	
	stack_buffer_push(buf);
	if (buffer_data_length(request) > 1)
	{
		event_t event;
		event.process = ssi_data_request;
		event.param = (void *) request;
		debug("Push req.\r\n");
		evtTimerStart(ssi_timer_id, (uint8_t *) &event,	120 / portTICK_RATE_MS );
	}
	else
	{
		stack_buffer_free(request);
	}
}

#ifdef SSI_HAVE_CONFIG
uint8_t ssi_attr_size(uint8_t type)
{
	if ((type >= SSI_ATTR_ASCII_1) && (type <= SSI_ATTR_ASCII_32))
	{
		return (1 << (type - 1));
	}
	else if ((type >= SSI_ATTR_INTD_1) && (type <= SSI_ATTR_INTD_1000000))
	{
		return 2;
	}
	else if (type == SSI_ATTR_FLOAT)
	{
		return 4;
	}
	return 0;
}
#endif
