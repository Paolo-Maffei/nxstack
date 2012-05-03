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
 * \file stack.c
 * \brief Protocol stack main module.
 *
 *  Protocol stack main module: handling of the buffer/event queue,
 *  buffer management
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <sys/inttypes.h>

#include "stack.h"
#include "buffer.h"
#include "socket.h"
#include "module.h"

#ifndef STACK_DEBUG
#undef HAVE_DEBUG
#endif
#include "debug.h"

#include "control_message.h"
#include "rf.h"
/*
#ifndef STACK_TASK_SIZE
#ifdef SDCC_CC2430
#define STACK_TASK_SIZE configMAXIMUM_STACK_SIZE
#else
#define STACK_TASK_SIZE 300
#endif
#endif
*/
#ifndef STACK_PRIORITY
#define STACK_PRIORITY (tskIDLE_PRIORITY + 2)
#endif

#ifndef STACK_BUFFERS_MAX
#define STACK_BUFFERS_MAX 8
#endif

#ifndef STACK_BUFFERS_MIN
#define STACK_BUFFERS_MIN 4
#endif 

/** Common variables */
xQueueHandle     buffers; 
xQueueHandle     events = 0; 

extern socket_t  sockets[];
extern module_t modules[];
extern nwk_manager_state_t nwk_state;
extern sockaddr_t mac_long;

uint8_t n_buffers;
#ifdef HAVE_MAC_15_4
stack_t stacks[] =
{
#ifdef HAVE_NRP
	
	{ 2, { MODULE_MAC_15_4, MODULE_NRP }},
#ifdef HAVE_CIPV6
#ifdef HAVE_ICMP
	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_ICMP, MODULE_NRP }},
#endif	/*ICMP*/
#ifdef HAVE_CUDP
	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP}},
#endif	/*CUDP*/
#endif	/*CIPV6*/	
			
#else /*NRP*/

#ifdef HAVE_CIPV6
#ifdef HAVE_ICMP
	{ 3, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif	/*ICMP*/
#ifdef HAVE_CUDP
	{ 3, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP}},
#ifdef HAVE_SSI
 	{ 4, { MODULE_MAC_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_SSI }},
#endif	/*SSI*/
#endif	/*CUDP*/
#endif	/*CIPV6*/

#endif /*NRP*/
	{ 0, { 0 } }
}; 

#else
stack_t stacks[] =
{

#ifdef HAVE_NRP
#ifdef	HAVE_RF_802_15_4
  { 2, { MODULE_RF_802_15_4, MODULE_NRP }},
#ifdef HAVE_NUDP	
  { 3, { MODULE_RF_802_15_4, MODULE_NUDP, MODULE_NRP}},
#endif
#endif

#ifdef	HAVE_802_15_4_RAW
  { 2, { MODULE_802_15_4_RAW, MODULE_NRP }},
#ifdef HAVE_NUDP	
  { 3, { MODULE_802_15_4_RAW, MODULE_NUDP, MODULE_NRP}},
#endif
#endif

#ifdef HAVE_CUDP
#ifdef HAVE_CIPV6
#ifdef	HAVE_RF_802_15_4
  	{ 4, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP }},
#endif	/*RF802_15_4*/
#ifdef HAVE_ICMP
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif
#ifdef	HAVE_802_15_4_RAW
  	{ 4, { MODULE_802_15_4_RAW, MODULE_CIPV6, MODULE_CUDP, MODULE_NRP }},
#endif	/*802_15_4_RAW*/
#endif /* HAVE_CIPV6 */
#endif /* HAVE_CUDP */

#else /*HAVE_NRP*/



#ifdef HAVE_CUDP
#ifdef HAVE_CIPV6
#ifdef	HAVE_RF_802_15_4
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP}},
#ifdef HAVE_ICMP
	{ 3, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_ICMP }},
#endif
#ifdef HAVE_SSI
  	{ 4, { MODULE_RF_802_15_4, MODULE_CIPV6, MODULE_CUDP, MODULE_SSI }},
#endif
#endif	/*RF802_15_4*/
#ifdef	HAVE_802_15_4_RAW
  	{ 3, { MODULE_802_15_4_RAW, MODULE_CIPV6, MODULE_CUDP }},
#endif	/*802_15_4_RAW*/

#endif /* HAVE_CIPV6 */
#endif /* HAVE_CUDP */

#ifdef HAVE_RS232  
#ifdef HAVE_NUDP	
  { 2, { MODULE_RS232, MODULE_NUDP }},
#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_RS232, MODULE_NTCP }},
#endif
#endif	/*RS232*/

#ifdef HAVE_ETH  
#ifdef HAVE_NUDP	
  { 2, { MODULE_ETH, MODULE_NUDP }},
#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_ETH, MODULE_NTCP }},
#endif
#endif  /*ETH*/

#ifdef HAVE_802_15_4_RAW  
#ifdef HAVE_NUDP	
  { 2, { MODULE_802_15_4_RAW, MODULE_NUDP }},
#ifdef HAVE_SSI
  { 3, { MODULE_802_15_4_RAW, MODULE_NUDP, MODULE_SSI }},
#endif	/*HAVE_SSI*/

#endif
#ifdef HAVE_NTCP			  
  { 2, { MODULE_802_15_4_RAW, MODULE_NTCP }},
#endif

#endif	/*RF802_15_4_RAW*/

#ifdef HAVE_RF_802_15_4  
#ifdef HAVE_NUDP	
  { 2, { MODULE_RF_802_15_4, MODULE_NUDP }},
#endif


#ifdef HAVE_NTCP			  
  { 2, { MODULE_RF_802_15_4, MODULE_NTCP }},
#endif
  
#endif	/*RF802_15_4*/

#endif	/*NRP*/
	{ 0, { 0 } }
}; 
#endif

#define STACKS_MAX (sizeof(stacks)/sizeof(stack_t))

/** Internals */
void stack_main ( void *pvParameters );
void stack_buffer(buffer_t *b);

void stack_module_call(uint8_t stack_id, uint8_t layer, buffer_t *b);
portCHAR stack_module_check(uint8_t stack_id, uint8_t layer, buffer_t *b);

/**
 * Initialize protocol stack and start stack task.
 * Allocate initial set of buffers.
 *
 * \return pdTRUE
 * \return pdFALSE	insufficient memory
 */
portCHAR stack_init( void )
{
	uint8_t i;
	buffer_t *b;
	           
  buffers = xQueueCreate( STACK_BUFFERS_MAX, sizeof( buffer_t * ) );
  events = xQueueCreate( STACK_BUFFERS_MAX + 20, sizeof( event_t ) );
	n_buffers = 0;
	
  xTaskCreate( stack_main, "Stack", configMAXIMUM_STACK_SIZE, NULL, STACK_PRIORITY, ( xTaskHandle * )NULL );

  /* Initialize modules */
  debug("Stack: mod inits.\r\n");
	
	i = 0;
	while (i < STACK_BUFFERS_MAX)
	{	/*Allocate a maximum of STACK_BUFFERS_MIN buffers */
		b = pvPortMalloc(sizeof(buffer_t)+BUFFER_SIZE);
		if (b)
		{
  			memset(b, 0, sizeof(buffer_t));
			b->size = BUFFER_SIZE;
			n_buffers++;
  		if (xQueueSend( buffers, ( void * ) &b,
					(portTickType) 0 ) == pdFALSE)
			{ 
				debug("Stack: buffers fail.\r\n");
				return pdFALSE;
			}
		}
		else
		{   
			debug("Stack: buffer allocation failed.\r\n");
			return pdFALSE;
		}
		if (i++ >= STACK_BUFFERS_MIN) i = STACK_BUFFERS_MAX;
	}
	debug_int(uxQueueMessagesWaiting(buffers));
	debug(" buffers available.\r\n"); 

	socket_init();
	{
		sockaddr_t init_addr;				/* Make sure that we have mac defined */
		init_addr.addr_type = ADDR_NONE;
		rf_mac_get(&init_addr);
	}
	return module_init();
}


/**
 * Initialize protocol stack logical type.
 * 
 *\param stack_parameters pointer for start parameters, 0 start stack on the default mode.
 * \return START_SUCCESS when start parameters are valid.
 * \return CHANNEL_NOT_SUPPORTED when given parameter is not valid.
 * \return TYPE_NOT_SUPPORTED when logical type is not supported.
 */
start_status_t stack_start(stack_init_t  *stack_parameters)
{
	#ifndef AD_HOC_STATE
	control_message_t *msg;
	uint8_t i;
	#endif


	uint8_t null_pointer_used=0;
	portCHAR retval;
	buffer_t *buffer;
	buffer = 0;

	/* Check if stack init is ready called */
	if(events)
	{
		retval = pdTRUE;
	}
	else
	{
		retval = stack_init(); /* Initialize stack core */
	}

	if(stack_parameters==NULL)
	{
		stack_parameters = pvPortMalloc(sizeof(stack_init_t));
		if(stack_parameters)
		{
			stack_parameters->type = DEFAULT_MODE;
	#ifdef PAN_CHANNEL
			stack_parameters->channel = PAN_CHANNEL;
	#else
			stack_parameters->channel = RF_DEFAULT_CHANNEL;
	#endif
			stack_parameters->pending_ttl_time = 5;
			stack_parameters->use_sw_mac = 0;
			null_pointer_used=1;
		}
		else
			return STACK_INIT_FAILED;
	}


	if(retval == pdTRUE)/* Start stack by defined mode */
	{
	#ifdef HAVE_RF_802_15_4
		if(stack_parameters->use_sw_mac == 0)
			mac_set_mac_pib_parameter(mac_long.address, MAC_IEEE_ADDRESS);
		else
			mac_set_mac_pib_parameter(stack_parameters->mac_address, MAC_IEEE_ADDRESS);
	#endif

		switch(stack_parameters->type)
		{
	#ifdef AD_HOC_STATE
			case DEFAULT_MODE:
			case AD_HOC_DEVICE:
	#ifdef HAVE_RF_802_15_4
				mac_handle_address_decoder(RF_DECODER_ON);
				rf_802_15_4_ip_layer_address_mode_set(0);
	#else
				rf_rx_enable();
	#endif
				if(null_pointer_used)
						vPortFree(stack_parameters);
				return START_SUCCESS;
				break;
			default:
				if(null_pointer_used)
					vPortFree(stack_parameters);
				return TYPE_NOT_SUPPORTED;
				break;
	#else
			case DEFAULT_MODE:
			#ifdef MAC_FFD
				stack_parameters->type = BEACON_ENABLE_COORDINATOR;
				for(i=0;i<2;i++)
				{
					stack_parameters->pan_id[i] = mac_long.address[i];
			 		stack_parameters->short_address[i] = mac_long.address[i];
				}
			#else
				stack_parameters->type=BEACON_ENABLE_CLIENT;
			#endif
			default:
				while(buffer == 0)
				{
					buffer = stack_buffer_get(50);
					if(buffer == 0)
					{
						debug("could not get buffer.\r\n");
						vTaskDelay(50 / portTICK_RATE_MS );
					}
				}
				buffer->options.type = BUFFER_CONTROL;
				buffer->socket = 0;
				msg = ( control_message_t*) buffer->buf;
	#ifdef HAVE_RF_802_15_4
				mac_set_mac_pib_parameter(stack_parameters->short_address, MAC_SHORT_ADDRESS);		
	#endif
	#ifdef HAVE_NWK_MANAGER
	#ifdef MAC_FFD

				nwk_manager_set_pan_id(stack_parameters->pan_id);
				if(stack_parameters->type == BEACON_ENABLE_COORDINATOR)
				{
					if(stack_parameters->channel >= 11 || stack_parameters->channel <=26)
					{
	#ifdef HAVE_RF_802_15_4
						mac_set_mac_pib_parameter(&(stack_parameters->pending_ttl_time), PENDING_TTL);
	#endif
	
						msg->message.mac_control.message_id = START_REQ;
						msg->message.mac_control.message.start_req.panid[0] = stack_parameters->pan_id[0];
						msg->message.mac_control.message.start_req.panid[1] = stack_parameters->pan_id[1];
						msg->message.mac_control.message.start_req.logical_channel = stack_parameters->channel;
	#ifdef SUPERFRAME_MODE
						msg->message.mac_control.message.start_req.beacon_order = BEACON_ORDER;
						msg->message.mac_control.message.start_req.superframe_order = SUPERFRAME_ORDER;
	#else
						msg->message.mac_control.message.start_req.beacon_order = 15;
						msg->message.mac_control.message.start_req.superframe_order = 15;
	#endif
						msg->message.mac_control.message.start_req.pan_cordinator =TRUE;
						msg->message.mac_control.message.start_req.batt_life_ext = FALSE;
						msg->message.mac_control.message.start_req.cord_realigment = FALSE;
						msg->message.mac_control.message.start_req.security_enable = FALSE;
						nwk_state = CORD_STATE;
					}
					else
					{
						stack_buffer_free(buffer);
						buffer=0;
						if(null_pointer_used)
							vPortFree(stack_parameters);
						return CHANNEL_NOT_SUPPORTED;
					}	
				}
				
				else if(stack_parameters->type==BEACON_ENABLE_GATEWAY)
				{
					enable_router_features();
					msg->message.mac_control.message_id = ROUTER_START;
					msg->message.mac_control.message.router_start.panid[0] = stack_parameters->pan_id[0];
					msg->message.mac_control.message.router_start.panid[1] = stack_parameters->pan_id[1];
					msg->message.mac_control.message.router_start.logical_channel = stack_parameters->channel;
					nwk_state = ROUTER_STATE;
				}
	#else
				if(stack_parameters->type==BEACON_ENABLE_CLIENT)
				{
					uint8_t temp = stack_parameters->type;
	#ifdef HAVE_RF_802_15_4
					mac_set_mac_pib_parameter(&(temp), RUNNING_MODE);
	#endif
					msg->message.mac_control.message_id = SCAN_REQ;
					msg->message.mac_control.message.scan_req.scan_type = ACTIVE_SCAN;
					msg->message.mac_control.message.scan_req.scan_channels = SCAN_ALL;
					msg->message.mac_control.message.scan_req.scan_duration = 5;	
					nwk_state = NWK_DISCOVER_STATE;	
				}
	#endif
				else
	#endif /*HAVE_NWK_MANAGER*/
				{
					stack_buffer_free(buffer);
					buffer=0;
					if(null_pointer_used)
						vPortFree(stack_parameters);
					return TYPE_NOT_SUPPORTED;
				}
				break;
	#endif /* AD_HOC_STATE */
		}
		if(null_pointer_used)
			vPortFree(stack_parameters);
		buffer->to 		= MODULE_RF_802_15_4;
		buffer->from 	= MODULE_NWK_MANAGER;
		stack_buffer_push(buffer);
		buffer=0;
		return START_SUCCESS;

	}
	else
	{
		if(null_pointer_used)
			vPortFree(stack_parameters);
		return STACK_INIT_FAILED;
	}
}




/**
 * The main stack task
 *
 *	\param pvParameters 	not used
 *
 */
void stack_main ( void *pvParameters )
{
	event_t event;
	portTickType xLastWakeTime;
	
	xLastWakeTime = (portTickType) pvParameters;
	
  xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil( &xLastWakeTime, 200 / portTICK_RATE_MS );
	
	if ( ( buffers == 0 ) || (events == 0) )
	{
		debug("Stack: Initialization failure. Halted.\r\n");
		for(;;)
		{
			vTaskDelayUntil( &xLastWakeTime, 5000 / portTICK_RATE_MS );
		}
	}
	debug("Stack: Go.\r\n");

	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		if (xQueueReceive(events, &( event ), 5000 / portTICK_RATE_MS) == pdTRUE)
		{
			if (event.process == 0)
			{	/*it is a buffer*/
				stack_buffer((buffer_t *) event.param);
			}
			else
			{	/*process event callback*/
				event.process(event.param);
			}
		}	/* end event handling */
		if ((xTaskGetTickCount() - xLastWakeTime)*portTICK_RATE_MS > 10000)
		{
			check_tables_status(0);
			xLastWakeTime = xTaskGetTickCount();
		}
	} /* end task loop */
}


/**
 * Buffer handler
 *
 *	\param b pointer to buffer
 */
void stack_buffer(buffer_t *b)
{
	portCHAR status = pdFAIL;
	uint8_t i;
	uint8_t found = 0;
	socket_t *si = (socket_t *) b->socket;
	
	if (b == 0) return;

	/*if ( ((si != 0) && (si->stack_id == STACKS_MAX)) || (b->options.type == BUFFER_CONTROL) )
	{	// this is a control packet
		debug("\r\n Control message");
		if ((b->to == MODULE_NONE) || (b->to == MODULE_APP))
		{	//socket routing
			switch(b->dir)
			{
				case BUFFER_DOWN:
					debug("\r\n Direction Down");
					if (si) b->to = (module_id_t)si->protocol;
					else
					{
						stack_buffer_free(b);
						b = 0;
					}
					break;
	
				case BUFFER_UP:
					if ((si))
					{
						if(socket_up(b) == pdTRUE)
						{
							debug("\r\n To control socket");
							b = 0;
						}
					}
					else
					{
						if(socket_up(b) == pdTRUE)
						{
							debug("\r\n To control socket");
							b = 0;
						}
						else
						{
							stack_buffer_free(b);
							b = 0;
						}
					}
					break;
			}
		}
		if (b)
		{
			
			if (module_call(b->to, b) == pdFALSE)
			{
				debug("Packet refused by module -> free.\r\n");
				stack_buffer_free(b);               					 
			}
		}
		return;
	}*/
	
	if ((b->to != MODULE_NONE) && (b->to != MODULE_APP))
	{	/*protocol identifier used*/
		debug_printf("buffer to: %d.\r\n", b->to);
		if (module_call(b->to, b) == pdFALSE)
		{
			debug("Packet refused by module -> free.\r\n");
			stack_buffer_free(b);               					 
		}
	}
	else if (b->dir == BUFFER_DOWN)
	{	/*go layer down*/
		debug_printf("down:  from %d to %d.\r\n", b->from, b->to);
		if (si)
		{
			debug_printf("using socket:  stack %d.\r\n", si->stack_id);
			if ((b->from == MODULE_APP) || (b->from == MODULE_NRP))
			{	/*call top layer*/
				b->from = (module_id_t) stacks[si->stack_id].layers;
				stack_module_call(si->stack_id, (b->from) - 1, b);
				b=0;
			}
			else
			{
				i=0;
				while ( (i < stacks[si->stack_id].layers) && b)
				{
					if (stacks[si->stack_id].module[i] == (module_id_t) b->from)
					{
						stack_module_call(si->stack_id, i-1, b);
						b = 0;
					}
					i++;
				}
			}
			if(b)
			{
				debug("Stack: from layer 0! Free buffer.\r\n");
				stack_buffer_free(b);
				b=0;
			}
		}
		else
		{
			i = 0;
			debug("Stack: no socket, scan.\r\n");
			while (b && (stacks[i].layers > 0) )
			{
				uint8_t j = 1;
				
				while ( (j < stacks[i].layers) && b)
				{
					if (stacks[i].module[j] == (module_id_t) b->from)
					{
						debug_printf("Stack: call module %d.\r\n", stacks[i].module[j-1]);
						stack_module_call(i, j-1, b);
						b = 0;
					}
					j++;
				}
				i++;
			}
			
			if (b)
			{
				debug("Stack: no stack found, free buffer.\r\n");
				stack_buffer_free(b);
			}
		}
	}
	else /* if (b->dir == BUFFER_UP)*/
	{	/*one layer up*/
		uint8_t next_layer;
		debug_printf("up:  from %d to %d.\r\n", b->from, b->to);
		i = 0;
		status = socket_up(b);
		if (status != pdTRUE)
		{               					 
			while (b && (stacks[i].layers > 0))
			{
				uint8_t j;
				debug_printf("Scanning stack %d.\r\n", i);
				j=0;
				next_layer = LAYERS_MAX + 1 ;
				while (j < (stacks[i].layers-1))
				{
					debug_printf("Scanning layer %d.\r\n", j);
					if (stacks[i].module[j] == (module_id_t) b->from)
					{
						next_layer = j+1;
						j = stacks[i].layers;
					}
					else j++;
				}
				if (next_layer != (LAYERS_MAX + 1))
				{
					debug_printf("Hit module: %d.\r\n", next_layer);
					status = stack_module_check(i, next_layer, b);
					if (status == pdTRUE)
					{
						stack_module_call(i, next_layer, b);
						b = 0;
					}
				}
				i++;
			}
		}
		
		if (status != pdTRUE)
		{
			if (!found)
			{
				debug("No module found.\r\n");
			}
			else
			{
				debug("Unknown packet(1) -> free.\r\n");
			}
			if (b != 0) stack_buffer_free(b);
		}
		debug("Handler: eject.\r\n");
	}
}

/**
 *  Call module handler. Free buffer if module can't be called.
 *
 *	\param stack_id   stack identifier
 *  \param layer      which layer is to be called
 *  \param b          pointer to buffer
 */
void stack_module_call(uint8_t stack_id, uint8_t layer, buffer_t *b)
{
	uint8_t i;
	uint8_t found = 0;
	if (layer < stacks[stack_id].layers)
	{	/*valid layer*/
		i = 0;
		while(modules[i].id)
		{
			if (modules[i].id == stacks[stack_id].module[layer])
			{
				found = 1;
				modules[i].handle(b);
				break;
			}
			else i++;
		}
		if (!found)
		{
			debug("Invalid module ID.\r\n");
			stack_buffer_free(b);
		}
	}
}

/**
 *  Call module check function. Return pdFALSE if module can't be called.
 *
 *	\param stack_id   stack identifier
 *  \param layer      which layer is to be called
 *  \param b          pointer to buffer
 */
portCHAR stack_module_check(uint8_t stack_id, uint8_t layer, buffer_t *b)
{
	uint8_t i;
		
	if (layer < stacks[stack_id].layers)
	{	/*valid layer*/
		i = 0;
		while(modules[i].id)
		{
			if (modules[i].id == stacks[stack_id].module[layer])
			{
				return modules[i].check(b);
			}
			else i++;
		}
	}
	return pdFALSE;
}

/**
 *  Get buffer from stack.
 *
 *	\param blocktime  time to wait for buffer
 *
 *  \return           pointer to buffer, 0 on failure (all buffers allocated)
 */
buffer_t* stack_buffer_get ( portTickType blocktime )
{
  buffer_t *b; 

  if (xQueueReceive( buffers, &( b ), 0) == pdTRUE)
	{
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->socket = 0;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
	else if (n_buffers < STACK_BUFFERS_MAX)
	{
		b = pvPortMalloc(sizeof(buffer_t)+BUFFER_SIZE);
		if (b)
		{
  			memset(b, 0, sizeof(buffer_t));
			b->options.type = BUFFER_DATA;
			b->dir = BUFFER_DOWN;
			b->size = BUFFER_SIZE;
			b->buf_ptr = 0;
			b->buf_end = 0;
			b->options.lowpan_compressed=0;
			b->options.handle_type = HANDLE_DEFAULT;
			n_buffers++;
			return b;
		}
	}
    else if (xQueueReceive( buffers, &( b ), blocktime / portTICK_RATE_MS) == pdTRUE)
	{
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->socket = 0;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
	return 0;
}

#ifdef HAVE_DYNAMIC_BUFFERS
/**
 *  Allocate a smaller buffer from stack (for data storage).
 *
 *	\param size  size of buffer
 *
 *  \return           pointer to buffer, 0 on failure (no memory)
 */
buffer_t* stack_buffer_allocate( uint8_t size )
{
  	buffer_t *b; 
	b = pvPortMalloc(sizeof(buffer_t)+ size);
	if (b)
	{
		memset(b, 0, sizeof(buffer_t));
		b->options.type = BUFFER_DATA;
		b->dir = BUFFER_DOWN;
		b->size = size;
		b->buf_ptr = 0;
		b->buf_end = 0;
		b->socket = 0;
		b->from = MODULE_NONE;
		b->to = MODULE_NONE;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
		return b;
	}
	return 0;
}
#endif

/**
 *  Free stack buffer.
 *
 *	\param b					pointer to buffer
 *
 */
void stack_buffer_free ( buffer_t *b )
{
	if (b)
  	{ 
		b->socket = 0;
		b->from = MODULE_NONE;
		b->to = MODULE_NONE;
		b->options.lowpan_compressed=0;
		b->options.handle_type = HANDLE_DEFAULT;
	#ifdef HAVE_DYNAMIC_BUFFERS
		if (b->size == BUFFER_SIZE)
		{
			if(xQueueSend( buffers, ( void * ) &b, ( portTickType ) 0 ) != pdTRUE)
				debug("FER\r\n");
		}
		else
		{
			vPortFree(b);
		}
	#else
		xQueueSend( buffers, ( void * ) &b, ( portTickType ) 0 );
	#endif
	}
}

/**
 *  Push buffer to stack.
 *
 *	\param b  pointer to buffer
 *
 *  \return  pdTRUE
 *  \return  pdFALSE queue full
 */
portCHAR stack_buffer_push( buffer_t * b)
{
	event_t event;
	event.process = 0;
	event.param = (void *) b;
	
	return xQueueSend( events, ( void * ) &event, ( portTickType ) 0 );
}

/**
 *  Check buffer headroom.
 *
 *	\param b  pointer to buffer
 *	\param size  required headroom space
 *
 *  \return  pdTRUE
 *  \return  pdFALSE packet too long
 */
portCHAR stack_buffer_headroom( buffer_t * b, uint16_t size)
{
	if (b->buf_ptr < size)
	{
		if ((b->buf_end + size) >  b->size)
		{
			return pdFALSE;
		}
		memmove(&(b->buf[(b->buf_ptr)+size]), 
		&(b->buf[b->buf_ptr]), 
		buffer_data_length(b));
		b->buf_end += size;
		b->buf_ptr += size;
	}
	return pdTRUE;
}

/**
 *  Get number of stacks.
 *
 *  \return  STACKS_MAX
 */
uint8_t stack_number_get()
{
	return STACKS_MAX;
}

stack_event_t event_queue;
/**
 *  Open bus for stack event messages.
 *
 *  \return  event_queue what is the pointer stacks event message queue.
 */
stack_event_t open_stack_event_bus(void)
{
	event_queue = xQueueCreate( 8, sizeof( buffer_t * ) );
	return event_queue;
}

/**
 *  Waiting stack event messages from stack.
 *
 *  \return  pointer to event message.
 */
buffer_t * waiting_stack_event(uint16_t time)
{
	buffer_t *b;
	if(xQueueReceive(event_queue, &(b), time / portTICK_RATE_MS) == pdTRUE)
		return b;
	else
		return 0;
}


/**
 *  Compare two addresses
 *
 *	\brief	Convert addresses to full format if fields are missing.
 *
 *	\param a1 first address to compare
 *	\param a2 second address to compare
 *
 *  \return  pdTRUE match
 *  \return  pdFALSE no match
 */
portCHAR stack_compare_address(sockaddr_t *a1, sockaddr_t *a2)
{
	uint8_t size[2] = {0,0};
	uint16_t pan[2] = {0xFFFF,0xFFFF};
	uint8_t match = 0;
	uint8_t i;
	
	{
		addrtype_t tmp_type;
		
		for (i=0; i<2; i++)
		{
			uint8_t *dptr = 0;
			sockaddr_t *tmp_addr;
			
			if (i==0)
			{ tmp_type = a1->addr_type;
				tmp_addr = a1;
			}
			else
			{ tmp_type = a2->addr_type;
				tmp_addr = a2;
			}
			
			switch(tmp_type)
			{					
				case ADDR_802_15_4_PAN_SHORT:
					dptr = tmp_addr->address;
					dptr += 2;
				case ADDR_802_15_4_SHORT:
					size[i] = 2;
					break;
					
				case ADDR_802_15_4_PAN_LONG:
					dptr = tmp_addr->address;
					dptr += 8;
				case ADDR_802_15_4_LONG:
					size[i] = 8;
					break;
				
				case ADDR_SHORT:
					size[i] = 1;
					break;
					
				case ADDR_PAN:
					dptr = tmp_addr->address;
				case ADDR_BROADCAST:
				case ADDR_NONE:
					size[i] = 0;
					break;
			}
			if (dptr)
			{
				pan[i] = *dptr++;
				pan[i] <<= 8;
				pan[i] = *dptr++;
			}
		}
	}
	if (stack_check_broadcast(a1->address, a1->addr_type) == pdTRUE)
	{
		match = 1;
	}
	if (stack_check_broadcast(a2->address, a2->addr_type) == pdTRUE)
	{
		match = 1;
	}

	if (!match)
	{
		uint8_t cmp_size;
		if (size[0] > size[1]) cmp_size = size[1];
		else 	cmp_size = size[0];
		if (memcmp(a2->address, a1->address, cmp_size) == 0)
		{
			match = 1;
		}
	}
	
	if (match)
	{
		if ((pan[0] != 0xFFFF) && (pan[1] != 0xFFFF))
		{
			if (pan[0] != pan[1]) return pdFALSE;
		}
	}
	else
	{
		return pdFALSE;
	}
	
	return pdTRUE;
}
		

/**
 *	Add an address to a given buffer
 *
 *	\param	dptr	Index to the data field of the buffer where to insert the address
 *	\param	type	Type of the address (ADDR_802_15_4_PAN_LONG, ADDR_802_15_4_LONG, ADDR_802_15_4_PAN_SHORT or ADDR_802_15_4_SHORT)
 *	\param	address	The actual address structure
 */
uint8_t * stack_insert_address_to_buffer(uint8_t *dptr, addrtype_t type, address_t address)
{
	uint8_t i;
	if(type == ADDR_802_15_4_PAN_LONG || type == ADDR_802_15_4_LONG )
	{
		if(type == ADDR_802_15_4_PAN_LONG)
		{
			/*buf->buf[ind++] = address[8];
			buf->buf[ind++] = address[9];*/
			*dptr++ = address[8];
			*dptr++ = address[9];
		}
		for (i = 0; i < 8; i++)
		{
			*dptr++ = address[i];
		}
	}
	else if(type == ADDR_802_15_4_PAN_SHORT  || type == ADDR_802_15_4_SHORT)
	{
		if(type == ADDR_802_15_4_PAN_SHORT)
		{
			*dptr++ = address[2];
			*dptr++ = address[3];
		}
		for (i = 0; i < 2; i++)
		{
			*dptr++ = address[i];
		}
	}
	else
	{

	}
return dptr;
}

/**
 *	Check if all the bytes in the address field are 0xff
 *
 *	\param	address	The address field to check
 *	\param	type	The address type (so that the function knows how many bytes to check)
 *	\return	pdFALSE	Not a broadcast address
 *	\return	pdTRUE	A brodcast address
 */
portCHAR stack_check_broadcast(address_t address, addrtype_t type)
{
	uint8_t i, len=4;
	if(type==ADDR_802_15_4_PAN_LONG || type==ADDR_802_15_4_LONG)
		len=8;
	if(type==ADDR_SHORT)
		len=2;
	for(i=0; i<len; i++)
	{
		if(address[i] != 0xff)
		{
			return pdFALSE;
		}
	}
	return pdTRUE;
}
