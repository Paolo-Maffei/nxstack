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
 * \file main.c
 * \brief Example application for the NanoSensor N710 product from Sensinode.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <sys/inttypes.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* NanoStack includes */
#include "socket.h"
#include "socket.h"
#include "debug.h"
#include "ssi.h"

#include "control_message.h"

/* Platform includes */
#include "uart.h"
#include "rf.h"
#include "bus.h"
#include "dma.h"
#include "timer.h"
#include "gpio.h"
#include "adc.h"
#include "socket.h"

static void vAppTask( int8_t *pvParameters );
static void vButtonTask( int8_t *pvParameters );

int8_t get_adc_value(adc_input_t channel, uint16_t *value);

/* SSI setup */
ssi_sensor_t ssi_sensor[] =
{/*  ID             | unit type        |scale|data|status*/
  {1, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {2, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {3, SSI_DATA_TYPE_INT,  0,   {0},   0}
};

const uint8_t *ssi_description[] =
{
  "Light",
  "Temp",
	"LEDs"
};

const uint8_t *ssi_unit[] =
{
	"RAW",
	"RAW",
	"xxxxxx21"
};

const uint8_t ssi_n_sensors = sizeof(ssi_sensor)/sizeof(ssi_sensor_t);

/* Setup a default address structure, short address, broadcast, to port 125 */
sockaddr_t broadcast = 
{
	ADDR_802_15_4_PAN_SHORT, 
	{ 0xff, 0xff, 0xff, 0xff },
	125
};

xQueueHandle button_events;

/*LED blink times*/
uint16_t led1_count;
uint16_t led2_count;

/* Main task, initialize hardware and start the FreeRTOS scheduler */
int main( void )
{
	/* Initialize the Nano hardware */
	bus_init();
	LED_INIT();
	N710_SENSOR_INIT();

	/* Setup the application task and start the scheduler */
	xTaskCreate( vAppTask, "App", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1 ), ( xTaskHandle * )NULL );
	xTaskCreate( vButtonTask, "Button", configMAXIMUM_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2 ), ( xTaskHandle * )NULL );

	vTaskStartScheduler();
	
	/* Scheduler has taken control, next vAppTask starts executing. */

	return 0;
}

/** 
 * The application task reading ADC values and using the LEDs.
 */
static void vAppTask( int8_t *pvParameters )
{
	uint8_t buttons = 0;
	uint16_t U1_value = 0;
	uint16_t U2_value = 0;
	uint8_t count = 0;
	int8_t error;
	
	pvParameters;
	
	/* Start the debug UART at 115k */
	debug_init(115200);
	button_events = xQueueCreate( 4, 1 /*message size one byte*/ );
	
	led1_count = 50;
	led2_count = 100;
	
	vTaskDelay( 50 / portTICK_RATE_MS );

	/* Initialize NanoStack with default parameters, NanoStack task automatically created. */
	{
		if(stack_start(NULL)==START_SUCCESS)
		{
			debug("NanoStack Start: Ok.\r\n");
		}
	}
	
	vTaskDelay( 100 / portTICK_RATE_MS );

	debug("NanoSensor N710 Example Application.\r\n");

	/* Turn IPv6 compression on, UDP compression off */
	cipv6_compress_mode(1);
	cudp_compress_mode(0);

	/* Start an endless task loop, we must sleep most of the time allowing execution of other tasks. */
	for (;;)
	{
		/* Sleep for 500 ms or until button event */
		if (xQueueReceive(button_events, &buttons, 500 / portTICK_RATE_MS) == pdTRUE)
		{
			switch(buttons)
			{
				case 0x81:
				case 0x01:
					/* Read the LM60 illumination sensor (U1) */
					/* P0.2 */

					debug("Reading illumination (U1). ");
					if (get_adc_value(N710_LIGHT, &U1_value) == 0)
					{
							ssi_sensor_update(0, (uint32_t) U1_value);
							debug_int(U1_value);
							led1_count = 30;
					}
					else
					{
						debug("failed.");
						led1_count = 300;
					}
					debug("\r\n");
					break;

				case 0x82:
				case 0x02:
					/* Read the EL7900 temperature sensor (U2) */
					/* P0.3 */

					debug("Reading temp (U2): ");
					if (get_adc_value(N710_TEMP, &U2_value) == 0)
					{
							debug_int(U2_value);
							ssi_sensor_update(1, (uint32_t) U2_value);
							led1_count = 30;
							led2_count = 30;
					}
					else
					{
						debug("failed.");
						led1_count = 300;
						led2_count = 300;
					}
					debug("\r\n");
					break;

				default:
					debug_hex(buttons);
					break;
			}
			count = 0;
		}
		else
		{
			if (count++ >= 2)
			{
				count = 0;
				buttons = 0x01;
				xQueueSend( button_events, ( void * ) &buttons,	(portTickType) 0 );
				buttons = 0x02;
				xQueueSend( button_events, ( void * ) &buttons,	(portTickType) 0 );
			}
		}
	}
}

discover_res_t echo_result;
stack_event_t stack_event;
int8_t ping_active=0;
portTickType ping_start = 0;

/** 
 * The task for generating button events, also does terminal and ping sending
 */
static void vButtonTask( int8_t *pvParameters )
{
	uint8_t event;
	uint8_t buttons = 0;
	uint8_t s1_count = 0;
	uint8_t s2_count = 0;
	int16_t byte;
	uint8_t i;
	uint8_t channel;

	pvParameters;
	
	N710_BUTTON_INIT();

	vTaskDelay( 200 / portTICK_RATE_MS );
	
	stack_event 		= open_stack_event_bus();		/* Open socket for stack status message */
	stack_service_init( stack_event,NULL, 0 , NULL );	/* No Gateway discover */

	channel = mac_current_channel();
	
	while (1)
	{		
		/* Sleep for 10 ms or received from UART */
		byte = debug_read_blocking(10 / portTICK_RATE_MS);
		if (byte != -1)
		{
			switch(byte)
			{
				case '1':	/*send a button 1 event*/
					event = 1;
					xQueueSend( button_events, ( void * ) &event,	(portTickType) 0 );
					break;
					
				case '2': /*send a button 2 event*/
					event = 2;
					xQueueSend( button_events, ( void * ) &event,	(portTickType) 0 );
					break;
					
				case '\r':
					debug("\r\n");
					break;

				case 'm':
					{
						sockaddr_t mac;
						
						rf_mac_get(&mac);
						
						debug("MAC: ");
						debug_address(&mac);
						debug("\r\n");
					}
					break;

				case 'p':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(ping(NULL, &echo_result) == pdTRUE) /* Broadcast */
						{
							ping_start = xTaskGetTickCount();
							ping_active = 2;
							debug("Ping\r\n");
						}
						else
							debug("No buffer.\r\n");
					}
					break;
					
				case 'u':
					if(ping_active == 0)
					{
						echo_result.count=0;
						if(udp_echo(NULL, &echo_result) == pdTRUE)
						{
							ping_start = xTaskGetTickCount();
							ping_active = 1;
							debug("udp echo_req()\r\n");
						}
						else
							debug("No buffer.\r\n");
					}
					break;

				case 'C':
					if (channel < 26) channel++;
					channel++;
				case 'c':
					if (channel > 11) channel--;
					mac_set_channel(channel);
					debug("Channel: ");
					debug_int(channel);
					debug("\r\n");
					break;
						
				default:
					debug_put(byte);				
			}
		}

		/* Read button (S1 and S2) status */
		if (N710_BUTTON_1 == 1)
		{
			if (s1_count > 5)
			{
				event = 1;
				if (s1_count >= 100) event |= 0x80; /*long keypress*/
				xQueueSend( button_events, ( void * ) &event,	(portTickType) 0 );
			}
			s1_count = 0;
		}
		else
		{
			if (s1_count < 100)
			{
				s1_count++;
			}
		}
		if (N710_BUTTON_2 == 1)
		{
			if (s2_count > 5)
			{
				event = 2;
				if (s2_count >= 100) event |= 0x80;	/*long keypress*/
				xQueueSend( button_events, ( void * ) &event,	(portTickType) 0 );
			}
			s2_count = 0;
		}			
		else
		{
			if (s2_count < 100)
			{
				s2_count++;
			}
		}
		/*LED blinkers*/
		if (led1_count)
		{
			led1_count--;
			LED1_ON();
		}
		else
		{
			LED1_OFF();
		}
		if (led2_count)
		{
			led2_count--;
			LED2_ON();
		}
		else
		{
			LED2_OFF();
		}
		buttons = 0;
		if (LED1())
		{
			buttons |= 1;
		}
		if (LED2())
		{
			buttons |= 2;
		}
		ssi_sensor_update(3, (uint32_t) buttons);

		/* ping response handling */
		if ((xTaskGetTickCount() - ping_start)*portTICK_RATE_MS > 1000 && ping_active)
		{
			debug("Ping timeout.\r\n");
			stop_ping();
			if(echo_result.count)
			{
				debug("Response: \r\n");
				for(i=0; i<echo_result.count; i++)
				{
					debug_address(&(echo_result.result[i].src));
					debug(" ");
					debug_int(echo_result.result[i].rssi);
					debug(" dbm, ");
					debug_int(echo_result.result[i].time);
					debug(" ms\r\n");
				}
				echo_result.count=0;
			}
			else
			{
				debug("No response.\r\n");
			}
			ping_active = 0;
		}
		
		/* stack events */
		if(stack_event)
		{
			buffer_t *buffer = waiting_stack_event(10);
			if(buffer)
			{
				switch (parse_event_message(buffer))
				{
					case BROKEN_LINK:
						debug("Route broken to ");
						debug("\r\n");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;

					case NO_ROUTE_TO_DESTINATION:
						debug("ICMP message back, no route ");
						debug("\r\n");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;
					
					case TOO_LONG_PACKET:
						debug("Payload Too Length\r\n");
						break;
			
					case DATA_BACK_NO_ROUTE:
						debug("DATA back, No route");
						debug("\r\n");
						debug("To ");
						debug_address(&(buffer->dst_sa));
						debug("\r\n");
						break;

					default:

						break;
				}
				if(buffer)
				{
					socket_buffer_free(buffer);
					buffer = 0;
				}
			}
		} /*end stack events*/
	} /*end main loop*/
}	

int8_t get_adc_value(adc_input_t channel, uint16_t *value)
{
	int8_t retval;
	
	if (adc_convert_single(channel, ADCREF_AVDD, ADCRES_14BIT) == 0) 
	{
		retval = 0;
		while (retval != 1) 
		{
			retval = adc_result_single(value);
		}
		retval = 0;
	}
	else
	{
		retval = -1;
	}
	
	return retval;
}					
