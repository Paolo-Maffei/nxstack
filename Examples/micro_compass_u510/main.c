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
 * \brief Example application for micro compas board.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <signal.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bus.h"
#include "gpio.h"

#include "debug.h"
#include "ssi.h"
#include "adc.h"
#include "compass.h"

//static xQueueHandle LEDQueue; 

static void micro_compass_sensor( void *pvParameters );

#include "socket.h"
#include "rf.h"

ssi_sensor_t ssi_sensor[] =
{/*  ID             | unit type        |scale|data|status*/
/*  {0x0800, SSI_DATA_TYPE_INT,  0,   {0},   0},*/
  {0xC801, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {0xC802, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {0xC803, SSI_DATA_TYPE_INT,  0,   {0},   0},  
  {0xC901, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {0xC902, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {0xC903, SSI_DATA_TYPE_INT,  0,   {0},   0},  
  {0x0009, SSI_DATA_TYPE_INT,  0,   {0},   0}/*,  
  {           0xC002, SSI_DATA_TYPE_INT,  0,   0,   0}*/
};

uint8_t *ssi_description[] =
{
/*  "Compass",*/
	"MX",
	"MY",
	"MZ",
	"AX",
	"AY",
	"AZ",
	"State"
};

uint8_t *ssi_unit[] =
{
/*  "",*/
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"000000Cc"
};

const uint8_t *sensor_description[] =
{
/*  "Compass",*/
	"MX",
	"MY",
	"MZ",
	"AX",
	"AY",
	"AZ",
	"State"
};

const uint8_t *sensor_unit[] =
{
/*  "",*/
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"000000Cc"
};

uint8_t ssi_n_sensors = 7;

const uint8_t sensor_n_sensors = 7;

const uint8_t *cal_description[] =
{
	"X-",
	"Y-",
	"Z-",
	"X+",
	"Y+",
	"Z+",
	"State"
};


const uint8_t *cal_unit[] =
{
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"RAW",
	"a0xXyYzZ"
};

const uint8_t cal_n_sensors = 7;

sockaddr_t test_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x01, 0x00, 
	  0x00, 0x00, 0x00, 0x01,
		0xFF, 0xFF },
	253
};

sockaddr_t broadcast_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF },
	254
};

uint8_t test_packet[] = 
{
	0x0C, 0x00, 0x02, 0xFC, 0xFE, 0x00, 0x01
};

extern sockaddr_t mac_long;

int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
		for (;;)
		{
			LED1_ON();
			LED1_OFF();
		}
	}	
	//LEDQueue = xQueueCreate( 8, ( unsigned portBASE_TYPE ) sizeof( uint16_t ) );
	xTaskCreate( micro_compass_sensor, "Sens", configMINIMAL_STACK_SIZE + 130, NULL, (tskIDLE_PRIORITY + 2 ), NULL );
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/
typedef uint8_t b1w_reg[8];

/**
 *	Sensor reading task. Sensor reading triggered by terminal
 *  or SSI request.
 *
 *	\param pvParameters not used
 *
 */ 
static void micro_compass_sensor( void *pvParameters )
{
	uint16_t count = 0;
	uint8_t	mag = 0;
	uint8_t	acc = 0;
			
	uint8_t i;
	pause(200);
	debug_init(115200);
	pause(300);

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}
	if (pvParameters);
	
	LED1_ON();
	vTaskDelay(500 / portTICK_RATE_MS );
	LED1_OFF();
	vTaskDelay(50 / portTICK_RATE_MS );

	for (;;)
	{	/*standard mode*/
		vTaskDelay(40 / portTICK_RATE_MS);

		count++;
		if (count > 200)
		{
			count = 0;
			LED1_OFF();
		}
		else if (count > 190)
		{
			LED1_ON();
		}
		for (i=0; i<3; i++)
		{
			if (ssi_sensor[i].status & SSI_STATUS_REQUESTED) mag = 0x07;
			if (ssi_sensor[i+3].status & SSI_STATUS_REQUESTED) acc = 0x70;
		}	
		
		if ( mag | acc )
		{	/*sensors requested*/
			int16_t value[3];

			if (compass_select() == pdTRUE)
			{	/*got bus, compass selected*/
				i = 0;
				pause_us(100);
				if (mag)
				{	/*read mag*/
					LED1_ON();
					count = 0;
					if (compass_mag_read_single(value) == pdTRUE)
					{
						//debug("Mag update.\r\n");
						for(i=0; i<3;i++)
						{
							ssi_sensor_update(i, (int32_t) value[i]);
							ssi_sensor_update(6, ssi_sensor[6].sdata.i | 0x07);
						}
						mag = 0;
					}
				}
				pause_us(100);
				if (acc)
				{	/*read acc*/
					LED1_ON();
					count = 0;

					if (compass_accel_read_single(value) == pdTRUE)
					{
						//debug("Accel update.\r\n");
						for(i=0; i<3;i++)
						{
							ssi_sensor_update(i+3, (int32_t) value[i]);
							ssi_sensor_update(6, ssi_sensor[6].sdata.i | 0x70);
						}
						acc = 0;
					}
				}
				compass_unselect();
			}
			else
			{
				//debug("Select failed.\n");
			}
		}
	}		
}
