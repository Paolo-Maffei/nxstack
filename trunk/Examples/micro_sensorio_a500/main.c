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
 * \brief Example application for sensor I/O sensors, application support SSi discover and request messages and ping response.
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

typedef uint8_t b1w_reg[8];
extern uint8_t bus_1wire_search(b1w_reg *devices, uint8_t n_devices);
extern sockaddr_t mac_long;
static void vSensorTask( void *pvParameters );

#include "socket.h"
#include "rf.h"

ssi_sensor_t ssi_sensor[] =
{/*  ID             | unit type        |scale|data|status*/
  {2, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {0, SSI_DATA_TYPE_INT,  0,   {0},   0},
  {8, SSI_DATA_TYPE_INT,  0,   {0},   0}/*,  
  {           0xC002, SSI_DATA_TYPE_INT,  0,   0,   0}*/
};

const uint8_t *ssi_description[] =
{
  "Temp",
  "Light",
  "Btns/LEDs"
};

const uint8_t *ssi_unit[] =
{
/*  "C",*/
	"RAW",
	"RAW",
	"00BB00LL"
};

const uint8_t ssi_n_sensors = sizeof(ssi_sensor)/sizeof(ssi_sensor_t);

sockaddr_t test_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0x00, 0x00, 0x01, 0x00,0x00, 
	  0x00, 0x00, 0x00, 0x01,0x00 },
	61619
};

sockaddr_t broadcast_address = 
{
	ADDR_802_15_4_PAN_LONG,
	{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 
	  0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	7
};

uint8_t test_packet[] = 
{
	0x0C, 0x00, 0x02, 0xFC, 0xFE, 0x00, 0x01
};

int main( void )
{
	LED_INIT();
	if (bus_init() == pdFALSE)
	{
		for (;;)
		{
			LED1_ON();
			LED2_OFF();
			LED1_OFF();
			LED2_ON();
		}
	}	
	
	xTaskCreate( vSensorTask, "Sens", configMINIMAL_STACK_SIZE + 100, NULL, (tskIDLE_PRIORITY + 2 ), NULL );
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/


static void vSensorTask( void *pvParameters )
{
	portTickType xLastWakeTime;
	uint16_t count = 0;
	uint8_t bl_state = 0;
	uint8_t bl_state_p = 0;
	pause(200);
	debug_init(115200);
	pause(300);

	if(stack_start(NULL)==START_SUCCESS)
	{
		debug("Start Ok\r\n");
	}

	xLastWakeTime = xTaskGetTickCount();
	LED1_ON();
	vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_RATE_MS );
	LED1_OFF();

	P6DIR |= 0xC0;
	P6DIR &= ~0x0F;
	P6OUT |= 0xC0;
	P5DIR &= ~0x80;
	P2DIR &= ~0x01;
		
	for (;;)
	{
		vTaskDelayUntil( &xLastWakeTime, 20 / portTICK_RATE_MS );
		bl_state &= ~ 0x33;
		/*read example board buttons*/
		if (P2IN & 0x01) 
		{
			bl_state |= 0x10;
		}
		if (P5IN & 0x80)
		{
		 	bl_state |= 0x20;
		}
		/*setup example board LEDs according to button changes*/
		if ((bl_state ^ bl_state_p) & 0x10)
		{
			P6OUT ^= 0x40;
		}
		if ((bl_state ^ bl_state_p) & 0x20)
		{
			P6OUT ^= 0x80;
		}
		if (P6OUT & 0x40) bl_state |= 0x01;
		if (P6OUT & 0x80) bl_state |= 0x02;
		if (bl_state != bl_state_p)
		{
			ssi_sensor_update(2, (uint32_t) bl_state);
		}	
		bl_state_p = bl_state;
		
		if (adc_state == 0)
		{	/*adc read complete, new values available*/
			ssi_sensor_update(0, (uint32_t) adc_value[2]);
			ssi_sensor_update(1, (uint32_t) adc_value[0]);
			adc_state = 16;
		}
		
		if (count++ >= 49)
		{
			uint8_t channels[] = {0,1,2};
			
			LED1_ON();
			adc_read(channels,sizeof(channels));
			count = 0;
		}
		else
		{
			LED1_OFF();
		}
	}
}
