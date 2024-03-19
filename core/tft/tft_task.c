/*
 * tft_task.c
 *
 *  Created on: Apr 23, 2022
 *      Author: sfdd
 */

#include "GUI.h"
#include <FreeRTOS.h>
#include <task.h>

#include "tft_task.h"
#include "i2c_utils.h"

void tft_display_task(void *arg)
{
	/* Sensors data. */
	uint8_t light_data;
	struct bmp2_data comp_data;

	/* Define 1000ms delay. */
	const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

	/* Initialize GUI */
	GUI_Init();
	// print headers
	GUI_DispStringAt("Data:  P:       T:       L:", 0, 10);

	for (;;)
	{
//		light_data = mtb_light_sensor_light_level(&light_sensor);
		get_bmp280_data(&comp_data);

		/* Display data. */
		GUI_DispDecAt(light_data, 55, 10, 6);
		GUI_DispDecAt(comp_data.pressure, 110, 10, 6);
		GUI_DispDecAt(comp_data.temperature, 165, 10, 6);

		vTaskDelay( xDelay );
	}
}
