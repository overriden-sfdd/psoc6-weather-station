/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the Empty PSoC6 Application
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "wifi_config.h"
#include "i2c_utils.h"
#include "secure_http_server.h"
#include "mqtt_task.h"
//#include "tft_task.h"

#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "stdlib.h"

/* RTOS related macros. */
#define HTTPS_SERVER_TASK_STACK_SIZE        (5 * 1024)
#define HTTPS_SERVER_TASK_PRIORITY          (1)

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;

/* HTTPS server task handle. */
TaskHandle_t https_server_task_handle;

/* Define extern variables. */
struct bmp2_dev bmp_sensor;
struct bmp2_config bmp_sensor_conf;
//mtb_light_sensor_t light_sensor;
cy_wcm_ip_address_t ip_addr;

int main(void)
{
	cy_rslt_t result;

	/* This enables RTOS aware debugging in OpenOCD */
	uxTopUsedPriority = configMAX_PRIORITIES - 1;

	/* Initialize the device and board peripherals */
	result = cybsp_init() ;
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	__enable_irq();
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* Initialize the User LED. */
	result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* Initialize I2C interfaces. */
	result = i2c_master_init();
	CY_ASSERT(result == CY_RSLT_SUCCESS);

//	/* Initialize the CY8CKIT_028_TFT board */
//	result = mtft_sensor_init();
//	CY_ASSERT(result == CY_RSLT_SUCCESS);
//
//	/* Initialize the light sensor board */
//	result = light_sensor_init();
//	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* Initialize BME280 sensor */
	bmp2_sensor_init();

//	while ( wifi_connect() != CY_RSLT_SUCCESS ) {
//		Cy_SysLib_Delay(500);
//	}

	/* Create the MQTT Client task. */
	xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE,
			NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);

	/* Starts the HTTPS server in secure mode. */
	xTaskCreate(https_server_task, "HTTPS Server", HTTPS_SERVER_TASK_STACK_SIZE, NULL,
			HTTPS_SERVER_TASK_PRIORITY, &https_server_task_handle);

//	/* Create the TFT task. */
//	xTaskCreate(tft_display_task, "TFT Display task", 1024,
//			NULL, 3, NULL);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();
}

/* [] END OF FILE */
