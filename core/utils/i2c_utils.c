/*
 * i2c_utils.c
 *
 *  Created on: Apr 19, 2022
 *      Author: sfdd
 */

#include "i2c_utils.h"
#include "cy_retarget_io.h"

/* I2C related macros. */
#define IMU_I2C_SDA (P6_1)
#define IMU_I2C_SCL (P6_0)

//#define LIGHT_SENSOR_PIN (CYBSP_A0)

/*! Variables that hold the I2C device address or SPI chip selection */
static uint8_t dev_addr;
static uint32_t i2c_timeout = 500; /* ms */
static cyhal_i2c_t i2c;
static uint32_t meas_time;

///*! Variables for tft */
//static cyhal_adc_t adc;
//static cyhal_adc_t* adc_ptr;
//static const mtb_st7789v_pins_t tft_pins =
//{
//		.db08 = CY8CKIT_028_TFT_PIN_DISPLAY_DB8,
//		.db09 = CY8CKIT_028_TFT_PIN_DISPLAY_DB9,
//		.db10 = CY8CKIT_028_TFT_PIN_DISPLAY_DB10,
//		.db11 = CY8CKIT_028_TFT_PIN_DISPLAY_DB11,
//		.db12 = CY8CKIT_028_TFT_PIN_DISPLAY_DB12,
//		.db13 = CY8CKIT_028_TFT_PIN_DISPLAY_DB13,
//		.db14 = CY8CKIT_028_TFT_PIN_DISPLAY_DB14,
//		.db15 = CY8CKIT_028_TFT_PIN_DISPLAY_DB15,
//		.nrd  = CY8CKIT_028_TFT_PIN_DISPLAY_NRD,
//		.nwr  = CY8CKIT_028_TFT_PIN_DISPLAY_NWR,
//		.dc   = CY8CKIT_028_TFT_PIN_DISPLAY_DC,
//		.rst  = CY8CKIT_028_TFT_PIN_DISPLAY_RST,
//};
//
//int8 mtft_sensor_init()
//{
//	return mtb_st7789v_init8(&tft_pins);
//}
//
//
//int8 light_sensor_init()
//{
//	cy_rslt_t rslt = cyhal_adc_init(&adc, CY8CKIT_028_TFT_PIN_ALS_OUT, NULL);
//	if (CY_RSLT_SUCCESS == rslt)
//	{
//		adc_ptr = &adc;
//	}
//
//	rslt = mtb_light_sensor_init(&light_sensor, adc_ptr, CY8CKIT_028_TFT_PIN_ALS_OUT);
//	return rslt;
//}


int8 bmp2_sensor_init()
{
	int8_t rslt;
	/* Interface selection is to be updated as parameter
	 * For I2C :  BMP2_I2C_INTF
	 * For SPI :  BMP2_SPI_INTF
	 */
	rslt = bmp2_interface_selection(BMP2_I2C_INTF);
	bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

	rslt = bmp2_init(&bmp_sensor);
	bmp2_error_codes_print_result("bmp2_init", rslt);

	/* Always read the current settings before writing, especially when all the configuration is not modified */
	rslt = bmp2_get_config(&bmp_sensor_conf, &bmp_sensor);
	bmp2_error_codes_print_result("bmp2_get_config", rslt);

	/* Configuring the over-sampling mode, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	bmp_sensor_conf.filter = BMP2_FILTER_OFF;

	/* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
	bmp_sensor_conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

	/* Setting the output data rate */
	bmp_sensor_conf.odr = BMP2_ODR_250_MS;

	rslt = bmp2_set_config(&bmp_sensor_conf, &bmp_sensor);
	bmp2_error_codes_print_result("bmp2_set_config", rslt);

	/* Set normal power mode */
	rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &bmp_sensor_conf, &bmp_sensor);
	bmp2_error_codes_print_result("bmp2_set_power_mode", rslt);

	/* Calculate measurement time in microseconds */
	rslt = bmp2_compute_meas_time(&meas_time, &bmp_sensor_conf, &bmp_sensor);
	bmp2_error_codes_print_result("bmp2_compute_meas_time", rslt);

	printf("BMP280 initialization is completed!\n");
	return rslt;
}

int8 i2c_master_init()
{
	cy_rslt_t result;

	cyhal_i2c_cfg_t i2c_cfg = {
			.is_slave = false,
			.address = 0,
			.frequencyhal_hz = 400000
	};


	/* Initialize i2c for motion sensor */
	result = cyhal_i2c_init(&i2c, IMU_I2C_SDA, IMU_I2C_SCL, NULL);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
	result = cyhal_i2c_configure(&i2c, &i2c_cfg);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
	return (int8)result;
}

void bmp2_delay_us(uint32_t period_us, void *intf_ptr)
{
	Cy_SysLib_DelayUs(period_us);
}

int8 bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	dev_addr = *(uint8_t*)intf_ptr;

	cy_rslt_t result ;
	result = cyhal_i2c_master_write(&i2c, dev_addr, &reg_addr, 1, i2c_timeout, false) ;
	if (result != CY_SCB_I2C_SUCCESS) {
		printf("i2c_read_regs: write slave address failed\n\r") ;
		return( result ) ;
	}

	result = cyhal_i2c_master_read(&i2c, dev_addr, reg_data, (uint16_t)length, i2c_timeout, true) ;

	if (result != CY_SCB_I2C_SUCCESS) {
		printf("i2c_read_regs: read data failed\n\r") ;
		return( result ) ;
	}

	return (int8)result;
}

int8 bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	dev_addr = *(uint8_t*)intf_ptr;

	CY_ASSERT((length + 1) < BMP2_MAX_LEN);
	uint8_t buf[BMP2_MAX_LEN];
	buf[0] = reg_addr;
	for (uint16_t i=0; i < length; i++)
	{
		buf[i+1] = reg_data[i];
	}

	return cyhal_i2c_master_write(&i2c, dev_addr, buf, (uint16_t)length + 1, i2c_timeout, false);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp2_interface_selection(uint8_t intf)
{
	int8_t rslt = BMP2_OK;

	/* Bus configuration : I2C */
	if (intf == BMP2_I2C_INTF)
	{
		printf("I2C Interface\n");

		dev_addr = BMP2_I2C_ADDR_PRIM;
		bmp_sensor.read = bmp2_i2c_read;
		bmp_sensor.write = bmp2_i2c_write;
		bmp_sensor.intf = BMP2_I2C_INTF;
	}
	/* Bus configuration : SPI */
	else if (intf == BMP2_SPI_INTF)
	{
		printf("SPI Interface is ignored!\n");
		rslt = BMP2_E_NULL_PTR;
	}

	/* Holds the I2C bmp_sensorice addr or SPI chip selection */
	bmp_sensor.intf_ptr = &dev_addr;

	/* Configure delay in microseconds */
	bmp_sensor.delay_us = bmp2_delay_us;

	Cy_SysLib_Delay(100);

	return rslt;
}

/*!
 *  @brief Function to get data from BMP280.
 */
int8_t get_bmp280_data(struct bmp2_data *comp_data)
{
	int8_t rslt = BMP2_E_NULL_PTR;
	int8_t idx = 1;
	struct bmp2_status status;

	rslt = bmp2_get_status(&status, &bmp_sensor);
	bmp2_error_codes_print_result("bmp2_get_status", rslt);

	if (status.measuring == BMP2_MEAS_DONE)
	{
		/* Delay between measurements */
		bmp_sensor.delay_us(meas_time, bmp_sensor.intf_ptr);

		/* Read compensated data */
		rslt = bmp2_get_sensor_data(comp_data, &bmp_sensor);
		bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

#ifdef BMP2_64BIT_COMPENSATION
		comp_data.pressure = comp_data.pressure / 256;
#endif

#ifdef BMP2_DOUBLE_COMPENSATION
		printf("Data[%d]:    Temperature: %.4lf deg C	Pressure: %.4lf Pa\n",
				idx,
				comp_data->temperature,
				comp_data->pressure);
#else
		printf("Data[%d]:    Temperature: %ld deg C	Pressure: %lu Pa\n", idx, (long int)comp_data.temperature,
				(long unsigned int)comp_data.pressure);
#endif

		idx++;
	}

	return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp2_error_codes_print_result(const char api_name[], int8_t rslt)
{
	if (rslt != BMP2_OK)
	{
		printf("%s\t", api_name);

		switch (rslt)
		{
		case BMP2_E_NULL_PTR:
			printf("Error [%d] : Null pointer error.", rslt);
			printf(
					"It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
			break;
		case BMP2_E_COM_FAIL:
			printf("Error [%d] : Communication failure error.", rslt);
			printf(
					"It occurs due to read/write operation failure and also due to power failure during communication\r\n");
			break;
		case BMP2_E_INVALID_LEN:
			printf("Error [%d] : Invalid length error.", rslt);
			printf("Occurs when length of data to be written is zero\n");
			break;
		case BMP2_E_DEV_NOT_FOUND:
			printf("Error [%d] : device not found error. It occurs when the device chip id is incorrectly read\r\n",
					rslt);
			break;
		case BMP2_E_UNCOMP_TEMP_RANGE:
			printf("Error [%d] : Uncompensated temperature data not in valid range error.", rslt);
			break;
		case BMP2_E_UNCOMP_PRESS_RANGE:
			printf("Error [%d] : Uncompensated pressure data not in valid range error.", rslt);
			break;
		case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
			printf(
					"Error [%d] : Uncompensated pressure data and uncompensated temperature data are not in valid range error.",
					rslt);
			break;
		default:
			printf("Error [%d] : Unknown error code\r\n", rslt);
			break;
		}
	}
}
