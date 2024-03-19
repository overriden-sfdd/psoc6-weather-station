/*
 * i2c_utils.h
 *
 *  Created on: Apr 19, 2022
 *      Author: sfdd
 */

#ifndef I2C_UTILS_H_
#define I2C_UTILS_H_

#include "cyhal.h"
#include "bmp2.h"
#include "cy8ckit_028_tft.h"

extern struct bmp2_dev bmp_sensor;
extern struct bmp2_config bmp_sensor_conf;
//extern mtb_light_sensor_t light_sensor;

///*!
// *  @brief Function for manual tft initialization.
// *
// *  @retval BMP2_INTF_RET_SUCCESS -> Success.
// *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
// */
//int8 mtft_sensor_init();
//
///*!
// *  @brief Function for light sensor initialization.
// *
// *  @retval BMP2_INTF_RET_SUCCESS -> Success.
// *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
// */
//int8 light_sensor_init();

/*!
 *  @brief Function for BMP280 sensor initialization.
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
int8 bmp2_sensor_init();

/*!
 *  @brief Function for master I2C initialization.
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
int8 i2c_master_init();

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr   : Register address.
 *  @param[out] reg_data  : Pointer to the data buffer to store the read data.
 *  @param[in] length     : No of bytes to read.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return Status of execution
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
int8 bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);


/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *  @param[in] intf_ptr : Interface pointer
 *
 *  @return Status of execution
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
int8 bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 *  APIs.
 *
 *  @param[in] period_us  : The required wait time in microsecond.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return void.
 */
void bmp2_delay_us(uint32_t period_us, void *intf_ptr);


/*!
 *  @brief This function is to select the interface between SPI and I2C.
 *
 *  @param[in] dev    : Structure instance of bmp2_dev
 *  @param[in] intf   : Interface selection parameter
 *                          For I2C : BMP2_I2C_INTF
 *                          For SPI : BMP2_SPI_INTF
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure
 */
int8_t bmp2_interface_selection(uint8_t intf);

/*!
 *  @brief This internal API is used to get compensated pressure and temperature data.
 *
 *  @param[in] period   : Contains the delay in microseconds
 *  @param[in] conf     : Structure instance of bmp2_config.
 *  @param[in] dev      : Structure instance of bmp2_dev.
 *
 *  @return Status of execution.
 */
int8_t get_bmp280_data(struct bmp2_data *comp_data);

/*!
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmp2_error_codes_print_result(const char api_name[], int8_t rslt);

#endif /* I2C_UTILS_H_ */
