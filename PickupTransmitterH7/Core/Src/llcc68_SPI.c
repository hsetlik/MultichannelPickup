/*
 * llcc68_SPI.c
 *
 *  Created on: Oct 25, 2024
 *      Author: hayden
 */

#include "llcc68_SPI.h"

#ifndef HAL_LLCC68_SPI
#define HAL_LLCC68_SPI

llcc68_hal_status_t llcc68_hal_read(const void *context, const uint8_t *command,
		const uint16_t commandLength, uint8_t *data,
		const uint16_t dataLength) {
	return LLCC68_HAL_STATUS_ERROR;
}

llcc68_hal_status_t llcc68_hal_write(const void *context,
		const uint8_t *command, const uint16_t command_length,
		const uint8_t *data, const uint16_t data_length) {
	return LLCC68_HAL_STATUS_ERROR;
}

llcc68_hal_status_t llcc68_hal_reset(const void *context) {
	return LLCC68_HAL_STATUS_ERROR;
}

llcc68_hal_status_t llcc68_hal_wakeup(const void *context) {
	return LLCC68_HAL_STATUS_ERROR;
}

#endif
