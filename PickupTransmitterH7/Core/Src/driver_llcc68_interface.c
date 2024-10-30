/*
 * driver_llcc68_interface.c
 *
 *  Created on: Oct 29, 2024
 *      Author: hayden
 */

#include "main.h"
#include "driver_llcc68_interface.h"

uint8_t llcc68_interface_spi_init(void){
	if(RADIO_SPI.ErrorCode == HAL_SPI_ERROR_NONE){
		return 0;
	}
	return 1;
}

uint8_t llcc68_interface_spi_deinit(void){
	return 0;
}


uint8_t llcc68_interface_spi_write_read(uint8_t *in_buf, uint32_t in_len,
                                        uint8_t *out_buf, uint32_t out_len){
	if(HAL_SPI_TransmitReceive(&RADIO_SPI, out_buf, in_buf, out_len, 1000) != HAL_OK){
		return 1;
	}
	return 0;
}

uint8_t llcc68_interface_reset_gpio_init(void){
  HAL_GPIO_WritePin(LORA_NRST_GPIO_Port, LORA_NRST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_NRST_GPIO_Port, LORA_NRST_Pin, GPIO_PIN_SET);
  return 0;
}


uint8_t llcc68_interface_reset_gpio_deinit(void){
	return 0;
}
