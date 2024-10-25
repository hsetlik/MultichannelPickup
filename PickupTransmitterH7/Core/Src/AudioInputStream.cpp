/*
 * AudioInputStream.cpp
 *
 *  Created on: Oct 25, 2024
 *      Author: hayden
 */
#include "AudioInputStream.h"

InputStream::InputStream(){

}

void InputStream::begin(){
	uint16_t* buf1 = (uint16_t*)adcState[0].buf;
	uint16_t* buf2 = (uint16_t*)adcState[1].buf;
	uint16_t* buf3 = (uint16_t*)adcState[2].buf;

	if(HAL_I2S_Receive_DMA(&hi2s1, buf1, STREAM_BUF_SIZE * 2) != HAL_OK){
		Error_Handler();
	}

	if(HAL_I2S_Receive_DMA(&hi2s2, buf2, STREAM_BUF_SIZE * 2) != HAL_OK){
		Error_Handler();
	}

	if(HAL_I2S_Receive_DMA(&hi2s3, buf3, STREAM_BUF_SIZE * 2) != HAL_OK){
		Error_Handler();
	}
}

void InputStream::rxHalfComplete(I2S_HandleTypeDef* i2s){
	dataReady = true;
	if(i2s == &hi2s1){
		adcState[0].state = StreamState::FrontHalfReady;
	}
	if(i2s == &hi2s2){
		adcState[1].state = StreamState::FrontHalfReady;
	}
	if(i2s == &hi2s3){
		adcState[2].state = StreamState::FrontHalfReady;
	}
}

void InputStream::rxComplete(I2S_HandleTypeDef* i2s){
	dataReady = true;
	if(i2s == &hi2s1){
		adcState[0].state = StreamState::BackHalfReady;
	}
	if(i2s == &hi2s2){
		adcState[1].state = StreamState::BackHalfReady;
	}
	if(i2s == &hi2s3){
		adcState[2].state = StreamState::BackHalfReady;
	}
}

//=======================================================
input_stream_t create_input_stream() {
	return new InputStream();
}

void input_stream_begin(input_stream_t stream){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->begin();
}

void rx_half_complete(input_stream_t stream, I2S_HandleTypeDef* i2s){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->rxHalfComplete(i2s);
}

void rx_complete(input_stream_t stream, I2S_HandleTypeDef* i2s){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->rxComplete(i2s);
}

