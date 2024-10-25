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
		lastFinishedHalf = adcState[0].buf;
		lastFinishedADC = 1;
	}
	if(i2s == &hi2s2){
		adcState[1].state = StreamState::FrontHalfReady;
		lastFinishedHalf = adcState[1].buf;
		lastFinishedADC = 2;
	}
	if(i2s == &hi2s3){
		adcState[2].state = StreamState::FrontHalfReady;
		lastFinishedHalf = adcState[2].buf;
		lastFinishedADC = 3;
	}
}

void InputStream::rxComplete(I2S_HandleTypeDef* i2s){
	dataReady = true;
	if(i2s == &hi2s1){
		adcState[0].state = StreamState::BackHalfReady;
		lastFinishedHalf = &adcState[0].buf[STREAM_BUF_SIZE / 2];
		lastFinishedADC = 1;
	}
	if(i2s == &hi2s2){
		adcState[1].state = StreamState::BackHalfReady;
		lastFinishedHalf = &adcState[1].buf[STREAM_BUF_SIZE / 2];
		lastFinishedADC = 2;
	}
	if(i2s == &hi2s3){
		adcState[2].state = StreamState::BackHalfReady;
		lastFinishedHalf = &adcState[1].buf[STREAM_BUF_SIZE / 2];
		lastFinishedADC = 3;
	}
}

void InputStream::tick(){
	if(dataReady){
		switch(lastFinishedADC){
		case 1:
			parseHalfChunkToStrings(lastFinishedHalf, 0, 1);
			break;
		case 2:
			parseHalfChunkToStrings(lastFinishedHalf, 2, 3);
			break;
		case 3:
			parseHalfChunkToStrings(lastFinishedHalf, 4, 5);
			break;
		default:
			break;
		}
		dataReady = false;
	}

}

float parse24Bit(uint32_t val){
	int32_t signedVal = (int32_t)val - 8388608;
	return (float)signedVal / 8388608.0f;
}

void InputStream::parseHalfChunkToStrings(uint32_t* arr, uint8_t strL, uint8_t strR){
	uint16_t idx = 0;
	while(idx < STREAM_BUF_SIZE / 2){
		strings[strL].data.push(parse24Bit(arr[idx]));
		strings[strR].data.push(parse24Bit(arr[idx + 1]));
		idx += 2;
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

void input_stream_tick(input_stream_t stream){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->tick();
}


void rx_half_complete(input_stream_t stream, I2S_HandleTypeDef* i2s){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->rxHalfComplete(i2s);
}

void rx_complete(input_stream_t stream, I2S_HandleTypeDef* i2s){
	InputStream* ptr = static_cast<InputStream*>(stream);
	ptr->rxComplete(i2s);
}

