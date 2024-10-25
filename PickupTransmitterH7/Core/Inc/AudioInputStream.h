/*
 * AudioInputStream.h
 *
 *  Created on: Oct 25, 2024
 *      Author: hayden
 */

#ifndef INC_AUDIOINPUTSTREAM_H_
#define INC_AUDIOINPUTSTREAM_H_

#define STREAM_BUF_SIZE 256
#ifdef __cplusplus


#include "main.h"
#include "RingBuffer.h"


typedef enum {
	Off,
	Busy,
	FrontHalfReady,
	BackHalfReady
} StreamState;

typedef struct {
	StreamState state = StreamState::Off;
	uint32_t buf[STREAM_BUF_SIZE];
}adc_state_t;

struct StringData {
	RingBuffer<STREAM_BUF_SIZE, float> data;
};

class InputStream {
private:
	adc_state_t adcState[3];
	bool dataReady = false;
	uint32_t* lastFinishedHalf = nullptr;
	// note: 0 indicates that none are finished yet, the first ADC is at index 1
	uint8_t lastFinishedADC = 0;
public:
	InputStream();
	// start the circular DMA transmisions
	void begin();
	// wire these to the ISRs in main.c to set the internal state flag
	void rxHalfComplete(I2S_HandleTypeDef* i2s);
	void rxComplete(I2S_HandleTypeDef* i2s);
	//call this in the main while loop to parse the last completed chunk as needed
	void tick();
	StringData strings[6];
private:
	//helper for parsing the data chunks
	void parseHalfChunkToStrings(uint32_t* ptr, uint8_t str1, uint8_t str2);

};

#endif

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

/* ----C-FRIENDLY BINDINGS GO HERE------*/

typedef void* input_stream_t;

EXTERNC input_stream_t create_input_stream();
EXTERNC void input_stream_begin(input_stream_t stream);
EXTERNC void input_stream_tick(input_stream_t stream);
EXTERNC void rx_half_complete(input_stream_t stream, I2S_HandleTypeDef* i2s);
EXTERNC void rx_complete(input_stream_t stream, I2S_HandleTypeDef* i2s);

#undef EXTERNC

#endif /* INC_AUDIOINPUTSTREAM_H_ */
