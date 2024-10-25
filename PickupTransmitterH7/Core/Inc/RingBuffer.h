/*
 * RingBuffer.h
 *
 *  Created on: Oct 25, 2024
 *      Author: hayden
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_
#ifdef __cplusplus
#include <stdint.h>

template <uint16_t S, typename T> class RingBuffer{
private:
	T data[S];
	const uint16_t length = S;
	uint16_t head = 0;
public:
	RingBuffer(){
	}
	T& operator[](uint16_t idx){
		return data[(head + idx) % length];
	}
	void push(T value){
		data[head] = value;
		head = (head + 1) % length;
	}
};



#endif

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

/* ----C-FRIENDLY BINDINGS GO HERE------*/

#undef EXTERNC

#endif /* INC_RINGBUFFER_H_ */
