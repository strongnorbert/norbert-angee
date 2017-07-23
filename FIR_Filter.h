/*
 * FIR_Filter.h
 *
 * Created: 24.12.2014 14:40:16
 *  Author: Moataz
 */ 


#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#include <avr/io.h>

#define NC_COUNT 11  // count of filter coefficient

float b[NC_COUNT], input_buffer[NC_COUNT];

void init_FIR_Filter(void);

float calculate_FIR_Filter(float newSample_f32);

#endif /* FIR_FILTER_H_ */