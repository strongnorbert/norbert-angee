/*
 * ieee754.h
 *
 *  Created on: 05.07.2014
 *      Author: moataz.naserddin
 */

#ifndef IEEE754_H_
#define IEEE754_H_

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>

struct ieeeSingleCharArray{uint8_t Values[4];};

#define pack754_32(f) (pack754((f), 32, 8))
#define pack754_64(f) (pack754((f), 64, 11))
#define unpack754_32(i) (unpack754((i), 32, 8))
#define unpack754_64(i) (unpack754((i), 64, 11))

// converts and Packs a float value with the ieee574 standard to a char array
void pack754toCharArray(float f32Number, uint8_t u8ValuesArray[]);
// read a float value with the ieee754 standard from a char array
float unpack754tofloat32(uint8_t u8ValuesArray[]);

uint64_t pack754(long double f, unsigned bits, unsigned expbits);
long double unpack754(uint64_t i, unsigned bits, unsigned expbits);


#endif /* IEEE754_H_ */
