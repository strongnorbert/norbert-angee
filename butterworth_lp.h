/*
 * butterworth_lp.h
 *
 * Created: 29.12.2014 19:39:08
 *  Author: Moataz
 */ 


#ifndef BUTTERWORTH_LP_H_
#define BUTTERWORTH_LP_H_

#include <avr/io.h>
#include "iir.h"


#define TRUE  1
#define FALSE 0
#define NULL  0
#define BUTTERWORTH_FILTER_ORDER	8
#define SYSTEM_TOTAL_VARIABLES		4
#define TEMP_1_FILTER_INDEX			0
#define TEMP_2_FILTER_INDEX			1
#define TEMP_1_1_FILTER_INDEX		2
#define TEMP_2_1_FILTER_INDEX		4


//double m_pi = 3.14159265358979323846264338327;
int sff;          // scale flag: 1 to scale, 0 to not scale ccof
double fcf;       // cutoff frequency (fraction of pi) 2*100Hz/(500Hz*pi)
double sf;        // scaling factor
double *dcof;     // d coefficients
int *ccof;        // c coefficients

double s; // sampling frequency
double f;    // half of Power frequency
double a;
double a2;
double r;
double A[BUTTERWORTH_FILTER_ORDER];
double d1[BUTTERWORTH_FILTER_ORDER];
double d2[BUTTERWORTH_FILTER_ORDER];
double w0[SYSTEM_TOTAL_VARIABLES][BUTTERWORTH_FILTER_ORDER];
double w1[SYSTEM_TOTAL_VARIABLES][BUTTERWORTH_FILTER_ORDER];
double w2[SYSTEM_TOTAL_VARIABLES][BUTTERWORTH_FILTER_ORDER];
	
uint8_t initButterworthFilter(void);
double calculateButterworthFilterValue(double SampleValue_f64, uint8_t variableIndex_u8);



#endif /* BUTTERWORTH_LP_H_ */