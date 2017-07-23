/*
 * iir.h
 *
 * Created: 29.12.2014 19:29:41
 *  Author: Moataz
 */ 




#ifndef IIR_H_
#define IIR_H_

#define _USE_MATH_DEFINES
#include <math.h>

#define NULL 0

double *binomial_mult( int n, double *p );
double *trinomial_mult( int n, double *b, double *c );

double *dcof_bwlp( int n, double fcf );
double *dcof_bwhp( int n, double fcf );
double *dcof_bwbp( int n, double f1f, double f2f );
double *dcof_bwbs( int n, double f1f, double f2f );

int *ccof_bwlp( int n );
int *ccof_bwhp( int n );
int *ccof_bwbp( int n );
double *ccof_bwbs( int n, double f1f, double f2f );

double sf_bwlp( int n, double fcf );
double sf_bwhp( int n, double fcf );
double sf_bwbp( int n, double f1f, double f2f );
double sf_bwbs( int n, double f1f, double f2f );

#endif /* IIR */


