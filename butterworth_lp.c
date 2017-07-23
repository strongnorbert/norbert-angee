/*
 * butterworth_lp.c
 *
 * Created: 29.12.2014 19:32:05
 *  Author: Moataz
 */ 

#include "butterworth_lp.h"


 /** @fn uint8_t initButterworthFilter(void)
  *  @brief this function initializes the Butterwoth filter
  *  @param[in] none
  *	 @return    initialization status
  */
uint8_t initButterworthFilter(void)
{
	uint8_t status_u8 = TRUE;
	uint8_t i_u8;

	//double m_pi = 3.14159265358979323846264338327;
	sff = 1;          // scale flag: 1 to scale, 0 to not scale ccof
	fcf = (2*5.0)/(30.0);       // cutoff frequency (fraction of pi) 2*100Hz/(500Hz*pi)
	sf = sf_bwlp(BUTTERWORTH_FILTER_ORDER, fcf);        // scaling factor
	s = 100.0;
	//s = 100.0; // sampling frequency
	f = 5.0;    // half of Power frequency
	a = tan((M_PI*f / s));
	a2 = a*a;
	
	/* calculate the d coefficients */
	dcof = dcof_bwlp(BUTTERWORTH_FILTER_ORDER, fcf);
	if (dcof == NULL)
	{
		return FALSE;
	}
	

	/* calculate the c coefficients */
	ccof = ccof_bwlp(BUTTERWORTH_FILTER_ORDER);
	if (ccof == NULL)
	{
		return FALSE;
	}

	sf = sf_bwlp(BUTTERWORTH_FILTER_ORDER, fcf); /* scaling factor for the c coefficients */


	for (i_u8 = 0; i_u8<BUTTERWORTH_FILTER_ORDER; ++i_u8){
		r = sin(M_PI*(2.0*i_u8 + 1.0) / (4.0*BUTTERWORTH_FILTER_ORDER));
		s = a2 + 2.0*a*r + 1.0;
		A[i_u8] = a2 / s;
		d1[i_u8] = 2.0*(1 - a2) / s;
		d2[i_u8] = -(a2 - 2.0*a*r + 1.0) / s;
	}

	return status_u8;
}

 /** @fn double calculateButterworthFilterValue(double SampleValue_f64, uint8_t variableIndex_u8)
  *  @brief this function calculates the filtered value with butterworth LP for the selected index
  *  @param[in] SampleValue_f64: The new measured Value
  *  @param[in] variableIndex_u8: Index of the measure variable
  *	 @return    the calculated filtered value
  */
double calculateButterworthFilterValue(double SampleValue_f64, uint8_t variableIndex_u8)
{

	uint8_t i_u8;
	double filteredValue_f64 = SampleValue_f64;
	for (i_u8 = 0; i_u8<BUTTERWORTH_FILTER_ORDER; ++i_u8){
		w0[variableIndex_u8][i_u8] = d1[i_u8] * w1[variableIndex_u8][i_u8] + d2[i_u8] * w2[variableIndex_u8][i_u8] + filteredValue_f64;
		filteredValue_f64 = A[i_u8] * (w0[variableIndex_u8][i_u8] + 2.0*w1[variableIndex_u8][i_u8] + w2[variableIndex_u8][i_u8]);
		w2[variableIndex_u8][i_u8] = w1[variableIndex_u8][i_u8];
		w1[variableIndex_u8][i_u8] = w0[variableIndex_u8][i_u8];
	}

	return filteredValue_f64;

}