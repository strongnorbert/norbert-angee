/*
 * FIR_Filter.c
 *
 * Created: 24.12.2014 14:42:01
 *  Author: Moataz
 */ 


#include "FIR_Filter.h"

void init_FIR_Filter(void)
{
	uint8_t i_u8 = 0;
	// Filter coefficients b[0] = b_0, ..., b[nc - 1] = b_N
	b[0] = 0.0;
	b[1] = 0;
	b[2] = -0.0;
	b[3] = 0;
	b[4] = 0.0;
	b[5] = 1.0;
	b[6] = 0.0;
	b[7] = 0;
	b[8] = -0.0;
	b[9] = 0;
	b[10] = 0.0;
	
	for(i_u8 = 0; i_u8 < NC_COUNT; i_u8++)
	{
		input_buffer[i_u8] = 0;
	}
	
	
}


float calculate_FIR_Filter(float newSample_f32)
{
	uint8_t i_u8 = 0;
	float newValue_f32 = 0.0;
	// Schiebe Werte im input_buffer nach rechts
	for (i_u8 = (NC_COUNT - 1); i_u8 > 0; i_u8--)
	{
		input_buffer[i_u8] = input_buffer[i_u8 - 1];
	}
	
	// Schreibe neuen Eingangswert in Buffer
	input_buffer[0] = newSample_f32;
	// Berechne neuen Ausgangswert
	newValue_f32 = 0;
	for (i_u8 = 0; i_u8 < NC_COUNT; i_u8++)
	{
		newValue_f32 += (b[i_u8] * input_buffer[i_u8]);
	}
	
}