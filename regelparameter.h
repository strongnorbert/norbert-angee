/*
 * regelparameter.h
 *
 * Created: 29.12.2013 10:03:14
 *  Author: M.Naserddin
 */ 


#ifndef REGELPARAMETER_H_
#define REGELPARAMETER_H_
#include <avr/io.h>

#include "EEPROM.h"

#define Y_MAX_VALUE    70.0   // Maximale Stellgröße
#define Y_MIN_VALUE     0.0   // Minimale Stellgröße
#define TA_TIME      1000.0   // Abtastzeit in ms MUSS ein n-Faches von 10 sein

#define TEMPERATURE_1_CONTROL 1
#define TEMPERATURE_2_CONTROL 2

typedef struct {
	float   Ta;			// Sample Time in ms
	float   Kp;			// Proportional Amplifying P-Term
	float   Ki;			// Integral Constant for   I-Term
	float   Kd;			// Differential Constant   D-Term
	float    e;			// Control Deviation
	float esum;			// Summer of control deviations
	float ealt;			// Deviation on Time  k-1
	float    y;			// control value
}PID_Struct;


 static volatile PID_Struct Temp_1_PID_Config;
 static volatile PID_Struct Temp_2_PID_Config;

void Init_PID_Structures(PID_Struct *PID_Config, uint8_t operationDevice_u8);

#endif /* REGELPARAMETER_H_ */