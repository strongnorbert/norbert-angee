/*
 * eeprom.h
 *
 * Created: 15.05.2015 18:19:09
 *  Author: admin
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/io.h>
#include <avr/eeprom.h>


#define VARIABLE_BUFFER_SIZE 4

// Temperature Channel 1
#define TEMP_1_SET_EEPROM_ADDRESS 0x00
#define KP_1_PARAM_EEPROM_ADDRESS (TEMP_1_SET_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define KI_1_PARAM_EEPROM_ADDRESS (KP_1_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define KD_1_PARAM_EEPROM_ADDRESS (KI_1_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)

#define MIN_TEMP_1_EEPROM_ADDRESS (KD_1_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MAX_TEMP_1_EEPROM_ADDRESS (MIN_TEMP_1_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MIN_VOLT_1_EEPROM_ADDRESS (MAX_TEMP_1_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MAX_VOLT_1_EEPROM_ADDRESS (MIN_VOLT_1_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)

// Temperature Channel 2

#define TEMP_2_SET_EEPROM_ADDRESS (MAX_VOLT_1_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define KP_2_PARAM_EEPROM_ADDRESS (TEMP_2_SET_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define KI_2_PARAM_EEPROM_ADDRESS (KP_2_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define KD_2_PARAM_EEPROM_ADDRESS (KI_2_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)

#define MIN_TEMP_2_EEPROM_ADDRESS (KD_2_PARAM_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MAX_TEMP_2_EEPROM_ADDRESS (MIN_TEMP_2_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MIN_VOLT_2_EEPROM_ADDRESS (MAX_TEMP_2_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)
#define MAX_VOLT_2_EEPROM_ADDRESS (MIN_VOLT_2_EEPROM_ADDRESS + VARIABLE_BUFFER_SIZE)

/************************************************************************/
/*		 		Writes a Float Value in the EEPROM Memory               */
/************************************************************************/
void  writeEepromFloatValue(float Value, uint8_t ComponentAddress);

/************************************************************************/
/*		 	    reads a Float Value from the EEPROM Memory              */
/************************************************************************/
float getEepromFloatValue(uint8_t ComponentAddress);

#endif /* EEPROM_H_ */