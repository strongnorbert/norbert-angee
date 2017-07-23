/*
 * eeprom.c
 *
 * Created: 15.05.2015 18:20:11
 *  Author: admin
 */ 

#include "eeprom.h"


 /** @fn void  writeEepromFloatValue(float Value, uint8_t ComponentAddress)
  *  @brief this function writes a Float Value in the EEPROM Memory 
  *  @param[in] Value: The value to be written in the EEPROM
  *  @param[in] ComponentAddress: The EEPROM-Address of the variable
  *	 @return    none
  */
void  writeEepromFloatValue(float Value, uint8_t ComponentAddress)
{
	eeprom_write_float((float*)ComponentAddress,Value);
	}

 /** @fn float getEepromFloatValue(uint8_t ComponentAddress)
  *  @brief this function reads a Float Value from the EEPROM Memory
  *  @param[in] ComponentAddress: The EEPROM-Address of the variable
  *	 @return    the read value
  */
float getEepromFloatValue(uint8_t ComponentAddress)
{
	uint8_t DataString[VARIABLE_BUFFER_SIZE]={0};
	uint32_t DecimalValue = 0;
	float Value = 0.0;
	//eeprom_read_block((void*)DataString,(const void*)ComponentAddress,VARIABLE_BUFFER_SIZE);
	return eeprom_read_float((float*)ComponentAddress);
}