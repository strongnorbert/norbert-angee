/*
 * regelparameter.c
 *
 * Created: 29.12.2013 10:03:14
 *  Author: M.Naserddin
 */ 


#include "regelparameter.h"
#include "TWI_I2C_Global_Defines.h"

#define TA_TIME      1000.0   // Sample time . it has to be a multiple decimal value of 10ms

 /** @fn void Init_PID_Structures(PID_Struct *PID_Config, uint8_t operationDevice_u8)
  *  @brief this function initializes a PID-Data structure with the controller parameters
  *  @param[in] PID_Config: Pointer to the PID-structure of the selected temperature channel
  *  @param[in] operationDevice_u8: Temperature channel
  *	 @return    none
  */
 void Init_PID_Structures(PID_Struct *PID_Config, uint8_t operationDevice_u8){
     //Version 1 -> Heizelement (HEAT) nicht im Gehäuse
	 //PID_Config->Ta   = (TA_TIME)/1000;
	 //PID_Config->Ki   = 0.23;
	 //PID_Config->Kp   = 3.1;	// = Kr
	 switch(operationDevice_u8)
	 {
		 //  Parameter Sensor/Body 
		 case TEMPERATURE_2_CONTROL:
			PID_Config->Ta   = (TA_TIME)/1000.0;
			PID_Config->Ki   = 0.175;
			PID_Config->Kp   = 4.2;	// = Kr
			PID_Config->Kd   = 0.05;
			PID_Config->ealt = 0.0;
			PID_Config->e    = 0.0;
			PID_Config->esum = 0.0;
			PID_Config->y    = 0.0;	// n(0-99) Pulse von 100 Pulse/Sek 
		break;
		//Parameter Referenzkammer
		case TEMPERATURE_1_CONTROL:
			PID_Config->Ta   = (TA_TIME)/1000.0;
			PID_Config->Ki   = 0.31;
			PID_Config->Kp   = 10.4;	// = Kr
			PID_Config->Kd   = 1.5;
			PID_Config->ealt = 0.0;
			PID_Config->e    = 0.0;
			PID_Config->esum = 0.0;
			PID_Config->y    = 0.0;	//n(0-99) Pulse von 100 Pulse/Sek
		break;
	 }
	 
	 // Nach dem Einstellen/Schreiben der Parameter die Zeilen auskommentieren
	 
	 /*
	 writeEepromFloatValue(PID_Config->Ki,KI_PARAM_EEPROM_ADDRESS);
	 writeEepromFloatValue(PID_Config->Kp,KP_PARAM_EEPROM_ADDRESS);
	 writeEepromFloatValue(PID_Config->Kd,KD_PARAM_EEPROM_ADDRESS);
	 
	 PID_Config->Ta   = (TA_TIME)/1000;
	 PID_Config->Ki = getEepromFloatValue(KI_PARAM_EEPROM_ADDRESS);
	 PID_Config->Kp = getEepromFloatValue(KP_PARAM_EEPROM_ADDRESS);
	 PID_Config->Kd = getEepromFloatValue(KD_PARAM_EEPROM_ADDRESS);
	 PID_Config->ealt = 0.0;
	 PID_Config->e    = 0.0;
	 PID_Config->esum = 0.0;
	 PID_Config->y    = 11.0;	// n(0-99) Pulse von 100 Pulse/Sek
	*/
	 
 }