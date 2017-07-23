/*
 * TWI_I2C_Global_Defines.h
 *
 * Created: 14.07.2013 19:44:26
 *  Author: Moataz.N
 */ 




#ifndef TWI_I2C_GLOBAL_DEFINES_H_
#define TWI_I2C_GLOBAL_DEFINES_H_

	
#define TEMP_CTRL_1_I2C_ADDRESS	0x20
#define TEMP_CTRL_2_I2C_ADDRESS	0x21
#define MEA_I2C_ADDRESS 0x23
#define RTC_I2C_ADDRESS 0x68
#define PRS_I2C_ADDRESS 0x77 // Pressure Sensor BMP085
#define LCD_I2C_ADDRESS 0x33
#define SIN_I2C_ADDRESS 0x44
#define PWB_I2C_ADRESS	0x0F

/*Message Size*/
#define I2C_DATA_REQUEST_DLC	 2
#define I2C_DATA_RESPONSE_DLC	 6
#define I2C_DATA_RESPONSE_DLC_32 7
#define I2C_RECEIVE_ACK_DLC		 4
#define I2C_TRANSMIT_DATA_DLC	 6
#define I2C_DATE_DATA_DLC		 12
#define I2C_TIME_DATA_DLC		 10

/* Message elements*/
#define I2C_SYSTEM_ID	   0
#define I2C_VARIABLE_ID    1
#define I2C_DATA_BYTE_1    2
#define I2C_DATA_BYTE_2    3
#define I2C_DATA_BYTE_3    4
#define I2C_DATA_BYTE_4	   5
#define I2C_ACK_BYTE       2
#define DATA_REQ_CRC_BYTE  2
#define DATA_RES_CRC_BYTE  5
#define SET_RES_ACK_BYTE   3


#define MESSAGE_DATA_DLC_32 4


#define I2C_DATE_STR_SIZE 10
#define I2C_TIME_STR_SIZE 8

#define DAC_SET_VALUE   1

#define ACK_SUCCESS     1
#define ACK_FAILURE     0
/*Temperature*/
 // get Values
// Temperature 1
 #define PT100MV_GET_ACTUAL_TEMP_1_VALUE		1
 #define PT100MV_GET_TEMP_CONTROL_1_VALUE	2
 #define PT100MV_GET_KP_PARAMETER_1_VALUE	3
 #define PT100MV_GET_KI_PARAMETER_1_VALUE	4
 #define PT100MV_GET_KD_PARAMETER_1_VALUE	5

// Calibration Values
#define PT100MV_GET_TEMP_1_CALIB_MIN_TEMP	6
#define PT100MV_GET_TEMP_1_CALIB_MAX_TEMP	7
#define PT100MV_GET_TEMP_1_CALIB_MIN_VOLT	8
#define PT100MV_GET_TEMP_1_CALIB_MAX_VOLT	9
 
// Temperature 2
 #define PT100MV_GET_ACTUAL_TEMP_2_VALUE	10
 #define PT100MV_GET_TEMP_CONTROL_2_VALUE	11
 #define PT100MV_GET_KP_PARAMETER_2_VALUE	12
 #define PT100MV_GET_KI_PARAMETER_2_VALUE	13
 #define PT100MV_GET_KD_PARAMETER_2_VALUE	14
 
 #define PT100MV_GET_TEMP_2_CALIB_MIN_TEMP	15
 #define PT100MV_GET_TEMP_2_CALIB_MAX_TEMP	16
 #define PT100MV_GET_TEMP_2_CALIB_MIN_VOLT	17
 #define PT100MV_GET_TEMP_2_CALIB_MAX_VOLT	18

 // Set Values
 // Temperature 1
 #define PT100MV_SET_CONTROL_1_VALUE       127
 #define PT100MV_SET_KP_PARAMETER_1_VALUE  128
 #define PT100MV_SET_KI_PARAMETER_1_VALUE  129
 #define PT100MV_SET_KD_PARAMETER_1_VALUE  130

 #define PT100MV_SET_TEMP_1_CALIB_MIN_TEMP	131
 #define PT100MV_SET_TEMP_1_CALIB_MAX_TEMP	132
 #define PT100MV_SET_TEMP_1_CALIB_MIN_VOLT	133
 #define PT100MV_SET_TEMP_1_CALIB_MAX_VOLT	134
 
// Temperature 2
 #define PT100MV_SET_CONTROL_2_VALUE       140
 #define PT100MV_SET_KP_PARAMETER_2_VALUE  141
 #define PT100MV_SET_KI_PARAMETER_2_VALUE  142
 #define PT100MV_SET_KD_PARAMETER_2_VALUE  143

 #define PT100MV_SET_TEMP_2_CALIB_MIN_TEMP	144
 #define PT100MV_SET_TEMP_2_CALIB_MAX_TEMP	145
 #define PT100MV_SET_TEMP_2_CALIB_MIN_VOLT	146
 #define PT100MV_SET_TEMP_2_CALIB_MAX_VOLT	147
 
 
/* Sinus Generator */
// get Values
#define SG_GET_VAR_SEN_PP_VALUE			1
#define SG_GET_VAR_REF_PP_VALUE			2
// set Values
#define SG_SET_VAR_SEN_PP_VALUE			127
#define SG_SET_VAR_REF_PP_VALUE			128

/* ERROR Defines*/
#define BAD_SYSTEM_ID		0x00
#define BAD_VARIABLE_ID		0x00
/* Destination IDs for Send Messages to the Master*/
#define I2C_TEMPERATURE_VALUE     1


/*Measure Amplifier*/
// Variables
// Sensor
#define MA_GET_REDC_VALUE		1
#define MA_GET_SIGNAL_VALUE		2
#define MA_GET_MEDC_VALUE		3
#define MA_GET_OFFSET_VALUE		4
#define MA_GET_VOL_PROC_VALUE	5
#define MA_GET_GAIN_VALUE		6
#define MA_GET_OFFSET_VALUE		7
// Set Values
#define MA_SET_GAIN_VALUE		127
#define MA_SET_OFFSET_VALUE		128


/*RTC*/
#define RTC_DATE_VALUE			1
#define RTC_TIME_VALUE			2

/*Pressure */
#define PRESSURE_GET_VALUE		1

/* USER PC */
#define SERIAL_HAND_CHECK_VARIABLE_ID  1

#endif /* TWI_I2C_GLOBAL_DEFINES_H_ */