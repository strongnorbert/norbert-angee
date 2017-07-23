/*
 * PT100MV_2_CH.c
 *
 * Created: 06.06.2015 19:06:09
 *  Author: M.N
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "twi_slave_driver.h"
#include "TWI_I2C_Global_Defines.h"
#include "AD7734.h"
#include "ieee754.h"
#include "regelparameter.h"
#include "eeprom.h"
#include "butterworth_lp.h"
#include "spi_driver.h"

/************************************************************************/
/*							 Defines                                    */
/************************************************************************/

#define TRUE	1
#define FALSE	0

/* DEBUG_MODUS = FALSE: Read and use System Parameter from EEPROM at start
 * DEBUG_MODUS = TRUE : Take the hard coded declaration values
 */
#define DEBUG_MODUS	FALSE

// Max Temperature Control set Value
#define MAX_TEMP_SET_VALUE 270.0
// Min Temperature Control set Value
#define MIN_TEMP_SET_VALUE 50.0

// Temperature Channel 1 Control Value
#define TEMP_1_DEF_CONTROL_VALUE	60.0

// Temperature Channel 2 Control Value
#define TEMP_2_DEF_CONTROL_VALUE	60.0

// Temperature Counter for timer ISR
volatile static const uint32_t  CONTROL_COUNTER_MAX_VALUE = 100;

/* PT100 Calibration Values */
// CH1 Min Voltage Value
volatile float temp1_voltage_offset_f32 = 20.53;				// unterer Wert für -30 °C
// CH1 Max Voltage Value
volatile float temp1_voltage_max_f32	= 4400.8;				// Oberetemperatur entspricht 100°C/500°C
// CH1 Min Temperature Value
volatile float temp1_min_temp_f32		= -30.0; 
// CH1 Max Temperature Value
volatile float temp1_max_temp_f32		= 500.0;

// CH2 Min Voltage Value
volatile float temp2_voltage_offset_f32 = 10.7;				// unterer Wert für 0/-20 °C
// CH2 Max Voltage Value
volatile float temp2_voltage_max_f32	= 4400.0;				// Oberetemperatur entspricht 100°C/500°C
// CH2 Min Temperature Value
volatile float temp2_min_temp_f32		= -30.0;
// CH2 Max Temperature Value
volatile float temp2_max_temp_f32		= 500.0;


/* ADC Defines */
#define ADC_MIN_MEASURE_VOLTAGE		0.0
#define ADC_MAX_MEASURE_VOLTAGE		5000.0		 // max voltage in mV
#define ADC_24_BIT_RESOLUTION		16777215.0   // muss angepasst werden

// ADC AD7734 Channels Defines
#define ADC_TEMP_1_INPUT_CHANNEL		1	// AD7734 PIN 12
#define ADC_TEMP_2_INPUT_CHANNEL		2   // AD7734 PIN 13    NC
#define ADC_TEMP_1_1_INPUT_CHANNEL		3	// AD7734 PIN16
#define	ADC_TEMP_2_1_INPUT_CHANNEL		4	// AD7734 PIN17		NC

// comment out the chosen i2c-Address for the Device
volatile uint8_t device_i2c_address;

// I2C Baudrate and Settings
#define BAUDRATE	9600
#define TWI_BAUDSETTING TWI_BAUD(32000000UL, BAUDRATE)

/************************************************************************/
/*					Global Variables                                    */
/************************************************************************/
/* twi i2c Slave */
#define I2C_CYCL_TO_WAIT	10
TWI_Slave_t twiSlave;      /*!< TWI slave module. */

/* Variables to check if there is a new Temperature Control/Calibrate Values to be writen in the EEPROM
 * Thats because its not allowed to use EEPROM-Write or read Routines in the i2c-ISR
 * That takes too long time and the i2c could crach !!!
 */
// CH1 Control Values and Parameter 
volatile uint8_t eeprom_new_temp1_values_u8	 = FALSE;
// CH1 Calobration Values and Parameter
volatile uint8_t eeprom_new_temp1_cal_u8	 = FALSE;
// CH2 Control Values and Parameter
volatile uint8_t eeprom_new_temp2_values_u8  = FALSE;
// CH2 Calibration Values and Parameter
volatile uint8_t eeprom_new_temp2_cal_u8     = FALSE;

/* Temperature values */

/* Channel 1 Temperature */   
// CH1 measured Voltage Value
volatile static float channel_1_Volt_Value_f32 = 0.0;
// CH1 calculated Temperature Value 
volatile static float temperature_1_Value_f32 = 0.0;
// CH1 Temperature Control Value
volatile static float temperature_1_control_value_f32 = (float)TEMP_1_DEF_CONTROL_VALUE;

 /* Channel 2 Temperature */
 // CH2 measured Voltage Value
volatile static float channel_2_Volt_Value_f32 = 0.0;
// CH2 calculated Temperature Value
volatile static float temperature_2_Value_f32 = 0.0;
// CH2 Temperature Control Value
volatile static float temperature_2_control_value_f32 = (float)TEMP_2_DEF_CONTROL_VALUE;

// Counter for the ISR to count a control Period of 100 half sinus period => 50 Hz for the Control Timer
volatile uint32_t Temperatur_Counter = 0;

// Measure Arrays for the Smoothing filter
#define MEASURE_ARRAY_SIZE		5
float temp1_dataInArray_u8a[MEASURE_ARRAY_SIZE] = {0};
float temp2_dataInArray_u8a[MEASURE_ARRAY_SIZE] = {0};

/************************************************************************/
/*							Functions                                    */
/************************************************************************/


/*	Low-Pass-Filter   */
float temp_1_old_f32 = 0.0;
float LPF_Beta = 0.25; // 0<ß<1

float calculate_lowPass(volatile float measured_f32, float old_f32);

// handels the arriving i2c-Messages
void twi_handleInMessage(TWI_Slave_t *twi);
// Compose a i2c - Tx Message with the given Parameter to be read from the i2c-Master 
void i2cComposeTxMessage(uint8_t systemID_u8, uint8_t variableID_u8, uint8_t dataLength_u8, uint8_t *data_u8a);
// reads afloat value from a i2c-Message
float getFloatValueFromI2cMessage(void);


void TWIC_SlaveProcessData(void);

// initializes Timer 0 for the PID-Control
void timer0_init(void);

// setes the CPU Clock Frequency to 32 MHz
void setClockTo32MHz(void);

/* ADC */
// reads the ADC Converted Voltage Value in mV for a given Channel
float getMeasureVoltage(uint8_t adcChannel);

// Smoothing filter
float  calculateMeasureValueFromArray(float *dataArray_f32a, uint8_t size_u8, float newSampleValue);

/* init gpio */
void gpio_init(void);

// Checks if that is PT100MV1 ot 2 and sets the i2c-address	
void seti2caddress(void);

 
 
int main(void)
{
	// Help Variables
    volatile uint8_t retvalue = 0;
	volatile float temp1voltageValue_f32 = 0.0, temp2voltageValue_f32 = 0.0, value_f32;
	double sum_value_64 = 0.0;
	uint8_t i_u8;
	// Set the LED pins as output pins
	PORTD.DIR |= (PIN0_bm + PIN1_bm + PIN2_bm + PIN3_bm + PIN4_bm + PIN5_bm);
	
	// When Debug Modus is not active. Set the COnfiguration values from EEPROM
	#if (DEBUG_MODUS==FALSE)
		// Temperature 1 Parameters
		temp1_voltage_offset_f32 = getEepromFloatValue(MIN_VOLT_1_EEPROM_ADDRESS);
		temp1_voltage_max_f32	 = getEepromFloatValue(MAX_VOLT_1_EEPROM_ADDRESS);
		temp1_min_temp_f32		 = getEepromFloatValue(MIN_TEMP_1_EEPROM_ADDRESS);
		temp1_max_temp_f32		 = getEepromFloatValue(MAX_TEMP_1_EEPROM_ADDRESS);
		
		// Temperature 2 Parameters
		temp2_voltage_offset_f32 = getEepromFloatValue(MIN_VOLT_2_EEPROM_ADDRESS);
		temp2_voltage_max_f32	 = getEepromFloatValue(MAX_VOLT_2_EEPROM_ADDRESS);
		temp2_min_temp_f32		 = getEepromFloatValue(MIN_TEMP_2_EEPROM_ADDRESS);
		temp2_max_temp_f32		 = getEepromFloatValue(MAX_TEMP_2_EEPROM_ADDRESS);
		
	#endif
	/*while (1)
	{
		PORTD.OUT |= (PIN0_bm + PIN1_bm + PIN2_bm + PIN3_bm + PIN4_bm + PIN5_bm);
		_delay_ms(1000);
		PORTD.OUT &= ~(PIN0_bm + PIN1_bm + PIN2_bm + PIN3_bm + PIN4_bm + PIN5_bm);
		_delay_ms(1000);
	}
	*/
	
	/* set cpu clock frequency to 32 MHz */
	setClockTo32MHz();
	
	/* init gpio */
	gpio_init();
	
	/* read DEV_SELn Values and set i2c Address */
	seti2caddress();
	
	/* init timer 0  */
	timer0_init();
	
	/* init Butterworh filter  */
	initButterworthFilter();
	
	/* init spi_per_io Interface */
	spiInitPerIO();
	
	AD7734_Reset();
	/* init AD7734 */
	retvalue = AD7734_Init();
	// hier florian  retvalue muss 1 sein
	AD7734_Setup();
	
	/* Initialize TWI slave.   */
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, TEMP_CTRL_2_I2C_ADDRESS, TWI_SLAVE_INTLVL_LO_gc);
	
	// init the standard parameter of the PID-Controllers
	Init_PID_Structures(&Temp_1_PID_Config,TEMPERATURE_1_CONTROL);
	Init_PID_Structures(&Temp_2_PID_Config,TEMPERATURE_2_CONTROL);

	// Enable LO interrupt level.
	PMIC.CTRL |= PMIC_HILVLEN_bm |PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
	/* enable global IntspiInitPerIOerrupts */
	sei();
	
	init_FIR_Filter();
	
	PORTC.DIR |= (1<<PIN2_bp);
	while(1)
	{
		sum_value_64 = 0.0;
		// Read the Temperature Value of Channel 1 x Times 1 and calculate the Mean Value
		for (i_u8 = 0; i_u8 < 10; i_u8++)
		{
			// florian in die Funktion getMeasureVoltage() reingehen
			channel_1_Volt_Value_f32 = getMeasureVoltage(ADC_TEMP_1_INPUT_CHANNEL);
			temp1voltageValue_f32 = (float)calculateButterworthFilterValue(channel_1_Volt_Value_f32,TEMP_1_FILTER_INDEX);
			// Calculate the Temperature Value of the measured Voltage Value
			value_f32 = (temp1_max_temp_f32-temp1_min_temp_f32)/(temp1_voltage_max_f32-temp1_voltage_offset_f32);
			value_f32 = value_f32*(temp1voltageValue_f32-temp1_voltage_offset_f32) + temp1_min_temp_f32;
			//temperature_2_Value_f32 = value_f32;
			sum_value_64 += value_f32;
			//sum_value_64 += (double)calculate_FIR_Filter(value_f32);
			_delay_ms(100);
			
			PORTC.OUT ^= (1<<PIN2_bp);
		}
		// Calculate the Mean Value 
		temp1voltageValue_f32 = (float)sum_value_64/10.0;
		
		// toggle Keep alive signal
		PORTD.OUT ^= (PIN1_bm + PIN3_bm);
		// calculate the smoothing filter value of the new measured temperature value
		value_f32 = calculateMeasureValueFromArray(temp1_dataInArray_u8a,MEASURE_ARRAY_SIZE,temp1voltageValue_f32);
		temperature_1_Value_f32 = value_f32;//;calculate_lowPass(value_f32,temperature_1_Value_f32);
		
		sum_value_64 = 0.0;
		// Read the Temperature Value of Channel 2 x Times 1 and calculate the Mean Value
		
		for (i_u8 = 0; i_u8 < 10; i_u8++)
		{
			// florian in die Funktion getMeasureVoltage() reingehen
			channel_2_Volt_Value_f32 = getMeasureVoltage(ADC_TEMP_2_INPUT_CHANNEL);
			temp2voltageValue_f32 = (float)calculateButterworthFilterValue(channel_2_Volt_Value_f32,TEMP_2_FILTER_INDEX);
			// Calculate the Temperature Value of the measured Voltage Value
			value_f32 = (temp2_max_temp_f32-temp2_min_temp_f32)/(temp2_voltage_max_f32-temp2_voltage_offset_f32);
			value_f32 = value_f32*(temp2voltageValue_f32-temp2_voltage_offset_f32) + temp2_min_temp_f32;
			//temperature_2_Value_f32 = value_f32;
			sum_value_64 += value_f32;
			//sum_value_64 += (double)calculate_FIR_Filter(value_f32);
			_delay_ms(100);
			
			PORTC.OUT ^= (1<<PIN2_bp);
		}
		
		// Calculate the Mean Value 
		temp2voltageValue_f32 = (float)sum_value_64/10.0;
		// calculate the smoothing filter value of the new measured temperature value
		value_f32 = calculateMeasureValueFromArray(temp2_dataInArray_u8a,MEASURE_ARRAY_SIZE,temp2voltageValue_f32);
		temperature_2_Value_f32 = value_f32;//;calculate_lowPass(value_f32,temperature_1_Value_f32);
		
	
		PORTD.OUT ^= (PIN1_bm + PIN3_bm);
		
		// Check if there are any new Values to be written in the EEPROM
		if (eeprom_new_temp1_values_u8)
		{
			// Temperature 1 PID-Parameters
			writeEepromFloatValue(temperature_1_control_value_f32,TEMP_1_SET_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_1_PID_Config.Kp,KP_1_PARAM_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_1_PID_Config.Ki,KI_1_PARAM_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_1_PID_Config.Kd,KD_1_PARAM_EEPROM_ADDRESS);
			eeprom_new_temp1_values_u8 = FALSE;
		}
		if (eeprom_new_temp1_cal_u8)
		{	// Temperature 1 Calibration-Parameters
			writeEepromFloatValue(temp1_min_temp_f32,MIN_TEMP_1_EEPROM_ADDRESS);
			writeEepromFloatValue(temp1_max_temp_f32,MAX_TEMP_1_EEPROM_ADDRESS);
			writeEepromFloatValue(temp1_voltage_offset_f32,MIN_VOLT_1_EEPROM_ADDRESS);
			writeEepromFloatValue(temp1_voltage_max_f32,MAX_VOLT_1_EEPROM_ADDRESS);
			eeprom_new_temp1_cal_u8 = false;
		}
		if(eeprom_new_temp2_values_u8)
		{
			// Temperature 2 PID-Parameters
			writeEepromFloatValue(temperature_2_control_value_f32,TEMP_2_SET_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_2_PID_Config.Kp,KP_2_PARAM_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_2_PID_Config.Ki,KI_2_PARAM_EEPROM_ADDRESS);
			writeEepromFloatValue(Temp_2_PID_Config.Kd,KD_2_PARAM_EEPROM_ADDRESS);
			eeprom_new_temp2_cal_u8 = FALSE;
		}
		if (eeprom_new_temp2_cal_u8)
		{
			// Temperature 2 Calibration-Parameters
			writeEepromFloatValue(temp2_min_temp_f32,MIN_TEMP_2_EEPROM_ADDRESS);
			writeEepromFloatValue(temp2_max_temp_f32,MAX_TEMP_2_EEPROM_ADDRESS);
			writeEepromFloatValue(temp2_voltage_offset_f32,MIN_VOLT_2_EEPROM_ADDRESS);
			writeEepromFloatValue(temp2_voltage_max_f32,MAX_VOLT_2_EEPROM_ADDRESS);
			eeprom_new_temp2_cal_u8 = FALSE;
		}
		_delay_ms(100); // check that with io and oscilloscope !!!!!
		
		
	}
    return 0;
}

 /** @fn void seti2caddress(void)
  *  @brief this function sets the i2c-Address by reading the Config-DIP-Switches
  *  @param[in] none
  *	 @return    none
  */
void seti2caddress(void)
{
	// Read device select Inputs
	uint8_t devsel_u8 = (PORTD.IN >> 6)&0x03;
	
	switch(devsel_u8)
	{
		case 0:
			device_i2c_address = TEMP_CTRL_1_I2C_ADDRESS;
		break;
		
		case 1:
			device_i2c_address = TEMP_CTRL_2_I2C_ADDRESS;
		break;
		
		case 2:
			//device_i2c_address = TEMP_CTRL_3_I2C_ADDRESS;
		break;
		
		case 3:
			//device_i2c_address = TEMP_CTRL_4_I2C_ADDRESS;
		break;
	}
}

 /** @fn void gpio_init(void)
  *  @brief this function initializes the gpio pins for Temperature Control
  *  @param[in] none
  *	 @return    none
  */
void gpio_init(void)
{
	// Set Temperature/LED Control pins as Output
	PORTD.DIR = ((1<<PIN0_bp) + (1<<PIN1_bp) + (1<<PIN2_bp) + (1<<PIN3_bp));
	// Set all values to 0
	PORTD.OUT = 0x00;
	
	// set DEV-SEL Pins as Input 
	PORTD.DIR &= ~((1<<PIN6_bp)+(1<<PIN7_bp));
}

 /** @fn float getMeasureVoltage(uint8_t adcChannel)
  *  @brief reads a voltage value from an ADC-Channel 1..4
  *  @param[in] adcChannel: Channel Number
  *	 @return    Measured Voltage Value of the selected channel
  */
float getMeasureVoltage(uint8_t adcChannel)
{
	volatile float value_f32 = 0.0;
	volatile unsigned long digitalValue_u32 = 0;
	// get the digital measure value
	digitalValue_u32 = AD7734_getChannelValue(adcChannel);
	
	// calculate the measured analog voltage value from the digital
	value_f32 = (((float)digitalValue_u32)/((float)ADC_24_BIT_RESOLUTION));
	value_f32 = value_f32*(((float)(ADC_MAX_MEASURE_VOLTAGE-ADC_MIN_MEASURE_VOLTAGE)));

	return value_f32;
}
 /** @fn void setClockTo32MHz(void)
  *  @brief this function sets the mcu clock frequency to 32MHz from the internal oscillator
  *  @param[in] none
  *	 @return    none
  */
void setClockTo32MHz(void)
{
	CCP = CCP_IOREG_gc;              // disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
	CCP = CCP_IOREG_gc;              // disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

 /** @fn void timer0_init(void)
  *  @brief this function initializes the Timer0 for Temperature Control
  *         with an overflow interrupt every 10ms (half sinus persiode of AC-Supply Voltage)
  *  @param[in] none
  *	 @return    none
  */
void timer0_init(void)
{
	// Prescaler 256 for timer 0
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
	// timer  overflow interrupt every 10ms
	TCC0.CTRLB = 0x00;
	TCC0.PER   = 1250;
	TCC0.CNT   = 0;
	TCC0.INTCTRLA = 0b00000011;
}

 /** @fn void PID_Calculate(float Sollwert,float Istwert, PID_Struct *PID_Config)
  *  @brief this function calculates the voltage control value (SSR-Output) with a PID-Controller
  *  @param[in] Sollwert: 	Set Value of Temperature
  *  @param[in] Istwert:	 	Actual measured Temperature Value
  *  @param[in] PID_Config:	Data pid structure of the temperature channel with the controller parameters
  *	@return    none
  */
void PID_Calculate(float Sollwert,float Istwert, PID_Struct *PID_Config){
	volatile float P_Anteil = 0.0;
	volatile float I_Anteil = 0.0;
	volatile float D_Anteil = 0.0;
	volatile float dErr = 0.0;
	// Die Regelabweichung berechnen
	// calculate control deviation
	PID_Config->e = Sollwert - Istwert;
	if (Sollwert!=0)
	{
		PID_Config->esum += PID_Config->e;
	}
	
	// I-Anteil muss begrenzt werden !!!
	if(PID_Config->esum > (float)Y_MAX_VALUE) PID_Config->esum = (float)Y_MAX_VALUE;
	if(PID_Config->esum <-(float)Y_MAX_VALUE) PID_Config->esum = -(float)Y_MAX_VALUE;
	
	// Calculate P-Term
	P_Anteil = (PID_Config->Kp*PID_Config->e);
	// Calculate I-Term
	I_Anteil = (PID_Config->Ki*PID_Config->Ta*PID_Config->esum);
	// Calculate D-Term
	dErr = (PID_Config->e - PID_Config->ealt);
	dErr /= PID_Config->Ta;
	D_Anteil = PID_Config->Kd*dErr;
	// Calculate Control value
	PID_Config->y = P_Anteil + I_Anteil + D_Anteil;
	// Regelabweichung für die nächste Abbtastung speichern
	PID_Config->ealt = PID_Config->e;
	// Die Stellgröße begrenzen
	if ((PID_Config->y) > (float)Y_MAX_VALUE) PID_Config->y = (float)Y_MAX_VALUE;

	if (PID_Config->y < (float)Y_MIN_VALUE) PID_Config->y = (float)Y_MIN_VALUE;
}

 /** @fn ISR(TCC0_OVF_vect)
  *  @brief this function is the Interrupt Service Routine of the Timer0 Overflow interrupt vector
  *  @param[in] TCC0_OVF_vect:   Interrupt vector
  *	@return    none
  */
ISR(TCC0_OVF_vect)
{
	
	int a = 0;
	if(Temperatur_Counter > (CONTROL_COUNTER_MAX_VALUE+1))
	{
		volatile int err = 1;
		err++;
	}
	if ((Temperatur_Counter++)>CONTROL_COUNTER_MAX_VALUE)
	{
		Temperatur_Counter = 0;
		PID_Calculate(temperature_1_control_value_f32,temperature_1_Value_f32,&Temp_1_PID_Config);
		//PID_Calculate(temperature_2_control_value_f32,temperature_2_Value_f32,&Temp_2_PID_Config);
	}

	//  Temperature (Solid-State-Relais)
	
	if(Temperatur_Counter < (uint32_t)Temp_1_PID_Config.y)
		PORTD.OUT |= (1<<PIN0_bp);
	else
		PORTD.OUT &= ~(1<<PIN0_bp);
	/*	 
	if(Temperatur_Counter < (uint32_t)Temp_2_PID_Config.y)
		PORTD.OUT |= (1<<PIN1_bp);
	else
		PORTD.OUT &= ~(1<<PIN1_bp);
	*/
}

 /** @fn void i2cComposeTxMessage(uint8_t systemID_u8, uint8_t variableID_u8, uint8_t dataLength_u8, uint8_t *data_u8a)
  *  @brief this function composes a i2c-Tx-Message with the defined application protocol
  *  @param[in] SystemID_u8:   The System ID of the Actual Subsystem
  *  @param[in] variableID_u8: The Variable id of the component
  *  @param[in] dataLength_u8: Length of the Data Field (almost 4 Bytes)
  *  @param[in] data_u8a:      char array for the composed tx-message
  *	@return    none
  */
void i2cComposeTxMessage(uint8_t systemID_u8, uint8_t variableID_u8, uint8_t dataLength_u8, uint8_t *data_u8a)
{
	uint8_t i_u8 = 0; // count variable
	// set System and variable ids
	twiSlave.sendData[I2C_SYSTEM_ID] = systemID_u8;
	twiSlave.sendData[I2C_VARIABLE_ID]  = variableID_u8;
	// copy the data content to the tx-message-array
	for(i_u8 = I2C_DATA_BYTE_1; i_u8 <(I2C_DATA_BYTE_1+dataLength_u8); i_u8++)
	{
		twiSlave.sendData[i_u8] = data_u8a[i_u8-I2C_DATA_BYTE_1];
	}
}

 /** @fn float getFloatValueFromI2cMessage(void)
  *  @brief this function reads the float value from a received Message
  *  @param[in] none
  *	 @return    the read float value
  */
float getFloatValueFromI2cMessage(void)
{
	uint8_t i_u8;
	float value_f32 = 0.0;
	// received data array 
	uint8_t dataArray32_au8[MESSAGE_DATA_DLC_32]={0};
	// copy the received data to the data array
	for(i_u8 = I2C_DATA_BYTE_1; i_u8 < (I2C_DATA_BYTE_1+MESSAGE_DATA_DLC_32);i_u8++)
	{
		dataArray32_au8[i_u8-I2C_DATA_BYTE_1] = twiSlave.receivedData[i_u8];
	}
	// unpack the array to float-value
	value_f32 = unpack754tofloat32(dataArray32_au8);
	
	return value_f32;
	
}


/** @fn void twi_handleInMessage(TWI_Slave_t *twi)
 *  @brief this function handels a received message over i2c bus
 *  @param[in] twi : Pointer to the i2c-interface
 *	@return    none
 */
void twi_handleInMessage(TWI_Slave_t *twi)
{
	volatile float SetValue = 0.0 ;
	uint32_t dataLength = 0;
	uint8_t data_u8a[MESSAGE_DATA_DLC_32] = {0};
	// Switch the system id from received message
	switch(twi->receivedData[I2C_SYSTEM_ID])
	{
		// Temperature 2 Control
		case TEMP_CTRL_2_I2C_ADDRESS:
		
		switch(twi->receivedData[I2C_VARIABLE_ID])
		{
			// get Temperature 1 measure value
			case PT100MV_GET_ACTUAL_TEMP_1_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temperature_1_Value_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 Control Value
			case PT100MV_GET_TEMP_CONTROL_1_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temperature_1_control_value_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 PID-Kp value
			case PT100MV_GET_KP_PARAMETER_1_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_1_PID_Config.Kp,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 PID-Ki value
			case PT100MV_GET_KI_PARAMETER_1_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_1_PID_Config.Ki,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 PID-Kd value
			case PT100MV_GET_KD_PARAMETER_1_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_1_PID_Config.Kd,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 min temp calibration value
			case PT100MV_GET_TEMP_1_CALIB_MIN_TEMP:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp1_min_temp_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 max temp calibration value
			case PT100MV_GET_TEMP_1_CALIB_MAX_TEMP:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp1_max_temp_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 min voltage calibration value
			case PT100MV_GET_TEMP_1_CALIB_MIN_VOLT:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp1_voltage_offset_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 1 max voltage calibration value
			case PT100MV_GET_TEMP_1_CALIB_MAX_VOLT:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp1_voltage_max_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// Temperature 2 get measure  value
			case PT100MV_GET_ACTUAL_TEMP_2_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temperature_2_Value_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 Control Value
			case PT100MV_GET_TEMP_CONTROL_2_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temperature_2_control_value_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 PID-Kp value
			case PT100MV_GET_KP_PARAMETER_2_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_2_PID_Config.Kp,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 PID-Ki value
			case PT100MV_GET_KI_PARAMETER_2_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_2_PID_Config.Ki,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 PID-Kd value
			case PT100MV_GET_KD_PARAMETER_2_VALUE:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(Temp_2_PID_Config.Kd,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 min temp. calibration value
			case PT100MV_GET_TEMP_2_CALIB_MIN_TEMP:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp2_min_temp_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 max temp. calibration value
			case PT100MV_GET_TEMP_2_CALIB_MAX_TEMP:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp2_max_temp_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 min voltage calibration value
			case PT100MV_GET_TEMP_2_CALIB_MIN_VOLT:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp2_voltage_offset_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			// get Temperature 2 max voltage calibration value
			case PT100MV_GET_TEMP_2_CALIB_MAX_VOLT:
				dataLength = MESSAGE_DATA_DLC_32;
				pack754toCharArray(temp2_voltage_max_f32,data_u8a);
				i2cComposeTxMessage(twi->receivedData[I2C_SYSTEM_ID],twi->receivedData[I2C_VARIABLE_ID],dataLength,data_u8a);
			break;
			
			// Set Temperature 1 Control Value
			case PT100MV_SET_CONTROL_1_VALUE:
				temperature_1_control_value_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp1_values_u8 = TRUE;
			break;
			// set Temperature 1 PID-Kp value
			case PT100MV_SET_KP_PARAMETER_1_VALUE:
				Temp_1_PID_Config.Kp = getFloatValueFromI2cMessage();
				eeprom_new_temp1_values_u8 = TRUE;
			break;
			// set Temperature 1 PID-Ki value
			case PT100MV_SET_KI_PARAMETER_1_VALUE:
				Temp_1_PID_Config.Ki = getFloatValueFromI2cMessage();
				eeprom_new_temp1_values_u8 = TRUE;
			break;
			// set Temperature 1 PID-Kd value
			case PT100MV_SET_KD_PARAMETER_1_VALUE:
				Temp_1_PID_Config.Kd = getFloatValueFromI2cMessage();
				eeprom_new_temp1_values_u8 = TRUE;
				
			break;
			
			// set Temperature 1 calibration min temp. value
			 case PT100MV_SET_TEMP_1_CALIB_MIN_TEMP:
				temp1_min_temp_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp1_cal_u8 = TRUE;
			 break;
			 // set Temperature 1 calibration ax temp. value
			 case PT100MV_SET_TEMP_1_CALIB_MAX_TEMP:
				temp1_max_temp_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp1_cal_u8 = TRUE;
			 break;
			 // set Temperature 1 calibration min voltage value
			 case PT100MV_SET_TEMP_1_CALIB_MIN_VOLT:
				temp1_voltage_offset_f32 = channel_1_Volt_Value_f32;
				eeprom_new_temp1_cal_u8 = TRUE;
			 break;
			 // set Temperature 1 calibration max voltage value
			 case PT100MV_SET_TEMP_1_CALIB_MAX_VOLT:
				temp1_voltage_max_f32 = channel_1_Volt_Value_f32;
				eeprom_new_temp1_cal_u8 = TRUE;
			 break;
			 
			// set Temperature 2 control value
			case PT100MV_SET_CONTROL_2_VALUE:
				temperature_2_control_value_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp2_values_u8 = TRUE;
				
			break;
			// set Temperature 2 PID-Kp Value
			case PT100MV_SET_KP_PARAMETER_2_VALUE:
				Temp_2_PID_Config.Kp = getFloatValueFromI2cMessage();
				eeprom_new_temp2_values_u8 = TRUE;
				
			break;
			// set Temperature 2 PID-Ki Value
			case PT100MV_SET_KI_PARAMETER_2_VALUE:
				Temp_2_PID_Config.Ki = getFloatValueFromI2cMessage();
				eeprom_new_temp2_values_u8 = TRUE;
				
			break;
			// set Temperature 2 PID-Kd Value
			case PT100MV_SET_KD_PARAMETER_2_VALUE:
				Temp_2_PID_Config.Kd = getFloatValueFromI2cMessage();
				eeprom_new_temp2_values_u8 = TRUE;
			break;
			// set Temperature 2 calibration min temp. value
			case PT100MV_SET_TEMP_2_CALIB_MIN_TEMP:
				temp2_min_temp_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp2_cal_u8 = TRUE;
			break;
			// set Temperature 2 calibration max temp. value
			case PT100MV_SET_TEMP_2_CALIB_MAX_TEMP:
				temp2_max_temp_f32 = getFloatValueFromI2cMessage();
				eeprom_new_temp2_cal_u8 = TRUE;
			break;
			// set Temperature 2 calibration min voltage value
			case PT100MV_SET_TEMP_2_CALIB_MIN_VOLT:
				temp2_voltage_offset_f32 = channel_2_Volt_Value_f32;
				eeprom_new_temp2_cal_u8 = TRUE;
			break;
			// set Temperature 2 calibration max voltage value
			case PT100MV_SET_TEMP_2_CALIB_MAX_VOLT:
				temp2_voltage_max_f32 = channel_2_Volt_Value_f32;
				eeprom_new_temp2_cal_u8 = TRUE;
			break;
			
			default:
			// Code
			break;
			
		}
		break;
		
		
		default:
		// Code
		break;
		
	}
}

 /** @fn float calculate_lowPass(volatile float measured_f32, float old_f32)
 *  @brief this function calculates the low pass filter value for a measures value
 *  @param[in] measured_f32 : Measured Value
 *  @param[in] old_f32 : the old measure value
 *	@return    the calculated filtered value
 */
float calculate_lowPass(volatile float measured_f32, float old_f32)
{
	float smooth_f32;
	smooth_f32 = old_f32 - (LPF_Beta * (old_f32 -measured_f32));
	return smooth_f32;
}

 /** @fn float  calculateMeasureValueFromArray(float *dataArray_f32a, uint8_t size_u8, float newSampleValue)
 *  @brief this function calculates the smooth filtered value for a new measure value
 *  @param[in] dataArray32_au8: the array with the stored measured data
 *  @param[in] size_u8 		  :	size of the data array
 *  @param[in] newSampleValue :  the new measured value
 *	@return    the calculated filtered value
 */
float  calculateMeasureValueFromArray(float *dataArray_f32a, uint8_t size_u8, float newSampleValue)
{
	uint8_t i_u8 = 0;
	float measureValue_f32 = 0.0;
	double valuesSummury_f64 = 0.0;
	for (i_u8 = (size_u8-1); i_u8 > 0; i_u8--)
	{
		dataArray_f32a[i_u8] = dataArray_f32a[i_u8-1];
	}
	dataArray_f32a[0] = newSampleValue;
	
	for (i_u8 = 0; i_u8 < size_u8; i_u8++)
	{
		valuesSummury_f64 += (double)dataArray_f32a[i_u8];
	}
	measureValue_f32 = (float)(valuesSummury_f64/((double)size_u8));
	
	return measureValue_f32;
}

/*! TWIC Slave Interrupt vector. */
 /** @fn ISR(TWIC_TWIS_vect)
 *  @brief this function is the interrupt service routine for the I2C bus on Port C
 *  @param[in] TWIC_TWIS_vect: the interrupt vector of the i2c-interface
 *	@return    none
 */
ISR(TWIC_TWIS_vect)
{
	volatile uint8_t test =0;
	if (twiSlave.interface->SLAVE.STATUS & TWI_SLAVE_APIF_bm) {
		twi_handleInMessage(&twiSlave);
	}
	TWI_SlaveInterruptHandler(&twiSlave);
}


/*! Simple function that invert the received value in the sendbuffer. This
 *  function is used in the driver and passed on as a pointer to the driver.
 */
 /** @fn void TWIC_SlaveProcessData(void)
 *  @brief This function is used in the driver and passed on as a pointer to the driver
 *  @param[in] none
 *	@return    none
 */
void TWIC_SlaveProcessData(void)
{
	uint8_t bufIndex = twiSlave.bytesReceived;
	twiSlave.sendData[bufIndex] = (~twiSlave.receivedData[bufIndex]);
}

