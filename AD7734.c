/*
 * AD7734.c
 *
 * Created: 13.12.2014 16:14:22
 *  Author: Moataz
 */ 



#include "AD7734.h"

/** @fn unsigned char AD7734_Ready(char ch)
 *  @brief this function writes a data array over spi
 *  @param[in] ch		: ADC channel
 *	@return : none
 */ 
unsigned char AD7734_Ready(char ch)
{
	unsigned char rdy = 0;
	ch--;
	rdy = (AD7734_GetRegisterValue( AD7734_REG_STATUS,1) & (1 << ch));
	
	return(rdy);
}

/** @fn uint8_t AD7734_Init(void)
 *  @brief this function initializes the AD7734-ADC over the spi interface
 *  @param[in] : none
 *	@return : Initialization state ,  1 = Successfull, 0 = Error
 */
uint8_t AD7734_Init(void)
{
	uint8_t status_u8 = 0x1;
	if((AD7734_GetRegisterValue(AD7734_REG_REV, 1) & 0x0F) != 0x2)
	{
		status_u8 = 0x0;
	}
	
	return(status_u8);
}

/** @fn void AD7734_Reset(void)
 *  @brief this function resetrs the AD7734 registers on start
 *  @param[in] : none
 *	@return : none
 */
void AD7734_Reset(void)
{
	unsigned char dataToSend[4] = {0xff, 0xff, 0xff, 0xff};
	spiWriteData(dataToSend,4);
	AD7734_PORT.OUT &= ~(1<<AD7734_RESET);
	_delay_ms(1000);
	AD7734_PORT.OUT |= (1<<AD7734_RESET);
}

/** @fn void AD7734_Setup(void)
 *  @brief this function configures the AD7734 Registers for Measuring
 *  @param[in] : none
 *	@return : none
 */
void AD7734_Setup(void)
{
	volatile uint8_t channelSetup_u8 = 0;
	volatile unsigned long Value = 0;
	
	/* Channel 1 Setup (AIN0)*/
	channelSetup_u8 = AD7734_CHSETUP_ENABLE | AD7734_CHRNG0_ENABLE | AD7734_CHRNG1_ENABLE ;
	// Enable Channel 1, Nominal input voltage +-10V
	AD7734_SetRegisterValue( AD7734_REG_CH1_SETUP,
	channelSetup_u8,
	1
	);
	channelSetup_u8 = AD7734_GetRegisterValue( AD7734_REG_CH1_SETUP,1);
	
	// Set channel conversion time
	AD7734_SetRegisterValue(AD7734_REG_CH1_CONVTIME,
							125 | AD7734_CHCONVTIME_CHOP,
							1
							);
	
	// Channel 2 Setup (AIN1)
	channelSetup_u8 = AD7734_CHSETUP_ENABLE | AD7734_CHRNG0_ENABLE | AD7734_CHRNG1_ENABLE ;
	// Enable Channel 1, Nominal input voltage +-10V
	AD7734_SetRegisterValue(AD7734_REG_CH2_SETUP,
							channelSetup_u8,
							1
							);
	channelSetup_u8 = AD7734_GetRegisterValue( AD7734_REG_CH2_SETUP,1);
	
	// Set channel conversion time
	AD7734_SetRegisterValue(AD7734_REG_CH2_CONVTIME,
							125 | AD7734_CHCONVTIME_CHOP,
							1
							);
	
	
	// Channel 3 Setup (AIN2)
	channelSetup_u8 = AD7734_CHSETUP_ENABLE | AD7734_CHRNG0_ENABLE | AD7734_CHRNG1_ENABLE ;
	// Enable Channel 1, Nominal input voltage +-10V
	AD7734_SetRegisterValue(AD7734_REG_CH3_SETUP,
							channelSetup_u8,
							1
							);
	channelSetup_u8 = AD7734_GetRegisterValue( AD7734_REG_CH3_SETUP,1);
	
	// Set channel conversion time
	AD7734_SetRegisterValue(AD7734_REG_CH3_CONVTIME,
							125 | AD7734_CHCONVTIME_CHOP,
							1
							);
	
	// Channel 4 Setup (AIN3)
	channelSetup_u8 = AD7734_CHSETUP_ENABLE | AD7734_CHRNG0_ENABLE | AD7734_CHRNG1_ENABLE ;
	// Enable Channel 1, Nominal input voltage +-10V
	AD7734_SetRegisterValue(AD7734_REG_CH4_SETUP,
							channelSetup_u8,
							1
							);
	channelSetup_u8 = AD7734_GetRegisterValue( AD7734_REG_CH4_SETUP,1);
	
	// Set channel conversion time
	AD7734_SetRegisterValue(AD7734_REG_CH4_CONVTIME,
							125 | AD7734_CHCONVTIME_CHOP,
							1
							);
	
}

/** @fn unsigned long AD7734_GetRegisterValue(uint8_t regAddress_u8, uint8_t size_u8)
 *  @brief this function reads an AD7734 register value
 *  @param[in] regAddress_u8:  Register Address
 *  @param[in] size_u8: Register Size
 *	@return : none
 */
unsigned long AD7734_GetRegisterValue(uint8_t regAddress_u8, uint8_t size_u8)
{
	uint8_t data_u8a[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	volatile unsigned long receivedData = 0x00;
	// compose the send message with Read Header and the register address
	data_u8a[0] = AD7734_COMM_READ |  AD7734_COMM_ADDR(regAddress_u8);
	// Set Chip Select to Low
	AD7734_PORT.OUT &= ~(1<<AD7734_CS);
	// Write the Config message
	spiWriteData(data_u8a,1);
	// Set Chip Select to High
	AD7734_PORT.OUT |= (1<<AD7734_CS);
	// wait
	//_delay_ms(500);
	_delay_ms(100);
	// set Chip Select to Low
	AD7734_PORT.OUT &= ~(1<<AD7734_CS);
	// read Data
	spiReadData(data_u8a,size_u8);
	// set Chip Select to High
	AD7734_PORT.OUT |= (1<<AD7734_CS);
	if(size_u8 == 1)
	{
		receivedData += (data_u8a[0] << 0);
	}
	if(size_u8 == 2)
	{
		receivedData += (data_u8a[0] << 8);
		receivedData += (data_u8a[1] << 0);
	}
	if(size_u8 == 3)
	{
		receivedData |= ((unsigned long)data_u8a[0] << 16);
		receivedData |= ((unsigned long)data_u8a[1] << 8);
		receivedData |= ((unsigned long)data_u8a[2] << 0);
	}
	// return read data
	return receivedData;
}

/** @fn void AD7734_SetRegisterValue(uint8_t regAddress_u8, unsigned long regValue_u32, uint8_t size_u8)
 *  @brief this function writes a register value in the AD7734 ADC
 *  @param[in] regAddress_u8:  Register Address
 *  @param[in] regValue_u32: register value, which has to be written
 *  @param[in] size_u8: Register Size
 *	@return : the register value
 */
void AD7734_SetRegisterValue(uint8_t regAddress_u8, unsigned long regValue_u32, uint8_t size_u8)
{
	uint8_t data_u8a[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	
	data_u8a[0] = AD7734_COMM_WRITE |  AD7734_COMM_ADDR(regAddress_u8);
	if(size_u8 == 1)
	{
		data_u8a[1] = (unsigned char)regValue_u32;
	}
	if(size_u8 == 2)
	{
		data_u8a[2] = (unsigned char)((regValue_u32 & 0x0000FF) >> 0);
		data_u8a[1] = (unsigned char)((regValue_u32 & 0x00FF00) >> 8);
	}
	if(size_u8 == 3)
	{
		data_u8a[3] = (unsigned char)((regValue_u32 & 0x0000FF) >> 0);
		data_u8a[2] = (unsigned char)((regValue_u32 & 0x00FF00) >> 8);
		data_u8a[1] = (unsigned char)((regValue_u32 & 0xFF0000) >> 16);
	}
	AD7734_PORT.OUT &= ~(1<<AD7734_CS);
	spiWriteData(data_u8a,size_u8+1);
	AD7734_PORT.OUT |= (1<<AD7734_CS);
}

/** @fn unsigned long AD7734_getChannelValue(uint8_t channel)
 *  @brief this function reads the digital measure value of a selected channel
 *  @param[in] channel:  selected channel
 *	@return : digital value of the measured channel
 */
unsigned long AD7734_getChannelValue(uint8_t channel)
{
	
	volatile uint8_t channelSetup_u8 = 0;
	volatile unsigned long Value = 0;
	// Continuous conversion on channel 1, enable 24 bits mode
	volatile uint8_t modeRegister  = (0x38+channel-1);
	volatile uint8_t setupRegister = (0x28+channel-1);
	volatile uint8_t convTRegister = (0x20+channel-1);
	volatile uint8_t dataRegister  = (0x08+channel-1);
	// tell the mode register to start conversation
	AD7734_SetRegisterValue( modeRegister,
	AD7734_MODE_MD(2) |
	AD7734_MODE_24BIT,
	1
	);
	_delay_ms(100);
	// wait until conversation in completed
	while((AD7734_PORT.IN & (1<<AD7734_RDY)));
	// read conversation data
	Value = AD7734_GetRegisterValue( dataRegister,3);
	return Value;
}