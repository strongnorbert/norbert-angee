/*
 * spi_per_io.c
 *
 * Created: 23.12.2014 12:15:45
 *  Author: Moataz
 */ 
#include "spi_per_io.h"


/** @fn void spiInitPerIO(void)
 *  @brief this function initializes the gpio pins for spi
 *  @param[in] none
 *	@return none
 */
void spiInitPerIO(void)
{
	// set the direction for the ouput pins
	AD7734_PORT.DIR = ((1<<AD7734_RESET)|(1<<AD7734_P0)|(1<<AD7734_SYNC)|(1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<AD7734_CS));
	// set the direction for the input pins
	AD7734_PORT.DIR &= ~((1<<AD7734_RDY)+(1<<SPI_MISO));
	//DDRA = (1<<AD7734_CS);
	// set RESET,
	AD7734_PORT.OUT |= (1<<AD7734_RESET) | (1<<SPI_SCK) | (1<<AD7734_CS);
	
}

/** @fn void spiWriteBytePerIO(uint8_t Byte_u8)
 *  @brief this function writes a data byte over spi
 *  @param[in] the data byte
 *	@return none
 */
void spiWriteBytePerIO(uint8_t Byte_u8)
{
	uint8_t i_u8 = 0;
	uint8_t temp_u8 = 0x80;
	// set clk to high
	AD7734_PORT.OUT |= (1<<SPI_SCK);
	// set mosi to low
	AD7734_PORT.OUT &= ~(1<<SPI_MOSI);
	for (i_u8 = 0; i_u8 < 8;i_u8++)
	{
		// set clk to low
		AD7734_PORT.OUT &= ~(1<<SPI_SCK);
		_delay_us(3000);
		// write bit
		if ((Byte_u8&temp_u8)==0)
		{
			AD7734_PORT.OUT &= ~(1<<SPI_MOSI);
		} 
		else
		{
			AD7734_PORT.OUT |= (1<<SPI_MOSI);
		}
		// shift the byte 1 bit to left
		temp_u8 = temp_u8 >> 1;
		// set sck to high
		AD7734_PORT.OUT |= (1<<SPI_SCK);
		_delay_us(3000);
	}
	
}

/** @fn uint8_t spiReadBytePerIO(uint8_t address_u8)
 *  @brief this function read a data byte from spi
 *  @param[in] address_u8: the spi chip select address
 *	@return : the read byte
 */
uint8_t spiReadBytePerIO(uint8_t address_u8)
{
	volatile uint8_t i_u8 = 0;
	volatile uint8_t data_u8 = 0;
	volatile uint8_t testdata;
	
	// set MOSI pin to low
	AD7734_PORT.OUT &= ~(1<<SPI_MOSI);
	for (i_u8 = 0; i_u8 < 8; i_u8++)
	{
		// set clk pin to low
		AD7734_PORT.OUT &= ~(1<<SPI_SCK);
		_delay_us(3000);
		testdata = AD7734_PORT.IN ;
		// read MISO pin value
		if (AD7734_PORT.IN & (1<<SPI_MISO))
		{
			data_u8 <<= 1;
			data_u8 += 0x01;
		} 
		else
		{
			data_u8 <<= 1;
			data_u8 &= 0xfe;
		}
		// set spi clk pin to high
		AD7734_PORT.OUT |= (1<<SPI_SCK);
		_delay_us(3000);
	}
	return data_u8;
}

/** @fn void spiWriteData(uint8_t *data_u8a, uint8_t byteNumber_u8)
 *  @brief this function writes a data array over spi
 *  @param[in] data_u8a		: the data array
 *  @param[in] byteNumber_u8: the data size
 *	@return : none
 */ 
void spiWriteData(uint8_t *data_u8a, uint8_t byteNumber_u8)
{
	uint8_t writeData[4]	= {0, 0, 0, 0};
	uint8_t readData[4]		= {0, 0, 0, 0};
	uint8_t i_u8 = 0;
	
	//PORTB &= ~(1<<AD7734_CS);
	for(i_u8 = 0;i_u8 < byteNumber_u8;i_u8 ++)
	{
		spiWriteBytePerIO(data_u8a[i_u8]);
	}
	//PORTB |= (1<<AD7734_CS);
}

/** @fn void spiReadData(uint8_t *data_u8a, uint8_t byteNumber_u8)
 *  @brief this function reads a data array from spi
 *  @param[in] data_u8a		: the data array
 *  @param[in] byteNumber_u8: the data size
 *	@return : none
 */ 
void spiReadData(uint8_t *data_u8a, uint8_t byteNumber_u8)
{
	uint8_t i_u8;
	//PORTB &= ~(1<<AD7734_CS);
	for (i_u8 = 0; i_u8 < byteNumber_u8 ; i_u8++)
	{
		data_u8a[i_u8] = spiReadBytePerIO(0xff);
	}
	//PORTB |= (1<<AD7734_CS);
}