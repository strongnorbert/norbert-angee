/*
 * spi.h
 *
 * Created: 13.12.2014 17:01:51
 *  Author: Moataz
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <avr/delay.h>
#define F_CPU	32000000UL
// AD7734 Signals
#define AD7734_CS		PIN0_bp
#define AD7734_RESET	PIN1_bp
#define AD7734_P0		PIN2_bp
#define AD7734_SYNC		PIN3_bp
#define AD7734_RDY		PIN4_bp

// SPI PINs
#define SPI_MOSI		PIN5_bp
#define SPI_MISO		PIN6_bp
#define SPI_SCK			PIN7_bp

#define AD7734_PORT		PORTA

// initializes the gpio pins for spi 
void spiInitPerIO(void);
// writes a byte over spi
void spiWriteBytePerIO(uint8_t Byte_u8);
// reads a Byte over spi
uint8_t spiReadBytePerIO(uint8_t address_u8);
// writes a data array over spi
void spiWriteData(uint8_t *data_u8a, uint8_t byteNumber_u8);
// reads a api data array
void spiReadData(uint8_t *data_u8a, uint8_t byteNumber_u8);

#endif /* SPI_H_ */