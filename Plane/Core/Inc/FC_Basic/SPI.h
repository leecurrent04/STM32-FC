/*
 * SPI.h
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_SPI_H_
#define INC_FC_BASIC_SPI_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>


/* Variables -----------------------------------------------------------------*/
typedef struct {
	SPI_TypeDef* spi_interface;	// ex. SPI1
	GPIO_TypeDef* cs_port;		// ex. GPIOB
	uint32_t cs_pin;			// ex. GPIO_PIN_4
} SPI_DeviceConfig_t;


/* Functions -----------------------------------------------------------------*/
void SPI_Enable(SPI_DeviceConfig_t* spi_cfg);

void SPI_ChipSelect(SPI_DeviceConfig_t* spi_cfg);
void SPI_ChipDiselect(SPI_DeviceConfig_t* spi_cfg);

uint8_t SPI_SendByte(SPI_TypeDef* spi, uint8_t data);
unsigned char SPI1_SendByte(unsigned char data);
unsigned char SPI2_SendByte(unsigned char data);
unsigned char SPI3_SendByte(unsigned char data);


uint8_t SPI_readbyte(SPI_DeviceConfig_t* spi_cfg, uint8_t reg_addr);
void SPI_readbytes(SPI_DeviceConfig_t* spi_cfg, unsigned char reg_addr, unsigned char len, unsigned char* data);
void SPI_writebyte(SPI_DeviceConfig_t* spi_cfg, uint8_t reg_addr, uint8_t val);
void SPI_writebytes(SPI_DeviceConfig_t* spi_cfg, unsigned char reg_addr, unsigned char len, unsigned char* data);


#endif /* INC_FC_BASIC_SPI_H_ */
