/*
 * SPI.c
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Basic/SPI.h>





/* Functions -----------------------------------------------------------------*/
/*
 * @brief SPI 활성화
 * @param SPI_DeviceConfig_t* device
 * @retval None
 */
void SPI_Enable(SPI_DeviceConfig_t* spi_cfg){
	if(LL_SPI_IsEnabled(spi_cfg->spi_interface)) return;

	LL_SPI_Enable(spi_cfg->spi_interface);
	return;
}


void SPI_ChipSelect(SPI_DeviceConfig_t* spi_cfg)
{
	LL_GPIO_ResetOutputPin(spi_cfg->cs_port, spi_cfg->cs_pin);
}

void SPI_ChipDiselect(SPI_DeviceConfig_t* spi_cfg)
{
	LL_GPIO_SetOutputPin(spi_cfg->cs_port, spi_cfg->cs_pin);
}


/*
 * @brief SPI 1Byte 전송
 * @detail
 * 		1Byte를 전송하고 리턴된 값을 반환함.
 * 		4-wire SPI에서 동작.
 * @param
 * 		SPI_TypeDef* spi : SPI 주소 (ex. SPI1)
 *		uint8_t data : 송신할 데이터
 * @retval
 * 		uint8_t : 수신 데이터
 */
uint8_t SPI_SendByte(SPI_TypeDef* spi, uint8_t data)
{
	while(LL_SPI_IsActiveFlag_TXE(spi)==RESET);
	LL_SPI_TransmitData8(spi, data);

	while(LL_SPI_IsActiveFlag_RXNE(spi)==RESET);
	return LL_SPI_ReceiveData8(spi);
}

unsigned char SPI1_SendByte(unsigned char data) { SPI_SendByte(SPI1, data); }
unsigned char SPI2_SendByte(unsigned char data) { SPI_SendByte(SPI2, data); }
unsigned char SPI3_SendByte(unsigned char data) { SPI_SendByte(SPI3, data); }


uint8_t SPI_readbyte(SPI_DeviceConfig_t* spi_cfg, uint8_t reg_addr)
{
	uint8_t val;

	SPI_ChipSelect(spi_cfg);

	SPI_SendByte(spi_cfg->spi_interface, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI_SendByte(spi_cfg->spi_interface, 0x00); //Send DUMMY to read data

	SPI_ChipDiselect(spi_cfg);

	return val;
}

void SPI_readbytes(SPI_DeviceConfig_t* spi_cfg, unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	SPI_ChipSelect(spi_cfg);
	SPI_SendByte(spi_cfg->spi_interface, reg_addr | 0x80); //Register. MSB 1 is read instruction.

	for(unsigned int i=0; i < len; i++)
	{
		data[i] = SPI_SendByte(spi_cfg->spi_interface, 0x00); //Send DUMMY to read data
	}
	SPI_ChipDiselect(spi_cfg);
}

void SPI_writebyte(SPI_DeviceConfig_t* spi_cfg, uint8_t reg_addr, uint8_t val)
{
	SPI_ChipSelect(spi_cfg);

	SPI_SendByte(spi_cfg->spi_interface, reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI_SendByte(spi_cfg->spi_interface, val); //Send Data to write

	SPI_ChipDiselect(spi_cfg);
}

void SPI_writebytes(SPI_DeviceConfig_t* spi_cfg, unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	SPI_ChipSelect(spi_cfg);
	SPI_SendByte(spi_cfg->spi_interface, reg_addr & 0x7F); //Register. MSB 0 is write instruction.

	for(unsigned int i=0; i < len; i++)
	{
		SPI_SendByte(spi_cfg->spi_interface, data[i]); //Send Data to write
	}
	SPI_ChipDiselect(spi_cfg);
}

