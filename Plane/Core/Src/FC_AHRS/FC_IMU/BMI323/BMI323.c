/*
 * BMI323.c
 * FC_AHRS/FC_IMU/BMI323/BMI323.c
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 ******************************************************************************
 *
 *  SPI Configuration
 *  CPOL = 1 (High) and CPHA = 1 (2nd)
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/BMI323/BMI323_module.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
bmi323_handle_t BMI323_Create(const bmi323_cfg_t* user_config)
{
	bmi323_handle_t handle = malloc(sizeof(bmi323_obj));

	if(0 == handle) { return 0; }

	bmi323_obj*  device = (bmi323_obj*)handle;
	SPI_DeviceConfig_t* bus = &(device->config.bus);

	device->config = *user_config;

	SPI_Enable(bus);
	SPI_ChipDiselect(bus);

	return handle;
}


uint8_t BMI323_Del(bmi323_handle_t handle)
{
	if(0 == handle) { return 2; }
	free(handle);

	return 0;
}


/*
 * @brief 초기 설정
 * @detail SPI 연결 수행, 감도 설정, offset 제거
 * 		datasheet p.15
 * 		who am i
 * 		power check
 * @retval 0 : 완료
 *         -1 : 센서 에러
 */
uint8_t BMI323_Initialization(bmi323_handle_t handle)
{
	if(0 == handle) { return 1; }

	bmi323_obj* device = (bmi323_obj*)handle;
	SPI_DeviceConfig_t bus = device->config.bus;


	uint16_t temp = 0;

	SPI_Enable(&bus);
	SPI_ChipDiselect(&bus);

	// Testing communication and initializing the device
	temp |= ((BMI323_readbyte(device, CHIP_ID)==0x43)<<0);

	// Checking the correct initialization status
	temp |= ((BMI323_readbyte(device, ERR_REG)==0x00)<<1);

	// power up detected : 0
	temp |= ((BMI323_checkNBit(device, STATUS, 0)==0)<<2);


//	 Configure the device for normal power mode
	BMI323_writebyte(device, ACC_CONF, 0x70A9);
	BMI323_writebyte(device, GYR_CONF, 0x70C9);

// Configure the device for high performance power mode
//	BMI323_writebyte(ACC_CONF, 0x4027);
//	BMI323_writebyte(GYR_CONF, 0x404B);


	return temp;
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 *         1 : isn't ready
 *         2 : sensor error
 */
uint8_t BMI323_GetData(bmi323_handle_t handle, SCALED_IMU* imu)
{
	bmi323_obj* device = (bmi323_obj*)handle;

	uint8_t retVal = BMI323_dataReady(device);
	if(retVal) return retVal;

	BMI323_get6AxisRawData(device);

	BMI323_convertGyroRaw2Dps();
	BMI323_convertAccRaw2G();

	*imu = device->data;

	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief 레지스터의 n번째 값을 확인함
 * @detail
 * @param
 * 		uint8_t addr	: 읽을 데이터의 주소
 * 		uint8_t n		: 확인할 비트
 * @retval
 * 		uint8_t 0 or 1
 */
uint16_t BMI323_checkNBit(bmi323_obj* device, uint8_t addr, uint8_t n)
{
	uint16_t temp = 0;
	temp = BMI323_readbyte(device, addr);

	return ((temp>>n)&0x1);
}


/*
 * @retval 0 : Data is ready
 *         1 : isn't ready
 *         2 : sensor error
 */
uint8_t BMI323_dataReady(bmi323_obj* device)
{
	uint16_t temp = 0;
	temp = BMI323_readbyte(device, STATUS);

	// check error
	if(temp&0xFF1F) return  0x2;

	if ( ((temp>>5)&0x03) != 0x03) return 0x1;

	return 0;
}


/*
 * @brief 6축 데이터를 레지스터 레벨에서 로딩
 * @detail SCALED_IMU2에 저장.
 * @retval None
 */
void BMI323_get6AxisRawData(bmi323_obj* device)
{
	SCALED_IMU* imu = &(device->data);

	uint16_t data[7] = {0,};

	BMI323_readbytes(device, ACC_DATA_X, sizeof(data)/sizeof(data[0]), &data[0]);

	imu->time_boot_ms = msg.system_time.time_boot_ms;

	imu->xacc = data[0];
	imu->yacc = data[1];
	imu->zacc = data[2];
	imu->xgyro = data[3];
	imu->ygyro = data[4];
	imu->zgyro = data[5];

	imu->temperature = data[6];

	return;
}


/*
 * @brief GYRO RAW를 mdps로 변환
 * @detail SCALED_IMU2에 저장.
 * 			m degree/s
 * @param none
 * @retval none
 */
void BMI323_convertGyroRaw2Dps(void)
{
//	float sensitivity;
//
//	msg.scaled_imu.time_boot_ms = msg.system_time.time_boot_ms;
//
//	// m degree
//	msg.scaled_imu.xgyro = (float)raw_imu.xgyro / sensitivity * 1000;
//	msg.scaled_imu.ygyro = (float)raw_imu.ygyro / sensitivity * 1000;
//	msg.scaled_imu.zgyro = (float)raw_imu.zgyro / sensitivity * 1000;
//
//	return;
}


/*
 * @brief Acc RAW를 mG로 변환
 * @detail SCALED_IMU2에 저장.
 * 			mG (Gauss)
 * @param none
 * @retval none
 */
void BMI323_convertAccRaw2G(void)
{
//	float sensitivity;
//
//	// mG
//	scaled_imu.xacc = (float)raw_imu.xacc / sensitivity * 1000;
//	scaled_imu.yacc = (float)raw_imu.yacc / sensitivity * 1000;
//	scaled_imu.zacc = (float)raw_imu.zacc / sensitivity * 1000;
//
//	return;
}


/* Functions 2 ---------------------------------------------------------------*/


uint16_t BMI323_readbyte(bmi323_obj* device, uint8_t reg_addr)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);

	uint16_t val=0;
	uint8_t* p = (uint8_t*)&val;

	SPI_ChipSelect(bus);

	SPI_SendByte(bus->spi_interface, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data

	p[0] = SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data
	p[1] = SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data

	SPI_ChipDiselect(bus);

	return val;
}

void BMI323_readbytes(bmi323_obj* device, uint8_t reg_addr, uint8_t len, uint16_t* data)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);
	SPI_ChipSelect(bus);

	SPI_SendByte(bus->spi_interface, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data

	for(int i=0; i<len; i++)
	{
		uint8_t* p = (uint8_t*)&data[i];
		p[0] = SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data
		p[1] = SPI_SendByte(bus->spi_interface, 0x00); //Send DUMMY to read data
	}

	SPI_ChipDiselect(bus);
	return;
}


void BMI323_writebyte(bmi323_obj* device, uint8_t reg_addr, uint16_t val)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);
	uint8_t* p = (uint8_t*)&val;

	SPI_ChipSelect(bus);

	SPI_SendByte(bus->spi_interface, reg_addr & 0x7F); //Register. MSB 0 is write instruction.

	SPI_SendByte(bus->spi_interface, p[0]); //Send Data to write
	SPI_SendByte(bus->spi_interface, p[1]); //Send Data to write

	SPI_ChipDiselect(bus);
}


