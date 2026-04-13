/*
 * ICM42688P.c
 * FC_AHRS/FC_IMU/ICM42688P/ICM42688P.c
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/ICM42688P/ICM42688P_module.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
icm42688p_handle_t ICM42688P_Create(const icm42688p_cfg_t* user_config)
{
	icm42688p_handle_t handle = malloc(sizeof(icm42688p_obj));

	if(0 == handle) { return 0; }

	icm42688p_obj*  device = (icm42688p_obj*)handle;
	device->config = *user_config;

	return handle;
}


uint8_t ICM42688P_Del(icm42688p_handle_t handle)
{
	if(0 == handle) { return 1; }
	free(handle);

	return 0;
}


/*
 * @brief 초기 설정
 * @detail SPI 연결 수행, 감도 설정, offset 제거
 * @retval 0 : 완료
 * @retval 1 : 센서 없음
 */
uint8_t ICM42688P_Initialization(icm42688p_handle_t handle)
{
	icm42688p_obj* device = (icm42688p_obj*)handle;
	SPI_DeviceConfig_t* bus = &(device->config.bus);

	SPI_Enable(bus);
	SPI_ChipDiselect(bus);

	if(SPI_readbyte(bus, WHO_AM_I) != 0x47) { return 1; }

	// PWR_MGMT0
	SPI_writebyte(bus, PWR_MGMT0, 0x0F); // Temp on, ACC, GYRO LPF Mode
	HAL_Delay(50);

	// GYRO_CONFIG0
	SPI_writebyte(bus, GYRO_CONFIG0, 0x26); // Gyro sensitivity 1000 dps, 1kHz
	HAL_Delay(50);
	SPI_writebyte(bus, GYRO_CONFIG1, 0x00); // Gyro temp DLPF 4kHz, UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	SPI_writebyte(bus, ACCEL_CONFIG0, 0x46); // Acc sensitivity 4g, 1kHz
	HAL_Delay(50);
	SPI_writebyte(bus, ACCEL_CONFIG1, 0x00); // Acc UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	SPI_writebyte(bus, GYRO_ACCEL_CONFIG0, 0x11); // LPF default max(400Hz,ODR)/4
	HAL_Delay(50);

	ICM42688P_getSensitivity(device);

	return 0; //OK
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 */
uint8_t ICM42688P_GetData(icm42688p_handle_t handle)
{
	icm42688p_obj* device = (icm42688p_obj*)handle;

	// Check data is ready
	if(ICM42688P_dataReady(device)) return 1;

	ICM42688P_get6AxisRawData(device);

	device->data.time_boot_ms = msg.system_time.time_boot_ms;

	ICM42688P_convertGyroRaw2Dps(device);
	ICM42688P_convertAccRaw2G(device);

	return 0;
}


/*
 * @brief Offset 캘리브레이션
 * @detail
 * @param none
 * @retval none
 */
uint8_t ICM42688P_CalibrateOffset(icm42688p_handle_t handle, int samples)
{
	icm42688p_obj* device = (icm42688p_obj*)handle;
	SPI_DeviceConfig_t* bus = &(device->config.bus);
	int16_t offset[6] = {0,};

	for(int cnt=0; cnt<samples; cnt++)
	{
		if(ICM42688P_get6AxisRawData(device)){ continue; }

		offset[0] += device->raw.xacc;
		offset[1] += device->raw.yacc;
		offset[2] += device->raw.zacc;
		offset[3] += device->raw.xgyro;
		offset[4] += device->raw.ygyro;
		offset[5] += device->raw.zgyro;
	}

	offset[0] *= (-1/samples);
	offset[1] *= (-1/samples);
	offset[2] *= (-1/samples);
	offset[3] *= (-1/samples);
	offset[4] *= (-1/samples);
	offset[5] *= (-1/samples);

	SPI_writebyte(bus, OFFSET_USER0, offset[0]&0xFF);
	SPI_writebyte(bus, OFFSET_USER1, ( ((offset[0]>>8)&0xF) | ((offset[1]>>8)&0xF)<<4) );
	SPI_writebyte(bus, OFFSET_USER2, offset[1]&0xFF);
	SPI_writebyte(bus, OFFSET_USER3, offset[2]&0xFF);
	SPI_writebyte(bus, OFFSET_USER4, ( ((offset[2]>>8)&0xF) | ((offset[3]>>8)&0xF)<<4) );
	SPI_writebyte(bus, OFFSET_USER5, offset[3]&0xFF);
	SPI_writebyte(bus, OFFSET_USER6, offset[4]&0xFF);
	SPI_writebyte(bus, OFFSET_USER7, ( ((offset[4]>>8)&0xF) | ((offset[5]>>8)&0xF)<<4));
	SPI_writebyte(bus, OFFSET_USER8, offset[5]&0xFF);


	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief GYRO RAW를 m rad/s로 변환
 * @detail SCALED_IMU에 저장.
 * 			m rad/s
 * @param none
 * @retval none
 */
void ICM42688P_convertGyroRaw2Dps(icm42688p_obj* device)
{
	SCALED_IMU* imu = &(device->data);
	float sensitivity = param.ins.imu0.gyro.sensitivity;

	// m degree
	imu->xgyro = (int16_t)(DEG2RAD(msg.raw_imu.xgyro/sensitivity)*1000 + 0.5f);
	imu->ygyro = (int16_t)(DEG2RAD(msg.raw_imu.ygyro/sensitivity)*1000 + 0.5f);
	imu->zgyro = (int16_t)(DEG2RAD(msg.raw_imu.zgyro/sensitivity)*1000 + 0.5f);

	return;
}



/*
 * @brief Acc RAW를 mG로 변환
 * @detail SCALED_IMU에 저장.
 * 			mG (9.8m/s^2)
 * @param none
 * @retval none
 */
void ICM42688P_convertAccRaw2G(icm42688p_obj* device)
{
	SCALED_IMU* imu = &(device->data);
	float sensitivity = param.ins.imu0.acc.sensitivity;

	// mG
	imu->xacc = (int16_t)(msg.raw_imu.xacc/sensitivity * 1000 + 0.5f);
	imu->yacc = (int16_t)(msg.raw_imu.yacc/sensitivity * 1000 + 0.5f);
	imu->zacc = (int16_t)(msg.raw_imu.zacc/sensitivity * 1000 + 0.5f);

	return;
}


/*
 * @brief 6축 데이터를 레지스터 레벨에서 로딩
 * @detail RAW_IMU에 저장
 * @retval 0
 */
int ICM42688P_get6AxisRawData(icm42688p_obj* device)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);
	RAW_IMU* imu = &(device->raw);

	uint8_t data[14];

	SPI_readbytes(bus, TEMP_DATA1, sizeof(data)/sizeof(data[0]), data);

	imu->time_usec = msg.system_time.time_unix_usec;
	imu->temperature = (data[0] << 8) | data[1];
	imu->xacc = (data[2] << 8) | data[3];
	imu->yacc = (data[4] << 8) | data[5];
	imu->zacc = ((data[6] << 8) | data[7]);
	imu->xgyro = ((data[8] << 8) | data[9]);
	imu->ygyro = ((data[10] << 8) | data[11]);
	imu->zgyro = ((data[12] << 8) | data[13]);
	imu->id = 0;

	return 0;
}


/*
 * @brief 민감도 값 로드
 * @detail 레지스터로부터 로드
 * @retval 0 : 완료
 */
int ICM42688P_getSensitivity(icm42688p_obj* device)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);
	float sensitivity;

	uint8_t gyro_reg_val = SPI_readbyte(bus, GYRO_CONFIG0);
	uint8_t gyro_fs_sel = (gyro_reg_val >> 5) & 0x07;

	uint8_t acc_reg_val = SPI_readbyte(bus, ACCEL_CONFIG0);
	uint8_t acc_fs_sel = (acc_reg_val >> 5) & 0x07;

	switch (gyro_fs_sel)
	{
	case 0: sensitivity = 16.4f; break;       // ±2000 dps
	case 1: sensitivity = 32.8f; break;       // ±1000 dps
	case 2: sensitivity = 65.5f; break;       // ±500 dps
	case 3: sensitivity = 131.0f; break;      // ±250 dps
	case 4: sensitivity = 262.0f; break;      // ±125 dps
	case 5: sensitivity = 524.3f; break;      // ±62.5 dps
	case 6: sensitivity = 1048.6f; break;     // ±31.25 dps
	case 7: sensitivity = 2097.2f; break;     // ±15.625 dps
	default: sensitivity = 16.4f; break;      // fallback: ±2000 dps
	}
	param.ins.imu0.gyro.sensitivity = sensitivity;

	switch (acc_fs_sel)
	{
	case 0: sensitivity = 2048.0f; break;    // ±16g
	case 1: sensitivity = 4096.0f; break;    // ±8g
	case 2: sensitivity = 8192.0f; break;    // ±4g
	case 3: sensitivity = 16384.0f; break;   // ±2g
	default: sensitivity = 2048.0f; break;   // fallback: ±16g
	}
	param.ins.imu0.acc.sensitivity = sensitivity;

	return 0;
}


/*
 * @brief 데이터가 준비되었는지 확인
 * @detail
 * 		ICM42688_
 * 		값 수신하기 전 확인
 * @retval 0 : 완료
 */
int ICM42688P_dataReady(icm42688p_obj* device)
{
	SPI_DeviceConfig_t* bus = &(device->config.bus);

	uint8_t temp = 0;
	temp =SPI_readbyte(bus, INT_STATUS);

	if((temp>>3)&0x01) return 0;

	return 1;
}


/* Functions 2 ---------------------------------------------------------------*/


