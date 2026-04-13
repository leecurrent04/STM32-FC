/*
 * IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/IMU_module.h>

#include <FC_Basic/SPI.h>


/* Macros --------------------------------------------------------------------*/
icm42688p_handle_t icm42688p;
bmi323_handle_t bmi323;

/* Functions -----------------------------------------------------------------*/
/*
 * @brief IMU 초기화
 * @detail IMU 1 - ICM42688P : GYRO, ACC, TEMP
 * @parm none
 * @retval 0 : 정상
 * 		0bNM : N - ICM42688 err
 * 		0bNM : M - BMI323 err
 */
uint8_t IMU_Initialization(void)
{
	uint8_t err = 0;

	icm42688p_cfg_t icm42688p_configure;
	icm42688p_configure.bus.spi_interface = SPI1;
	icm42688p_configure.bus.cs_port = GYRO1_NSS_GPIO_Port;
	icm42688p_configure.bus.cs_pin = GYRO1_NSS_Pin;

	bmi323_cfg_t bmi323_configure;
	bmi323_configure.bus.spi_interface = SPI3;
	bmi323_configure.bus.cs_port = GYRO2_NSS_GPIO_Port;
	bmi323_configure.bus.cs_pin = GYRO2_NSS_Pin;

	icm42688p = ICM42688P_Create(&icm42688p_configure);
	bmi323 = BMI323_Create(&bmi323_configure);

	err |= (ICM42688P_Initialization(icm42688p)<<0);
	err |= (BMI323_Initialization(bmi323)<<1);

	IMU_CalibrateOffset();
	return err;
}


/*
 * @brief 데이터 로딩
 * @detail
 * 		필터 및 오프셋 보정된 값
 * 		SCALED_IMU(_, 2,3)에 저장
 * @parm none
 * @retval none
 */
uint8_t IMU_GetData(void)
{
	uint16_t retVal = 0;

	// SCALED_IMU
	ICM42688P_GetData(icm42688p, &(msg.scaled_imu));

	// SCALED_IMU2
	retVal = (BMI323_GetData(bmi323,(SCALED_IMU*)&msg.scaled_imu2) << 4);

	// SCALED_IMU3

	return retVal;
}



/*
 * @brief IMU 오프셋 보정
 * @detail SCALED_IMU(2,3)에 저장
 * @parm none
 * @retval none
 */
void IMU_CalibrateOffset(void)
{
	ICM42688P_CalibrateOffset(icm42688p, 10);

	return;
}
