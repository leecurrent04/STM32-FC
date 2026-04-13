/*
 * BMI323/driver.h
 * FC_AHRS/FC_IMU/BMI323/driver.h
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_IMU_BMI323_DRIVER_H_
#define INC_FC_AHRS_FC_IMU_BMI323_DRIVER_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
typedef struct {
    SPI_DeviceConfig_t bus;
} bmi323_cfg_t;

struct bmi323_obj;
typedef struct bmi323_obj* bmi323_handle_t;


/* Functions -----------------------------------------------------------------*/
bmi323_handle_t BMI323_Create(const bmi323_cfg_t* user_config);
uint8_t BMI323_Del(bmi323_handle_t handle);

uint8_t BMI323_Initialization(bmi323_handle_t handle);
uint8_t BMI323_GetData(bmi323_handle_t handle, SCALED_IMU* imu);


#endif /* INC_FC_AHRS_FC_IMU_BMI323_DRIVER_H_ */
