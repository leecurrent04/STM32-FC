/*
 * ICM42688P_module.h
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_ICM42688P_MODULE_H_
#define INC_FC_IMU_ICM42688P_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

#include <FC_AHRS/AHRS_common.h>

#include <FC_AHRS/FC_IMU/ICM42688P/ICM42688P.h>
#include <FC_AHRS/FC_IMU/ICM42688P/register_map.h>

#include <FC_Serial/MiniLink/MiniLink.h>



/* Macros --------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/
typedef struct{
	icm42688p_cfg_t config;
	SCALED_IMU data;
	RAW_IMU raw;
} icm42688p_obj;


extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;


/* Functions 1 ---------------------------------------------------------------*/
void ICM42688P_convertGyroRaw2Dps(icm42688p_obj* device);
void ICM42688P_convertAccRaw2G(icm42688p_obj* device);
int ICM42688P_dataReady(icm42688p_obj* device);
int ICM42688P_get6AxisRawData(icm42688p_obj* device);
int ICM42688P_getSensitivity(icm42688p_obj* device);


/* Functions 2 ---------------------------------------------------------------*/


#endif
