/*
 * ICM42688P/driver.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_ICM42688P_DRIVER_H_
#define INC_FC_IMU_ICM42688P_DRIVER_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Basic/SPI.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
typedef enum {
    ICM42688P_LPF_6DB_400HZ = 0,
    ICM42688P_LPF_6DB_200HZ,
    // ...
} icm42688p_lpf_t;

typedef enum {
    ICM42688P_DRIVE_NORMAL = 0,
    ICM42688P_DRIVE_BOOST
} icm42688p_drive_mode_t;

typedef struct {
    SPI_DeviceConfig_t bus;

    struct {
        icm42688p_lpf_t lpf_config;
        icm42688p_drive_mode_t drive_mode;
        uint8_t accel_odr; // TODO : odr emum
        uint8_t gyro_odr;
    } settings;
} icm42688p_cfg_t;

struct icm42688p_obj;
typedef struct icm42688p_obj* icm42688p_handle_t;


/* Functions -----------------------------------------------------------------*/
icm42688p_handle_t ICM42688P_Create(const icm42688p_cfg_t* user_config);
uint8_t ICM42688P_Del(icm42688p_handle_t handle);

uint8_t ICM42688P_Initialization(icm42688p_handle_t handle);
uint8_t ICM42688P_GetData(icm42688p_handle_t handle, SCALED_IMU* imu);
uint8_t ICM42688P_GetRawData(icm42688p_handle_t handle, RAW_IMU* imu);
uint8_t ICM42688P_CalibrateOffset(icm42688p_handle_t handle, int samples);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
