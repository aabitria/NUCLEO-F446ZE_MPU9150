/*
 * mpu9150.h
 *
 *  Created on: Feb 23, 2025
 *      Author: Lenovo310
 */

#ifndef INC_MPU9150_H_
#define INC_MPU9150_H_

#include "main.h"
#include "mpu9150_regs.h"
#include "i2c.h"

typedef struct imu_
{
	I2C_HandleTypeDef *hi2c;
	uint16_t dev_addr;

	/* sensitivity factor, not full-scale value */
	float gyro_sf;
	float accel_sf;

	/* raw readings */
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;

	/* offsets */
	int16_t gyro_x_offset;
	int16_t gyro_y_offset;
	int16_t gyro_z_offset;

	int16_t accel_x_offset;
	int16_t accel_y_offset;
	int16_t accel_z_offset;

	/* gyro processed readings (deg/s) */
	float   gyro_x;
	float   gyro_y;
	float   gyro_z;

	float   accel_x;
	float   accel_y;
	float   accel_z;

	float   angle_accl_x;
	float	angle_accl_y;

	float   angle_x;
	float	angle_y;
} imu;

static inline void mpu9150_write(imu *i, uint16_t reg_addr, uint8_t *buf, uint16_t buf_len)
{
	HAL_I2C_Mem_Write(i->hi2c, (i->dev_addr) << 1, reg_addr, 1, buf, buf_len, 2);
}

static inline void mpu9150_read(imu *i, uint16_t reg_addr, uint8_t *buf, uint16_t buf_len)
{
	HAL_I2C_Mem_Read(i->hi2c, (i->dev_addr) << 1, reg_addr, 1, buf, buf_len, 2);
}


void mpu9150_get_angle (imu *i);
void mpu9150_calibrate (imu *i);
void mpu9150_convert_from_raw (imu *i);
void mpu9150_read_gyro (imu *i, int16_t *dest);
void mpu9150_read_accel (imu *i, int16_t *dest);
void mpu9150_init (imu *i);


#endif /* INC_MPU9150_H_ */
