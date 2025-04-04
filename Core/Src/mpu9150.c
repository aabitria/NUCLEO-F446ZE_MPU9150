/*
 * mpu9150.c
 *
 *  Created on: Feb 23, 2025
 *      Author: Lenovo310
 */


#include "mpu9150_regs.h"
#include "mpu9150.h"
#include "math.h"

float x_gyro = 0.0f, x_accel = 0.0f;

static void mpu9150_reset (imu *i)
{
	uint8_t data = 0x80;
	mpu9150_write(i, MPU9150_PWR_MGMT_1, &data, sizeof(uint8_t));
}


static void mpu9150_config (imu *i)
{
	uint8_t param = 0;

	mpu9150_write(i, MPU9150_PWR_MGMT_1, &param, sizeof(uint8_t));

	param = 0x3;
	mpu9150_write(i, MPU9150_SMPLRT_DIV, &param, sizeof(uint8_t));

	param = 0x1;
	mpu9150_write(i, MPU9150_CONFIG, &param, sizeof(uint8_t));

	// Set up gyro and accel config
	param = MPU9150_GC_FSEL2;
	mpu9150_write(i, MPU9150_GYRO_CONFIG, &param, sizeof(uint8_t));
	switch (param)
	{
	case MPU9150_GC_FSEL0:
		i->gyro_sf = 131.0f;
		break;

	case MPU9150_GC_FSEL1:
		i->gyro_sf = 65.5f;
		break;

	case MPU9150_GC_FSEL2:
		i->gyro_sf = 32.8f;
		break;

	case MPU9150_GC_FSEL3:
		i->gyro_sf = 16.4f;
		break;

	default:
		i->gyro_sf = 1.0f;
	}

	param = MPU9150_AC_AFSEL2;
	mpu9150_write(i, MPU9150_ACCEL_CONFIG, &param, sizeof(uint8_t));
	switch (param)
	{
	case MPU9150_AC_AFSEL0:
		i->accel_sf = 16384.0f;
		break;

	case MPU9150_AC_AFSEL1:
		i->accel_sf = 8192.0f;
		break;

	case MPU9150_AC_AFSEL2:
		i->accel_sf = 4096.0f;
		break;

	case MPU9150_AC_AFSEL3:
		i->accel_sf = 2048.0f;
		break;

	default:
		i->accel_sf = 1.0f;
	}
}


void mpu9150_read_gyro (imu *i, int16_t *dest)
{
	uint8_t raw_data[6];
	mpu9150_read(i, MPU9150_GYRO_XOUT_H, raw_data, sizeof(raw_data));
	dest[0] = (int16_t)((raw_data[0] << 8) | (raw_data[1]));
	dest[1] = (int16_t)((raw_data[2] << 8) | (raw_data[3]));
	dest[2] = (int16_t)((raw_data[4] << 8) | (raw_data[5]));
}


void mpu9150_read_accel (imu *i, int16_t *dest)
{
	uint8_t raw_data[6];
	mpu9150_read(i, MPU9150_ACCEL_XOUT_H, raw_data, sizeof(raw_data));
	dest[0] = (int16_t)((raw_data[0] << 8) | (raw_data[1]));
	dest[1] = (int16_t)((raw_data[2] << 8) | (raw_data[3]));
	dest[2] = (int16_t)((raw_data[4] << 8) | (raw_data[5]));
}


void mpu9150_convert_from_raw (imu *i)
{
	i->gyro_x_raw -= i->gyro_x_offset;
	i->gyro_y_raw -= i->gyro_y_offset;
	i->gyro_z_raw -= i->gyro_z_offset;

	i->accel_x_raw -= i->accel_x_offset;
	i->accel_y_raw -= i->accel_y_offset;
	i->accel_z_raw -= i->accel_z_offset;

	i->gyro_x = (float)i->gyro_x_raw / i->gyro_sf;
	i->gyro_y = (float)i->gyro_y_raw / i->gyro_sf;
	i->gyro_z = (float)i->gyro_z_raw / i->gyro_sf;

	i->accel_x = (float)i->accel_x_raw / i->accel_sf;
	i->accel_y = (float)i->accel_y_raw / i->accel_sf;
	i->accel_z = (float)i->accel_z_raw / i->accel_sf;
}


int16_t calib_data[3 * 128] = {0};

void mpu9150_calibrate (imu *i)
{
	int num_readings = 128;
	int32_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
	int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
	int16_t *dest = NULL;

	dest = calib_data;
	for (int idx = 0; idx < num_readings; idx++)
	{
		mpu9150_read_gyro(i, dest);
		dest += 3;
		HAL_Delay(4);
	}

	// use data from 65th sample to 128th
	for (int idx = 0; idx < 64; idx++)
	{
		dest = &calib_data[(idx + 64) * 3];
		gx_offset += (int32_t)*dest;
		gy_offset += (int32_t)*(dest + 1);
		gz_offset += (int32_t)*(dest + 2);
	}

	gx_offset >>= 6;
	gy_offset >>= 6;
	gz_offset >>= 6;

	i->gyro_x_offset = gx_offset;
	i->gyro_y_offset = gy_offset;
	i->gyro_z_offset = gz_offset;

	/* calibrate for accel */
	/* assumption is that board is lying on level field, 9150 chip up */
	dest = calib_data;
	for (int idx = 0; idx < num_readings; idx++)
	{
		mpu9150_read_accel(i, dest);
		dest += 3;
		HAL_Delay(4);
	}

	// use data from 65th sample to 128th
	for (int idx = 0; idx < 64; idx++)
	{
		dest = &calib_data[(idx + 64) * 3];
		ax_offset += (int32_t)*dest;
		ay_offset += (int32_t)*(dest + 1);
		az_offset += (int32_t)*(dest + 2) - (int32_t)i->accel_sf; 	// - 1g;
	}

	ax_offset >>= 6;
	ay_offset >>= 6;
	az_offset >>= 6;

	i->accel_x_offset = ax_offset;
	i->accel_y_offset = ay_offset;
	i->accel_z_offset = az_offset;
}

/* Complementary filter */
void mpu9150_get_angle (imu *i)
{
	const float alpha = 0.98;
	const float alpha_c = 1.0 - alpha;

	i->angle_accl_x = atan(-i->accel_x / sqrt(pow(i->accel_y, 2) + pow(i->accel_z, 2)));
	i->angle_accl_x *= 57.2958f;

	i->angle_accl_y = atan(i->accel_y / sqrt(pow(i->accel_x, 2) + pow(i->accel_z, 2)));
	i->angle_accl_y *= 57.2958f;

	i->angle_x = (alpha * (i->angle_x + i->gyro_x * 0.004)) + (alpha_c * i->angle_accl_y);
	i->angle_y = (alpha * (i->angle_y + i->gyro_y * 0.004)) + (alpha_c * i->angle_accl_x);

	x_gyro = x_gyro + (i->gyro_x * 0.004);
	x_accel = i->angle_accl_y;
	//x_accel = 57.2958f * (atan(i->accel_y / sqrt(pow(i->accel_x, 2) + pow(i->accel_z, 2))));
}

void mpu9150_init (imu *i)
{
	// reset
	mpu9150_reset(i);
    HAL_Delay(100);

	// configure
	mpu9150_config(i);
}
