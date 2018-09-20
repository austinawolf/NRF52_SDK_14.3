#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"

#define WHO_AM_I 0x75
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37

#define DMP_RATE 10
#define INV_RATE 100000


typedef struct {
	short x;
	short y;
	short z;
} Acc;

typedef struct {
	short x;
	short y;
	short z;
} Gyro;	

typedef struct {
	short x;
	short y;
	short z;
} Compass;	

typedef struct {
	uint8_t sensor_num;
	uint32_t event;	
	short gyro[3], accel[3], compass[3], sensors;
	unsigned char more;
	long quat[4];
	unsigned long sensor_timestamp;
	unsigned long compass_timestamp;
	int8_t cstatus;
	int8_t status;
} Motion;
	

struct platform_data_s {
    signed char orientation[9];
};


int imu_init(void);
int imu_init_madgwick(void);

void get_motion_data(Motion *motion);
void get_compass_data(Motion *motion);

void imu_log_fifo(void);
void imu_self_test(void);

Acc mpu_get_acc(void);
Compass mpu_get_mag(void);
Gyro mpu_get_gyro(void);

void mpu_compass_config(void);
void mpu_get_reg(uint8_t reg);
void mpu_write_reg(uint8_t reg, unsigned char data);
void mag_get_reg(uint8_t reg);
void mag_write_reg(uint8_t reg, unsigned char data);

#endif
