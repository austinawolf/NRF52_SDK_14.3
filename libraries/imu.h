#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"

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
	long _0;
	long _1;	
	long _2;
	long _3;
} Quaternion;

typedef struct {
	Acc acc;
	Gyro gyro;
	Quaternion quat;
	unsigned long sensor_timestamp;
	short sensors;
	unsigned char more;
	int status;
} Motion;
	

struct platform_data_s {
    signed char orientation[9];
};


int imu_init(void);
unsigned char imu_get_fifo(void);
Acc mpu_get_acc(void);
#endif
