#ifndef MPU9150_HELPER_H_
#define MPU9150_HELPER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"



typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	int status;
} Acc;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	int status;
} Gyro;	


struct platform_data_s {
    signed char orientation[9];
};


Acc mpu_get_acc(void);
Gyro mpu_get_gyro(void);

int mpu_helper_init(void);
int mpu_helper_dmp_setup(void);
int	mpu_helper_inv_setup(void);

unsigned char mpu_log_fifo(void);

#endif
