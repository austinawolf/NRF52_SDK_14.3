#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"


/* IMU CONFIG */
#define SENSOR_NUM 2
#define IMU_SAMPLE_RATE_HZ 10
#define IMU_SAMPLE_PERIOD_MS (uint32_t) 1000/IMU_SAMPLE_RATE_HZ
#define INV_QUAT_SAMPLE_RATE 10000
#define MAG_USE_CAL

/* IMU CONVERSIONS */
#define RAW_GYRO_TO_RADS (float) 2000.0f*2.0f/0xFFFFf * 3.14f/180.0f
#define RAW_ACCEL_TO_GS (float) 2.0f * 2.0f/0xFFFFf

//


typedef struct {
	uint8_t sensor_num;
	uint32_t event;	
	short gyro[3], accel[3], sensors;
	float compass[3];
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

// Offsets applied to raw x/y/z values
extern const float mag_offsets[3];
// Soft iron error compensation matrix
extern const float mag_softiron_matrix[3][3];
extern const float mag_field_strength;
										
//program biases
#define SCALE_ACC_OFFSET 1/8
#define SCALE_GYRO_OFFSET 2
extern const long accel_bias[3];
extern long gyro_bias[3];


int imu_init(void);
void imu_self_test(void);
int imu_init_madgwick(void);

void imu_get_data(Motion *motion);
void imu_get_compass(Motion *motion);
void imu_send_to_mpl(Motion *motion);
void imu_log_data(Motion *motion);
void imu_log_motion_cal(Motion *motion);


#endif
