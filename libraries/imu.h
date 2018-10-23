#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"
#include "config.h"

/* IMU CONFIG */
#define IMU_SAMPLE_PERIOD_MS (uint32_t) 1000/IMU_SAMPLE_RATE_HZ
#define INV_QUAT_SAMPLE_RATE 10000
#define MAG_USE_CAL

/* IMU CONVERSIONS */
#define RAW_GYRO_TO_RADS (float) 2000.0f*2.0f/0xFFFFf * 3.14f/180.0f
#define RAW_ACCEL_TO_GS (float) 2.0f * 2.0f/0xFFFFf
#define RAW_MAG_TO_uT (float) 4800.0f*2.0f/(0x4000f)

/* IMU CAL */

#if RUN_CAL == 1
	//mpu calibration data	
	#define GYRO_BIAS 	{	  0		*	SCALE_GYRO_OFFSET, \
							  0		*	SCALE_GYRO_OFFSET, \
							  0		*	SCALE_GYRO_OFFSET}
	#define ACCEL_BIAS 	{	  0		*	SCALE_ACC_OFFSET, \
							  0		*	SCALE_ACC_OFFSET, \
							  0		*	SCALE_ACC_OFFSET}	
	#define MAG_SOFTIRON_MATRIX		{{ 1, 0, 0 },	\
									{  0, 1, 0 },	\
									{  0, 0, 1 }}
	#define MAG_OFFSETS { 0, 0, 0 }
	#define MAG_FIELD_STRENTH 1f
#elif SENSOR_NUM == 1
	//mpu calibration data
	#define GYRO_BIAS 	{	  -24		*	SCALE_GYRO_OFFSET, \
							    0		*	SCALE_GYRO_OFFSET, \
							    1		*	SCALE_GYRO_OFFSET}
	#define ACCEL_BIAS 	{	 -196		*	SCALE_ACC_OFFSET, \
							  210		*	SCALE_ACC_OFFSET, \
							-1794		*	SCALE_ACC_OFFSET}	
	#define MAG_SOFTIRON_MATRIX		{ { 0.934, 0.005, 0.013 },	\
									{ 0.005, 0.948, 0.012 },	\
									{ 0.013, 0.012, 1.129 }}
	#define MAG_OFFSETS { -2.20F, -5.53F, -26.34F }
	#define MAG_FIELD_STRENTH 48.41f	
#elif SENSOR_NUM == 2

	//mpu calibration data	
	#define GYRO_BIAS 	{	  -24		*	SCALE_GYRO_OFFSET, \
							   9		*	SCALE_GYRO_OFFSET, \
							  -17		*	SCALE_GYRO_OFFSET}
	#define ACCEL_BIAS 	{	   257		*	SCALE_ACC_OFFSET, \
							  213		*	SCALE_ACC_OFFSET, \
							-1631		*	SCALE_ACC_OFFSET}	
	#define MAG_SOFTIRON_MATRIX		{ { 0.934, 0.005, 0.013 },	\
									{ 0.005, 0.948, 0.012 },	\
									{ 0.013, 0.012, 1.129 }}
	#define MAG_OFFSETS { -2.20F, -5.53F, -26.34F }
	#define MAG_FIELD_STRENTH 48.41f	
#else
	#error "SENSOR NUMBER NOT DEFINED"	
#endif

typedef struct {
	long q0;
	long q1;
	long q2;
	long q3;	
} Quaternion;

typedef struct {
	short x;
	short y;
	short z;
} Accel;

typedef struct {
	short x;
	short y;
	short z;
} Gyro;

typedef struct {
	float x;
	float y;
	float z;
} Compass;

#define X 0
#define Y 1
#define Z 2
#define XYZ 3

typedef struct {
	uint8_t sensor_num;
	uint32_t event;	
	short gyro[XYZ], accel[XYZ], sensors;
	float compass[XYZ];
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
#define RAW_1G_REFERENCE (uint16_t) 16383
extern const long accel_bias[3];
extern long gyro_bias[3];


int imu_init(void);
void imu_self_test(void);
int imu_init_madgwick(void);
int imu_start(void);
int imu_stop(void);
void imu_get_data(Motion *motion);
void imu_get_compass(Motion *motion);
void imu_send_to_mpl(Motion *motion);
void imu_log_data(Motion *motion);
void imu_log_motion_cal(Motion *motion);


#endif
