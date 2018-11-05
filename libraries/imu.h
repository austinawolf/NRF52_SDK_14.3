#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"
#include "config.h"
#include "imu_cal.h"

/* IMU CONFIG */
#define IMU_SAMPLE_PERIOD_MS (uint32_t) 1000/IMU_SAMPLE_RATE_HZ
#define INV_QUAT_SAMPLE_RATE 10000
#define MAG_USE_CAL

/* IMU CONVERSIONS */
#define RAW_GYRO_TO_RADS (float) 2000.0f*2.0f/0xFFFFf * 3.14f/180.0f
#define RAW_ACCEL_TO_GS (float) 2.0f * 2.0f/0xFFFFf
#define RAW_MAG_TO_uT (float) 4800.0f*2.0f/(0x4000f)

#define X 0
#define Y 1
#define Z 2
#define XYZ 3

#define ON 1
#define OFF 0

// Sensor Config bit masks
#define SENSOR_SAMPLE_QUATERNION    (1<<0)
#define SENSOR_SAMPLE_RAW_IMU  		(1<<1)
#define SENSOR_COMPASS         		(1<<2)
#define SENSOR_USE_GYRO_CAL      	(1<<3)
//#define UNUSED					(1<<4)
//#define UNUSED					(1<<5)
//#define UNUSED					(1<<6)
//#define UNUSED					(1<<7)

typedef uint8_t SensorConfig;

typedef struct {
	SensorConfig sensor_config;
	void (*motion_cb[4]) (void * p_context);
	void (*compass_cb[4]) (void * p_context);
} MotionConfig;


typedef struct {
	uint32_t event;	
	short gyro[XYZ], accel[XYZ], sensors;
	unsigned char more;
	long quat[4];
	unsigned long timestamp;
	int8_t status;
} Motion;

typedef union {
	short s;
	float f;
} Compass;

typedef struct {
	Compass compass[XYZ];
	unsigned long timestamp;
	unsigned char status;
} CompassSample;

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


int imu_init(MotionConfig * motion_config);

void imu_get_data(Motion *motion);
void imu_get_compass(CompassSample *compass_sample);

void imu_motion_to_mpl(Motion *motion);
void imu_compass_to_mpl(CompassSample * compass_sample);

int imu_start(void);
int imu_stop(void);

void imu_self_test(void);
int imu_init_madgwick(void);



void imu_log_data(Motion *motion);
void imu_log_motion_cal(Motion *motion);


#endif
