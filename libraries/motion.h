#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"
#include "config.h"
#include "imu_cal.h"


#define ORIENTATION_MEAS_INTERVAL_MS			1000/IMU_SAMPLE_RATE_HZ         /**< Heart rate measurement interval (ticks). */
#define COMPASS_MEAS_INTERVAL_MS 				1000/COMPASS_SAMPLE_RATE_HZ        /**< Heart rate measurement interval (ticks). */


/* MOTION CONFIG */
#define MOTION_SAMPLE_PERIOD_MS (uint32_t) 1000/MOTION_SAMPLE_RATE_HZ
#define INV_QUAT_SAMPLE_RATE 10000

/* IMU CONVERSIONS */
#define RAW_GYRO_TO_RADS (float) 2000.0f*2.0f/0xFFFFf * 3.14f/180.0f
#define RAW_ACCEL_TO_GS (float) 2.0f * 2.0f/0xFFFFf

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

//Motion Sample Data flags bit masks
#define QUATERNION_DATA    			(1<<0)
#define IMU_DATA  					(1<<1)
#define COMPASS_DATA   				(1<<2)
#define TIMESTAMP_DATA      		(1<<3)
//#define UNUSED					(1<<4)
//#define UNUSED					(1<<5)
//#define UNUSED					(1<<6)
#define ERROR						(1<<7)


typedef uint8_t SensorConfig;

typedef struct {
	SensorConfig sensor_config;
	void (*event_cb) (void * p_context);

} MotionInit;

typedef struct {
	void (*motion_event_cb) (void * p_context);
	uint8_t compass_ready;	
	uint32_t motion_sample_interval;
	uint32_t compass_sample_interval;	
} Motion;

typedef struct {
	uint32_t event;	
	uint32_t timestamp;
	int16_t gyro[XYZ], accel[XYZ], compass[XYZ];
	int32_t quat[4];
	uint8_t data_flags;	
	int8_t status;
} MotionSample;

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


int imu_init(MotionInit * motion_init);


int imu_start(void);
int imu_stop(void);

void imu_self_test(void);



#endif
