#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "twi_interface.h"
#include "config.h"
#include "imu_cal.h"


/* MOTION CONFIG */
#define MOTION_SAMPLE_INTERVAL_MS					1000/MOTION_SAMPLE_RATE_HZ
#define COMPASS_SAMPLE_INTERVAL_MS 				1000/COMPASS_SAMPLE_RATE_HZ

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

typedef enum {
	MIN_SAMPLE_RATE = 0x00,
	_1_HZ = MIN_SAMPLE_RATE,
	_2_HZ,
	_5_HZ,
	_10_HZ,
	_20_HZ,
	_50_HZ,
	_100_HZ,
	MAX_SAMPLE_RATE = _100_HZ,
} SAMPLE_RATE;

typedef uint8_t SensorConfig;

typedef struct {
	SensorConfig sensor_config;
	void (*event_cb) (void * p_context);

} MotionInit;

typedef struct {
	uint16_t num_of_samples;
	volatile uint32_t sample_num;
	int accel_sum[XYZ];
	int gyro_sum[XYZ];
} ImuCal;

typedef struct {
	void (*motion_event_cb) (void * p_context);
	uint8_t compass_ready;	
	SAMPLE_RATE motion_sample_rate;
	SAMPLE_RATE compass_sample_rate;
	ImuCal * p_imu_cal;
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


int motion_init(MotionInit * motion_init_s);

int motion_start(void);
int motion_stop(void);

void motion_sample(void * p_context);
void motion_sample_schedule_cb(void * p_context, uint16_t len);
	
void motion_set_sample_rate(SAMPLE_RATE sample_rate);
SAMPLE_RATE motion_get_sample_rate(void);

void compass_set_sample_rate(SAMPLE_RATE sample_rate);
SAMPLE_RATE compass_get_sample_rate(void);

void motion_run_imu_cal(uint16_t num_of_samples);

void mpu_self_test(void);



#endif
