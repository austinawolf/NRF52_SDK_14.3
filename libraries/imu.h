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


typedef uint8_t SensorConfigReg;

typedef enum 
{
	SAMPLE_QUATERNION,
	SAMPLE_ACCEL,
	SAMPLE_GYRO,
	SAMPLE_COMPASS,
	
} SENSOR_CONFIG_BIT_DEF;
#define ON 1
#define OFF 0
	
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
