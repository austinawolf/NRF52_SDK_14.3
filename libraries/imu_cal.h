#ifndef IMU_CAL_H_
#define IMU_CAL_H_

#include <stddef.h>
#include <stdio.h>
#include "config.h"

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
	#define ACCEL_BIAS 	{	 -248		*	SCALE_ACC_OFFSET, \
							  419		*	SCALE_ACC_OFFSET, \
							-1825		*	SCALE_ACC_OFFSET}	
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
	#define ACCEL_BIAS 	{	  257		*	SCALE_ACC_OFFSET, \
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



#endif
