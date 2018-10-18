
#ifndef CONFIG_H
#define CONFIG_H



								
#define SENSOR_NUM 2
#define IMU_SAMPLE_RATE_HZ 1
							
								
#if SENSOR_NUM == 1

	//mpu calibration data
	#define GYRO_BIAS 	{-21*SCALE_GYRO_OFFSET, \
						  11*SCALE_GYRO_OFFSET, \
						 -26*SCALE_GYRO_OFFSET}	
	#define ACCEL_BIAS 	{42*SCALE_ACC_OFFSET, \
						133*SCALE_ACC_OFFSET, \
						-1287*SCALE_ACC_OFFSET}	
	#define MAG_SOFTIRON_MATRIX		{ { 0.934, 0.005, 0.013 },	\
									{ 0.005, 0.948, 0.012 },	\
									{ 0.013, 0.012, 1.129 }}
	#define MAG_OFFSETS { -2.20F, -5.53F, -26.34F }
	#define MAG_FIELD_STRENTH 48.41f

	
#elif SENSOR_NUM == 2

	//mpu calibration data	
	#define GYRO_BIAS 	{-21*SCALE_GYRO_OFFSET, \
						  11*SCALE_GYRO_OFFSET, \
						 -26*SCALE_GYRO_OFFSET}
	#define ACCEL_BIAS 	{38*SCALE_ACC_OFFSET, \
						120*SCALE_ACC_OFFSET, \
						-1752*SCALE_ACC_OFFSET}	
	#define MAG_SOFTIRON_MATRIX		{ { 0.934, 0.005, 0.013 },	\
									{ 0.005, 0.948, 0.012 },	\
									{ 0.013, 0.012, 1.129 }}
	#define MAG_OFFSETS { -2.20F, -5.53F, -26.34F }
	#define MAG_FIELD_STRENTH 48.41f
	
#else
	
	#warning "SENSOR NUMBER NOT DEFINED"
	#define GYRO_BIAS  {0, 0, 0}
	#define ACCEL_BIAS {0, 0, 0}
	#define MAG_SOFTIRON_MATRIX		{{1, 0, 0 },	\
									{ 0, 1, 0 },	\
									{ 0, 0, 1 }}	
	#define MAG_OFFSETS { 0f, 0f, 0f }
	#define MAG_FIELD_STRENTH 1.0f
	//#define MAG_FIELD_STRENTH 1f;
	
#endif
	
	
	
#endif

