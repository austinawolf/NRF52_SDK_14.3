
#ifndef CONFIG_H
#define CONFIG_H

//ble config
#define DEV_MODE

//motion config
#define RUN_CAL 0
#define SENSOR_NUM 1
#define MOTION_SAMPLE_RATE_HZ _1_HZ
#define COMPASS_SAMPLE_RATE_HZ _1_HZ
#define MPU_INTERAL_RATE 100
#define INV_QUAT_SAMPLE_RATE_uS 10000
//#define MOTION_SIM_MODE

#define DEFAULT_SENSOR_CONFIG	  SENSOR_SAMPLE_QUATERNION \
								| SENSOR_SAMPLE_RAW_IMU \
								| SENSOR_COMPASS \
								| SENSOR_USE_GYRO_CAL


#endif

